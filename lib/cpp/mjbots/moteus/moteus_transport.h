// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fcntl.h>
#include <glob.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/serial.h>
#include <net/if.h>
#include <poll.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus_protocol.h"
#include "moteus_tokenizer.h"

namespace mjbots {
namespace moteus {

using CompletionCallback = std::function<void(int /* errno */)>;

/// Turn async methods into synchronous ones.
class BlockingCallback {
 public:
  /// Pass the result of this to a singular async call.
  CompletionCallback callback() {
    return [&](int v) {
      std::unique_lock<std::recursive_mutex> lock(mutex_);
      done_.store(true);
      result_.store(v);
      cv_.notify_one();
    };
  }

  /// Then call this to perform the blocking.
  int Wait() {
    cv_.wait(lock_, [&]() { return done_.load(); });
    return result_.load();
  }

 private:
  std::atomic<bool> done_{false};
  std::atomic<int> result_{0};
  std::recursive_mutex mutex_;
  std::condition_variable_any cv_;
  std::unique_lock<std::recursive_mutex> lock_{mutex_};
};

class Transport {
 public:
  virtual ~Transport() {}

  /// Start sending all the frames in the @p frames / @p size list.
  ///
  /// Any replies are collated in the @p replies result.
  ///
  /// Upon completion, invoke @p completed_callback from an arbitrary
  /// thread that may or may not be the calling one and may or may not
  /// be invoked rentrantly from within this call.
  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) = 0;

  /// The same operation as above, but block until it is complete.
  ///
  /// A default implementation is provided which delegates to the
  /// asynchronous version.
  virtual void BlockingCycle(const CanFdFrame* frames,
                             size_t size,
                             std::vector<CanFdFrame>* replies) {
    BlockingCallback cbk;

    this->Cycle(frames, size, replies, cbk.callback());

    cbk.Wait();
  }

  /// Schedule the given callback to be invoked at a later time.  This
  /// will either be invoked from an arbitrary thread, or from within
  /// another call to "Cycle or BlockingCycle".
  virtual void Post(std::function<void()> callback) = 0;
};

namespace details {
/// This is just a simple RAII class for managing file descriptors.
class FileDescriptor {
 public:
  FileDescriptor() {}
  FileDescriptor(int fd) { fd_ = fd; }
  ~FileDescriptor() {
    if (fd_ >= 0) { ::close(fd_); }
  }

  FileDescriptor& operator=(int fd) {
    if (fd_ >= 0) { ::close(fd_); }
    fd_ = fd;
    return *this;
  }

  bool operator==(const FileDescriptor& rhs) const {
    return fd_ == rhs.fd_;
  }

  operator int() const {
    return fd_;
  }

  int release() {
    const auto result = fd_;
    fd_ = -1;
    return result;
  }

 private:
  int fd_ = -1;
};

/// A basic event loop implemented using C++11 primitives.
class ThreadedEventLoop {
 public:
  ThreadedEventLoop() {
    thread_ = std::thread(std::bind(&ThreadedEventLoop::CHILD_Run, this));
  }

  ~ThreadedEventLoop() {
    {
      std::unique_lock<std::recursive_mutex> lock(mutex_);
      done_ = true;
      do_something_ = true;
      something_cv_.notify_one();
    }
    thread_.join();
  }

  // This is purely to catch out of control queues earlier, as
  // typically there will just be 1 outstanding event at a time.
  static constexpr int kMaxQueueSize = 2;

  void Post(std::function<void()> callback) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    event_queue_.push_back(std::move(callback));
    if (event_queue_.size() > kMaxQueueSize) {
      throw std::runtime_error("There should never be more than one!");
    }
    do_something_ = true;
    something_cv_.notify_one();
  }

 private:
  void CHILD_Run() {
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    while (true) {
      something_cv_.wait(lock, [&]() {
        return do_something_ || !event_queue_.empty();
      });
      do_something_ = false;

      if (done_) {
        return;
      }

      // Do at most one event.
      if (!event_queue_.empty()) {
        auto top = event_queue_.front();
        event_queue_.pop_front();
        top();
      }
    }
  }

  std::thread thread_;

  // The following variables are controlled by 'something_mutex'.
  std::recursive_mutex mutex_;
  std::condition_variable_any something_cv_;

  bool do_something_ = false;
  bool done_ = false;

  std::deque<std::function<void()>> event_queue_;
};

/// A helper base class for transports that want to manage timeout
/// behavior in a similar manner.
class TimeoutTransport : public Transport {
 public:
  struct Options {
    bool disable_brs = false;

    uint32_t min_ok_wait_ns = 1000000;
    uint32_t min_rcv_wait_ns = 5000000;

    uint32_t rx_extra_wait_ns = 5000000;

    // Send at most this many frames before waiting for responses.  -1
    // means no limit.
    int max_pipeline = -1;

    Options() {}
  };

  TimeoutTransport(const Options& options) : t_options_(options) {}

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) override {
    // The event loop should never be empty here, but we make a copy
    // just to assert that.
    auto copy = std::atomic_load(&UNPROTECTED_event_loop_);
    FailIf(!copy, "unexpected null event loop");
    copy->Post(
        std::bind(&TimeoutTransport::CHILD_Cycle,
                  this, frames, size, replies, completed_callback));
  }

  virtual void Post(std::function<void()> callback) override {
    // We might have an attempt to post an event while we are being
    // destroyed.  In that case, just ignore it.
    auto copy = std::atomic_load(&UNPROTECTED_event_loop_);
    if (copy) {
      copy->Post(callback);
    }
  }

  static int64_t GetNow() {
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
  }

  static void Fail(const std::string& message) {
    throw std::runtime_error(message);
  }

  static void FailIf(bool terminate, const std::string& message) {
    if (terminate) {
      Fail(message);
    }
  }

  static void FailIfErrno(bool terminate) {
    if (terminate) {
      Fail(::strerror(errno));
    }
  }


 protected:
  virtual int CHILD_GetReadFd() const = 0;
  virtual void CHILD_SendCanFdFrame(const CanFdFrame&) = 0;

  struct ConsumeCount {
    int rcv = 0;
    int ok = 0;
  };

  virtual ConsumeCount CHILD_ConsumeData(
      std::vector<CanFdFrame>* replies,
      int expected_ok_count,
      std::vector<int>* expected_reply_count) = 0;
  virtual void CHILD_FlushTransmit() = 0;

  void CHILD_Cycle(const CanFdFrame* frames,
                   size_t size,
                   std::vector<CanFdFrame>* replies,
                   CompletionCallback completed_callback) {
    if (replies) { replies->clear(); }
    CHILD_CheckReplies(replies, kFlush, 0, nullptr);

    const auto advance = t_options_.max_pipeline < 0 ?
        size : t_options_.max_pipeline;

    for (size_t start = 0; start < size; start += advance) {
      int expected_ok_count = 0;
      for (auto& v : expected_reply_count_) { v = 0; }

      for (size_t i = start; i < (start + advance) && i < size; i++) {
        expected_ok_count++;
        CHILD_SendCanFdFrame(frames[i]);
        if (frames[i].reply_required) {
          if ((frames[i].destination + 1) > expected_reply_count_.size()) {
            expected_reply_count_.resize(frames[i].destination + 1);
          }
          expected_reply_count_[frames[i].destination]++;
        }
      }

      CHILD_FlushTransmit();

      CHILD_CheckReplies(replies,
                         kWait,
                         expected_ok_count,
                         &expected_reply_count_);
    }

    Post(std::bind(completed_callback, 0));
  }

  enum ReadDelay {
    kWait,
    kFlush,
  };

  void CHILD_CheckReplies(std::vector<CanFdFrame>* replies,
                          ReadDelay read_delay,
                          int expected_ok_count,
                          std::vector<int>* expected_reply_count) {
    const auto start = GetNow();

    const auto any_reply_checker = [&]() {
      if (!expected_reply_count) { return false; }
      for (auto v : *expected_reply_count) {
        if (v) { return true; }
      }
      return false;
    };
    auto end_time =
        start +
        (read_delay == kWait ?
         std::max(expected_ok_count != 0 ? t_options_.min_ok_wait_ns : 0,
                  any_reply_checker() ? t_options_.min_rcv_wait_ns : 0) :
         5000);

    struct pollfd fds[1] = {};
    fds[0].fd = CHILD_GetReadFd();
    fds[0].events = POLLIN;

    int ok_count = 0;

    while (true) {
      const auto now = GetNow();
      fds[0].revents = 0;

      struct timespec tmo = {};
      const auto to_sleep_ns = std::max<int64_t>(0, end_time - now);
      tmo.tv_sec = to_sleep_ns / 1000000000;
      tmo.tv_nsec = to_sleep_ns % 1000000000;

      const int poll_ret = ::ppoll(&fds[0], 1, &tmo, nullptr);
      if (poll_ret < 0) {
        if (errno == EINTR) {
          // Go back and try again.
          continue;
        }
        FailIfErrno(true);
      }
      if (poll_ret == 0) { return; }

      const auto consume_count = CHILD_ConsumeData(
          replies, expected_ok_count, expected_reply_count);

      ok_count += consume_count.ok;

      if (read_delay != kFlush &&
          !any_reply_checker() && ok_count >= expected_ok_count) {
        // Once we have the expected number of CAN replies and OKs,
        // return immediately.
        return;
      }

      if (consume_count.rcv || consume_count.ok) {
        const auto finish_time = GetNow();
        end_time = finish_time + t_options_.rx_extra_wait_ns;
      }
    }
  }

  // This is protected, because derived classes need to delete it
  // before freeing any file descriptors.  The public methods of the
  // ThreadedEventLoop require no locking, but the shared_ptr itself
  // requires either synchronization or access using the atomic std
  // library methods.  We'll exclusively use the atomic std library
  // methods.
  std::shared_ptr<details::ThreadedEventLoop> UNPROTECTED_event_loop_ =
      std::make_shared<details::ThreadedEventLoop>();

 private:
  const Options t_options_;

  std::vector<int> expected_reply_count_;
};
}

class Fdcanusb : public details::TimeoutTransport {
 public:
  struct Options : details::TimeoutTransport::Options {
    Options() {}
  };

  // If @p device is empty, attempt to auto-detect a fdcanusb in the
  // system.
  Fdcanusb(const std::string& device_in, const Options& options = {})
      : details::TimeoutTransport(options),
        options_(options) {
    Open(device_in);
  }

  // This constructor overload is intended for use in unit tests,
  // where the file descriptors will likely be pipes.
  Fdcanusb(int read_fd, int write_fd, const Options& options = {})
      : details::TimeoutTransport(options),
        options_(options) {
    Open(read_fd, write_fd);
  }

  virtual ~Fdcanusb() {
    std::atomic_store(&UNPROTECTED_event_loop_, {});

    if (read_fd_ == write_fd_) {
      write_fd_.release();
    }
  }

  static std::string DetectFdcanusb() {
    // For now, we'll only do linux like systems.
    {
      std::ifstream inf("/dev/fdcanusb");
      if (inf.is_open()) { return "/dev/fdcanusb"; }
    }

    {
      glob_t glob_data = {};
      const int result = ::glob(
          "/dev/serial/by-id/*fdcanusb*", 0,
          nullptr,
          &glob_data);

      std::string maybe_path;

      if (result == 0 && glob_data.gl_pathc > 0) {
        maybe_path = glob_data.gl_pathv[0];
      }

      globfree(&glob_data);

      if (!maybe_path.empty()) { return maybe_path; }
    }

    return "";
  }

 private:
  void Open(const std::string& device_in) {
    std::string device = device_in;
    if (device.empty()) {
      device = DetectFdcanusb();
      if (device.empty()) {
        throw std::runtime_error("Could not detect fdcanusb");
      }
    }

    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    FailIfErrno(fd == -1);

#ifndef _WIN32
    {
      struct serial_struct serial;
      FailIfErrno(::ioctl(fd, TIOCGSERIAL, &serial) < 0);
      serial.flags |= ASYNC_LOW_LATENCY;
      FailIfErrno(::ioctl(fd, TIOCSSERIAL, &serial) < 0);

      struct termios toptions;
      FailIfErrno(::tcgetattr(fd, &toptions) < 0);

      // Turn off things that could munge our byte stream to the
      // device.
      toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      toptions.c_oflag &= ~OPOST;

      FailIfErrno(::tcsetattr(fd, TCSANOW, &toptions) < 0);
      FailIfErrno(::tcsetattr(fd, TCSAFLUSH, &toptions) < 0);
    }
#else  // _WIN32
    {
      // Windows is likely broken for many other reasons, but if we do
      // fix all the other problems, this will be necessary.
      COMMTIMEOUTS new_timeouts = {MAXDWORD, 0, 0, 0, 0};
      SetCommTimeouts(fd, &new_timeouts);
    }
#endif

    Open(fd, fd);
  }

  void Open(int read_fd, int write_fd) {
    read_fd_ = read_fd;
    write_fd_ = write_fd;
  }

  virtual int CHILD_GetReadFd() const override {
    return read_fd_;
  }

  virtual ConsumeCount CHILD_ConsumeData(
      std::vector<CanFdFrame>* replies,
      int expected_ok_count,
      std::vector<int>* expected_reply_count) override {
    // Read into our line buffer.
    const int to_read = sizeof(line_buffer_) - line_buffer_pos_;
    const int read_ret = ::read(
        read_fd_, &line_buffer_[line_buffer_pos_], to_read);
    if (read_ret < 0) {
      if (errno == EINTR || errno == EAGAIN) { return {}; }
      FailIfErrno(true);
    }
    line_buffer_pos_ += read_ret;

    const auto consume_count = CHILD_ConsumeLines(
        replies, expected_reply_count);
    if (line_buffer_pos_ >= sizeof(line_buffer_)) {
      // We overran our line buffer.  For now, just drop everything
      // and start from 0.
      line_buffer_pos_ = 0;
    }

    return consume_count;
  }

  /// Return the number of CAN frames received.
  ConsumeCount CHILD_ConsumeLines(std::vector<CanFdFrame>* replies,
                                  std::vector<int>* expected_reply_count) {
    const auto start_size = replies ? replies->size() : 0;
    ConsumeCount result;
    while (CHILD_ConsumeLine(replies, &result.ok, expected_reply_count)) {}
    result.rcv = replies ? (replies->size() - start_size) : 0;
    return result;
  }

  bool CHILD_ConsumeLine(std::vector<CanFdFrame>* replies, int* ok_count,
                         std::vector<int>* expected_reply_count) {
    const auto line_end = [&]() -> int {
      for (size_t i = 0; i < line_buffer_pos_; i++) {
        if (line_buffer_[i] == '\r' || line_buffer_[i] == '\n') { return i; }
      }
      return -1;
    }();
    if (line_end < 0) { return false; }

    CHILD_ProcessLine(std::string(&line_buffer_[0], line_end), replies,
                      ok_count, expected_reply_count);

    std::memmove(&line_buffer_[0], &line_buffer_[line_end + 1],
                 line_buffer_pos_ - line_end - 1);
    line_buffer_pos_ -= (line_end + 1);

    return true;
  }

  void CHILD_ProcessLine(const std::string& line,
                         std::vector<CanFdFrame>* replies,
                         int* ok_count,
                         std::vector<int>* expected_reply_count) {
    if (line == "OK") {
      (*ok_count)++;
      return;
    }

    // The only line we actually do anything with is a "rcv" line.
    detail::Tokenizer tokenizer(line, " ");
    const auto start = tokenizer.next();
    if (start != "rcv") { return; }
    const auto address = tokenizer.next();
    const auto data = tokenizer.next();

    if (address.empty() || data.empty()) { return; }

    CanFdFrame this_frame;
    this_frame.arbitration_id = std::stoul(address, nullptr, 16);
    this_frame.destination = this_frame.arbitration_id & 0x7f;
    this_frame.source = (this_frame.arbitration_id >> 8) & 0x7f;
    this_frame.can_prefix = (this_frame.arbitration_id >> 16);

    this_frame.size = ParseCanData(data, this_frame.data);

    while (true) {
      const auto maybe_flags = tokenizer.next();
      if (maybe_flags.empty()) { break; }
      if (maybe_flags.size() != 1) { continue; }
      for (const char c : maybe_flags) {
        if (c == 'b') { this_frame.brs = CanFdFrame::kForceOff; }
        if (c == 'B') { this_frame.brs = CanFdFrame::kForceOn; }
        if (c == 'f') { this_frame.fdcan_frame = CanFdFrame::kForceOff; }
        if (c == 'F') { this_frame.fdcan_frame = CanFdFrame::kForceOn; }
      }
    }

    if (expected_reply_count) {
      if (this_frame.source < expected_reply_count->size()) {
        (*expected_reply_count)[this_frame.source] = std::max(
            (*expected_reply_count)[this_frame.source] - 1, 0);
      }
    }

    if (replies) {
      replies->emplace_back(std::move(this_frame));
    }
  }

  struct Printer {
    Printer(char* buf, size_t capacity) : buf_(buf), capacity_(capacity) {};

    const char* buf() { return buf_; }
    size_t size() const { return pos_; }
    size_t remaining() const { return capacity_ - pos_ - 1; }

    void operator()(const char* fmt, ...) {
      va_list ap;
      va_start(ap, fmt);
      auto n = ::vsnprintf(&buf_[pos_], remaining(), fmt, ap);
      va_end(ap);
      if (n < 0) { ::abort(); }
      pos_ += n;
    };

    char* const buf_;
    size_t pos_ = 0;
    const size_t capacity_;
  };

  virtual void CHILD_SendCanFdFrame(const CanFdFrame& frame) override {
    char buf[256] = {};

    Printer p(buf, sizeof(buf));

    p("can send %04x ", frame.arbitration_id);

    const auto dlc = RoundUpDlc(frame.size);
    for (size_t i = 0; i < frame.size; i++) {
      p("%02x", static_cast<int>(frame.data[i]));
    }
    for (size_t i = frame.size; i < dlc; i++) {
      p("50");
    }

    if (options_.disable_brs || frame.brs == CanFdFrame::kForceOff) {
      p(" b");
    } else if (frame.brs == CanFdFrame::kForceOn) {
      p(" B");
    }
    if (frame.fdcan_frame == CanFdFrame::kForceOff) {
      p(" f");
    } else if (frame.fdcan_frame == CanFdFrame::kForceOn) {
      p(" F");
    }
    p("\n");

    if (p.size() > (sizeof(tx_buffer_) - tx_buffer_size_)) {
      CHILD_FlushTransmit();
    }

    std::memcpy(&tx_buffer_[tx_buffer_size_], &buf[0], p.size());
    tx_buffer_size_ += p.size();
  }

  virtual void CHILD_FlushTransmit() override {
    for (size_t n = 0; n < tx_buffer_size_; ) {
      int ret = ::write(write_fd_, &tx_buffer_[n], tx_buffer_size_ - n);
      if (ret < 0) {
        if (errno == EINTR || errno == EAGAIN) { continue; }

        FailIfErrno(true);
      } else {
        n += ret;
      }
    }
    tx_buffer_size_ = 0;
  }

  static size_t RoundUpDlc(size_t size) {
    if (size <= 8) { return size; }
    if (size <= 12) { return 12; }
    if (size <= 16) { return 16; }
    if (size <= 20) { return 20; }
    if (size <= 24) { return 24; }
    if (size <= 32) { return 32; }
    if (size <= 48) { return 48; }
    if (size <= 64) { return 64; }
    return size;
  }

  static int ParseHexNybble(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
  }

  static int ParseHexByte(const char* value) {
    const int high = ParseHexNybble(value[0]);
    if (high < 0) { return high; }
    const int low = ParseHexNybble(value[1]);
    if (low < 0) { return low; }
    return (high << 4) | low;
  }

  static int ParseCanData(const std::string& data, uint8_t* out) {
    size_t to_read = std::min<size_t>(64 * 2, data.size());
    for (size_t i = 0; i < to_read; i+= 2) {
      out[i / 2] = ParseHexByte(&data[i]);
    }
    return to_read / 2;
  }

  // This is set in the parent, then used in the child.
  const Options options_;

  // We have these scoped file descriptors first in our member list,
  // so they will only be closed after the threaded event loop has
  // been destroyed during destruction.
  details::FileDescriptor read_fd_;
  details::FileDescriptor write_fd_;

  // The following variables are only used in the child.
  char line_buffer_[4096] = {};
  size_t line_buffer_pos_ = 0;

  char tx_buffer_[4096] = {};
  size_t tx_buffer_size_ = 0;
};


class Socketcan : public details::TimeoutTransport {
 public:
  struct Options : details::TimeoutTransport::Options {
    std::string ifname = "can0";

    Options() {}
  };

  Socketcan(const Options& options)
      : details::TimeoutTransport(options),
        options_(options) {
    socket_ = Open(options_.ifname);
  }

  virtual ~Socketcan() {
    std::atomic_store(&UNPROTECTED_event_loop_, {});
  }

 private:
  static void SetNonblock(int fd) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    FailIf(flags < 0, "error getting flags");
    flags |= O_NONBLOCK;
    FailIf(::fcntl(fd, F_SETFL, flags), "error setting flags");
  }

  static int Open(const std::string& ifname) {
    const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    FailIf(fd < 0, "error opening CAN socket");

    SetNonblock(fd);

    struct ifreq ifr = {};
    std::strncpy(&ifr.ifr_name[0], ifname.c_str(),
                 sizeof(ifr.ifr_name) - 1);
    FailIf(::ioctl(fd, SIOCGIFINDEX, &ifr) < 0,
           "could not find CAN: " + ifname);

    const int enable_canfd = 1;
    FailIf(::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                        &enable_canfd, sizeof(enable_canfd)) != 0,
           "could not set CAN-FD mode");

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    FailIf(::bind(fd,
                  reinterpret_cast<struct sockaddr*>(&addr),
                  sizeof(addr)) < 0,
           "could not bind to CAN if");

    return fd;
  }

  virtual int CHILD_GetReadFd() const override {
    return socket_;
  }

  virtual void CHILD_SendCanFdFrame(const CanFdFrame& frame) override {
    struct canfd_frame send_frame = {};
    send_frame.can_id = frame.arbitration_id;
    if (send_frame.can_id >= 0x7ff) {
      // Set the frame format flag if we need an extended ID.
      send_frame.can_id |= (1 << 31);
    }
    send_frame.len = frame.size;
    std::memcpy(send_frame.data, frame.data, frame.size);

    using F = CanFdFrame;

    send_frame.flags =
        ((frame.fdcan_frame == F::kDefault ||
          frame.fdcan_frame == F::kForceOn) ? CANFD_FDF : 0) |
        (((frame.brs == F::kDefault && !options_.disable_brs) ||
          frame.brs == F::kForceOn) ? CANFD_BRS : 0);

    FailIf(::write(socket_, &send_frame, sizeof(send_frame)) < 0,
           "error writing CAN");
  }

  virtual ConsumeCount CHILD_ConsumeData(
      std::vector<CanFdFrame>* replies,
      int expected_ok_count,
      std::vector<int>* expected_reply_count) override {
    struct canfd_frame recv_frame = {};
    FailIf(::read(socket_, &recv_frame, sizeof(recv_frame)) < 0,
           "error reading CAN frame");

    CanFdFrame this_frame;
    this_frame.arbitration_id = recv_frame.can_id & 0x1fffffff;
    this_frame.destination = this_frame.arbitration_id & 0x7f;
    this_frame.source = (this_frame.arbitration_id >> 8) & 0x7f;
    this_frame.can_prefix = (this_frame.arbitration_id >> 16);

    this_frame.brs = (recv_frame.flags & CANFD_BRS) ?
        CanFdFrame::kForceOn : CanFdFrame::kForceOff;
    this_frame.fdcan_frame = (recv_frame.flags & CANFD_FDF) ?
        CanFdFrame::kForceOn : CanFdFrame::kForceOff;

    std::memcpy(this_frame.data, recv_frame.data, recv_frame.len);
    this_frame.size = recv_frame.len;

    if (expected_reply_count) {
      if (this_frame.source < expected_reply_count->size()) {
        (*expected_reply_count)[this_frame.source] = std::max(
            (*expected_reply_count)[this_frame.source] - 1, 0);
      }
    }

    if (replies) {
      replies->emplace_back(std::move(this_frame));
    }

    ConsumeCount result;
    result.ok = 1;
    result.rcv = 1;
    return result;
  }

  virtual void CHILD_FlushTransmit() override {}

  const Options options_;
  details::FileDescriptor socket_;
};


/// A factory which can create transports given an optional set of
/// commandline arguments.
class TransportFactory {
 public:
  virtual ~TransportFactory() {}

  virtual int priority() = 0;
  virtual std::string name() = 0;
  using TransportArgPair = std::pair<std::shared_ptr<Transport>,
                                     std::vector<std::string>>;
  virtual TransportArgPair make(const std::vector<std::string>&) = 0;

  struct Argument {
    std::string name;
    int nargs = 1;
    std::string help;

    bool operator<(const Argument& rhs) const {
      if (name < rhs.name) { return true; }
      if (name > rhs.name) { return false; }
      return help < rhs.help;
    }
  };

  virtual std::vector<Argument> cmdline_arguments() = 0;

  virtual bool is_args_set(const std::vector<std::string>&) = 0;
};

class FdcanusbFactory : public TransportFactory {
 public:
  virtual ~FdcanusbFactory() {}

  virtual int priority() override { return 10; }
  virtual std::string name() override { return "fdcanusb"; }

  virtual TransportArgPair make(const std::vector<std::string>& args_in) override {
    auto args = args_in;

    Fdcanusb::Options options;
    std::string device;

    {
      auto it = std::find(args.begin(), args.end(), "--can-disable-brs");
      if (it != args.end()) {
        options.disable_brs = true;
        args.erase(it);
      }
    }

    {
      auto it = std::find(args.begin(), args.end(), "--fdcanusb");
      if (it != args.end()) {
        if ((it + 1) != args.end()) {
          device = *(it + 1);
          args.erase(it, it + 2);
        } else {
          throw std::runtime_error("--fdcanusb requires a path");
        }
      }
    }

    auto result = std::make_shared<Fdcanusb>(device, options);
    return TransportArgPair(result, args);
  }

  virtual std::vector<Argument> cmdline_arguments() override {
    return {
      { "--fdcanusb", 1, "path to fdcanusb device" },
      { "--can-disable-brs", 0, "do not set BRS" },
    };
  }

  virtual bool is_args_set(const std::vector<std::string>& args) override {
    for (const auto& arg : args) {
      if (arg == "--fdcanusb") { return true; }
    }
    return false;
  }
};

class SocketcanFactory : public TransportFactory {
 public:
  virtual ~SocketcanFactory() {}

  virtual int priority() override { return 11; }
  virtual std::string name() override { return "socketcan"; }

  virtual TransportArgPair make(const std::vector<std::string>& args_in) override {
    auto args = args_in;

    Socketcan::Options options;
    std::string device;

    {
      auto it = std::find(args.begin(), args.end(), "--can-disable-brs");
      if (it != args.end()) {
        options.disable_brs = true;
        args.erase(it);
      }
    }

    {
      auto it = std::find(args.begin(), args.end(), "--socketcan-iface");
      if (it != args.end()) {
        if ((it + 1) != args.end()) {
          options.ifname = *(it + 1);
          args.erase(it, it + 2);
        } else {
          throw std::runtime_error("--socketcan-iface requires an interface name");
        }
      }
    }

    auto result = std::make_shared<Socketcan>(options);
    return TransportArgPair(result, args);
  }

  virtual std::vector<Argument> cmdline_arguments() override {
    return {
      { "--socketcan-iface", 1, "socketcan iface name" },
      { "--can-disable-brs", 0, "do not set BRS" },
    };
  }

  virtual bool is_args_set(const std::vector<std::string>& args) override {
    for (const auto& arg : args) {
      if (arg == "--socketcan-iface") { return true; }
    }
    return false;
  }
};

class TransportRegistry {
 public:
  template <typename T>
  void Register() {
    items_.push_back(std::make_shared<T>());
  }

  static TransportRegistry& singleton() {
    static TransportRegistry reg;
    return reg;
  }

  std::vector<TransportFactory::Argument> cmdline_arguments() const {
    std::vector<TransportFactory::Argument> result;
    std::set<TransportFactory::Argument> uniqifier;

    result.push_back({"--force-transport", 1,
        "force the given transport type to be used"});
    uniqifier.insert(result.back());

    for (const auto& item : items_) {
      const auto item_args = item->cmdline_arguments();
      for (const auto& arg : item_args) {
        if (uniqifier.count(arg) == 0) {
          result.push_back(arg);
          uniqifier.insert(arg);
        }
      }
    }

    return result;
  }

  TransportFactory::TransportArgPair make(const std::vector<std::string>& args_in) const {
    auto args = args_in;
    auto to_try = items_;

    std::sort(to_try.begin(), to_try.end(),
              [](const std::shared_ptr<TransportFactory>& lhs,
                 const std::shared_ptr<TransportFactory>& rhs) {
                return lhs->priority() < rhs->priority();
              });

    // Is the transport forced?
    const auto it = std::find(args.begin(), args.end(), "--force-transport");
    if (it != args.end()) {
      if ((it + 1) != args.end()) {
        to_try = {};
        const auto name_to_find = *(it + 1);
        for (auto item : items_) {
          if (item->name() == name_to_find) { to_try.push_back(item); }
        }
        args.erase(it, it + 2);
      } else {
        throw std::runtime_error("--force-transport requires an argument");
      }
    } else {
      std::vector<std::shared_ptr<TransportFactory>> options_set;
      for (auto item : items_) {
        if (item->is_args_set(args)) {
          options_set.push_back(item);
        }
      }

      if (!options_set.empty()) { to_try = options_set; }
    }

    std::string errors;
    for (auto factory : to_try) {
      try {
        auto maybe_result = factory->make(args);
        if (maybe_result.first) {
          return maybe_result;
        }
      } catch (std::runtime_error& re) {
        if (!errors.empty()) { errors += ", "; }
        errors += factory->name() + ": " + re.what();
      }
    }
    throw std::runtime_error("Unable to find a default transport: " + errors);
  }

 private:
  TransportRegistry() {
    Register<FdcanusbFactory>();
    Register<SocketcanFactory>();
  }

  std::vector<std::shared_ptr<TransportFactory>> items_;
};

}
}
