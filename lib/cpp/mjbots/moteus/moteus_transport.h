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
#include <linux/serial.h>
#include <poll.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
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

class Fdcanusb : public Transport {
 public:
  struct Options {
    bool disable_brs = false;

    uint32_t min_ok_wait_ns = 1000000;
    uint32_t min_rcv_wait_ns = 2000000;

    uint32_t rx_extra_wait_ns = 250000;

    Options() {}
  };

  // This is purely to catch out of control queues earlier, as
  // typically there will just be 1 outstanding event at a time.
  static constexpr int kMaxQueueSize = 2;

  // If @p device is empty, attempt to auto-detect a fdcanusb in the
  // system.
  Fdcanusb(const std::string& device_in, const Options& options = {})
      : options_(options) {
    Open(device_in);
  }

  Fdcanusb(int read_fd, int write_fd, const Options& options = {})
      : options_(options) {
    Open(read_fd, write_fd);
  }

  virtual ~Fdcanusb() {
    {
      std::unique_lock<std::recursive_mutex> lock(mutex_);
      done_ = true;
      do_something_ = true;
      something_cv_.notify_one();
    }
    thread_.join();

    if (read_fd_ != write_fd_) {
      ::close(write_fd_);
    }
    ::close(read_fd_);
  }

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) override {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    work_ = std::bind(&Fdcanusb::CHILD_Cycle,
                      this, frames, size, replies, completed_callback);
    do_something_ = true;
    something_cv_.notify_one();
  }

  virtual void Post(std::function<void()> callback) override {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    event_queue_.push_back(std::move(callback));
    if (event_queue_.size() > kMaxQueueSize) {
      throw std::runtime_error("There should never be more than one!");
    }
    do_something_ = true;
    something_cv_.notify_one();
  }

  static void Fail(const std::string& str) {
    throw std::runtime_error(str);
  }

  static void FailIfErrno(bool terminate) {
    if (terminate) {
      Fail(::strerror(errno));
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
    }
#else  // _WIN32
    {
      COMMTIMEOUTS new_timeouts = {MAXDWORD, 0, 0, 0, 0};
      SetCommTimeouts(fd, &new_timeouts);
    }
#endif

    Open(fd, fd);
  }

  void Open(int read_fd, int write_fd) {
    read_fd_ = read_fd;
    write_fd_ = write_fd;
    thread_ = std::thread(std::bind(&Fdcanusb::CHILD_Run, this));
  }

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
      if (work_) {
        work_();
        work_ = {};
      }
      // Do at most one event.
      if (!event_queue_.empty()) {
        auto top = event_queue_.front();
        event_queue_.pop_front();
        top();
      }
    }
  }

  void CHILD_Cycle(const CanFdFrame* frames,
                   size_t size,
                   std::vector<CanFdFrame>* replies,
                   CompletionCallback completed_callback) {
    if (replies) { replies->clear(); }
    CHILD_CheckReplies(replies, kNoWait, 0, 0);

    int expected_reply_count = 0;

    for (size_t i = 0; i < size; i++) {
      CHILD_SendCanFdFrame(frames[i]);
      if (frames[i].reply_required) { expected_reply_count++; }
    }

    CHILD_CheckReplies(replies,
                       kWait,
                       size,
                       expected_reply_count);
    Post(std::bind(completed_callback, 0));
  }

  enum ReadDelay {
    kNoWait,
    kWait,
  };

  void CHILD_CheckReplies(std::vector<CanFdFrame>* replies,
                          ReadDelay read_delay,
                          int expected_ok_count,
                          int expected_rcv_count) {
    const auto start = GetNow();
    auto end_time =
        start +
        (read_delay == kWait ?
         std::max(expected_ok_count != 0 ? options_.min_ok_wait_ns : 0,
                  expected_rcv_count != 0 ? options_.min_rcv_wait_ns : 0) : 0);

    struct pollfd fds[1] = {};
    fds[0].fd = read_fd_;
    fds[0].events = POLLIN;

    int ok_count = 0;
    int rcv_count = 0;

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

      // Read into our line buffer.
      const int to_read = sizeof(line_buffer_) - line_buffer_pos_;
      const int read_ret = ::read(
          read_fd_, &line_buffer_[line_buffer_pos_], to_read);
      if (read_ret < 0) {
        if (errno == EINTR || errno == EAGAIN) { continue; }
        FailIfErrno(true);
      }
      line_buffer_pos_ += read_ret;

      const auto consume_count = CHILD_ConsumeLines(replies);
      if (line_buffer_pos_ >= sizeof(line_buffer_)) {
        // We overran our line buffer.  For now, just drop everything
        // and start from 0.
        line_buffer_pos_ = 0;
      }

      rcv_count += consume_count.rcv;
      ok_count += consume_count.ok;

      if (rcv_count >= expected_rcv_count && ok_count >= expected_ok_count) {
        // Once we have the expected number of CAN replies and OKs,
        // return immediately.
        return;
      }

      if (read_delay == kWait && consume_count.rcv) {
        const auto finish_time = GetNow();
        end_time = finish_time + options_.rx_extra_wait_ns;
      }
    }
  }

  struct ConsumeCount {
    int rcv = 0;
    int ok = 0;
  };

  /// Return the number of CAN frames received.
  ConsumeCount CHILD_ConsumeLines(std::vector<CanFdFrame>* replies) {
    const auto start_size = replies ? replies->size() : 0;
    ConsumeCount result;
    while (CHILD_ConsumeLine(replies, &result.ok)) {}
    result.rcv = replies ? (replies->size() - start_size) : 0;
    return result;
  }

  bool CHILD_ConsumeLine(std::vector<CanFdFrame>* replies, int* ok_count) {
    const auto line_end = [&]() -> int {
      for (size_t i = 0; i < line_buffer_pos_; i++) {
        if (line_buffer_[i] == '\r' || line_buffer_[i] == '\n') { return i; }
      }
      return -1;
    }();
    if (line_end < 0) { return false; }

    CHILD_ProcessLine(std::string(&line_buffer_[0], line_end), replies, ok_count);

    std::memmove(&line_buffer_[0], &line_buffer_[line_end + 1],
                 line_buffer_pos_ - line_end);
    line_buffer_pos_ -= (line_end + 1);

    return true;
  }

  void CHILD_ProcessLine(const std::string& line,
                         std::vector<CanFdFrame>* replies,
                         int* ok_count) {
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

  void CHILD_SendCanFdFrame(const CanFdFrame& frame) {
    char buf[256] = {};

    Printer p(buf, sizeof(buf));

    p("can send %04x ", frame.arbitration_id);
    for (size_t i = 0; i < frame.size; i++) {
      p("%02x", static_cast<int>(frame.data[i]));
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

    for (size_t n = 0; n < p.size(); ) {
      int ret = ::write(write_fd_, &buf[n], p.size() - n);
      if (ret < 0) {
        if (errno == EINTR || errno == EAGAIN) { continue; }

        FailIfErrno(true);
      } else {
        n += ret;
      }
    }
  }

  static int64_t GetNow() {
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
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
  std::thread thread_;
  const Options options_;
  int read_fd_ = -1;
  int write_fd_ = -1;

  // The following variables are controlled by 'something_mutex'.
  std::recursive_mutex mutex_;
  std::condition_variable_any something_cv_;
  bool do_something_ = false;
  bool done_ = false;
  std::function<void()> work_;
  std::deque<std::function<void()>> event_queue_;

  // The following variables are only used in the child.
  char line_buffer_[4096] = {};
  size_t line_buffer_pos_ = 0;
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
  }

  std::vector<std::shared_ptr<TransportFactory>> items_;
};

}
}
