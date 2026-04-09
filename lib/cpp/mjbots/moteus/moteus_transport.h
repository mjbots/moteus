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
#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/serial.h>
#endif
#ifdef MJBOTS_MOTEUS_ENABLE_IOKIT
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#endif
#include <limits.h>
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
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus_protocol.h"
#include "moteus_tokenizer.h"

#ifdef CANFD_FDF
#define MJBOTS_MOTEUS_ENABLE_SOCKETCAN 1
#endif

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

    // Wait at least this long for the initial OK.
    uint32_t min_ok_wait_ns = 2000000;

    // And wait at least this long for any expected reply packet.
    uint32_t min_rcv_wait_ns = 50000000;

    // After we have received a reply packet, and are still expecting
    // more, wait at least this long for every new receipt.
    uint32_t rx_extra_wait_ns = 50000000;

    // And after we have received all "expected" things, wait this
    // much longer for anything "more" that might come around.
    uint32_t final_wait_ns = 50000;

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
          if ((frames[i].destination + 1) >
              static_cast<int>(expected_reply_count_.size())) {
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
         t_options_.final_wait_ns);

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


#ifdef __APPLE__
      const int poll_ret = ::poll(&fds[0], 1, static_cast<int>(to_sleep_ns / 1000000));
#else
      const int poll_ret = ::ppoll(&fds[0], 1, &tmo, nullptr);
#endif

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
    // Serial baud rate.  fdcanusb ignores this since it is a USB
    // device.  For UART connections, this must match the firmware's
    // configured baud rate.
    int baudrate = 921600;

    // Enable UART mode (non-pipelined operation with per-frame
    // retry logic).
    bool uart_mode = false;

    // Enable CRC-8 checksums.  When false, checksums can still be
    // enabled dynamically if the device responds with "ERR checksum".
    bool checksum_enabled = false;

    // Maximum number of retry attempts for UART mode.
    int max_retries = 3;

    // When true, auto-detect whether the device is an fdcanusb or
    // a UART by checking USB VID/PID.  When false or when uart_mode
    // is explicitly set, skip detection.
    bool auto_detect = true;

    Options() {}
  };

  // If @p device is empty, attempt to auto-detect a fdcanusb in the
  // system.
  Fdcanusb(const std::string& device_in, const Options& options = {})
      : details::TimeoutTransport(options),
        options_(options),
        uart_mode_(options.uart_mode),
        checksum_active_(options.checksum_enabled) {
    Open(device_in);
  }

  // This constructor overload is intended for use in unit tests,
  // where the file descriptors will likely be pipes.
  Fdcanusb(int read_fd, int write_fd, const Options& options = {})
      : details::TimeoutTransport(options),
        options_(options),
        uart_mode_(options.uart_mode),
        checksum_active_(options.checksum_enabled) {
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

  /// Returns true if the device at the given path is a known fdcanusb
  /// (USB CAN-FD adapter), false if it is likely a direct UART
  /// connection.
  static bool DetectIsFdcanusb(const std::string& device) {
#ifdef __linux__
    // Resolve symlinks to get the actual device path.
    char resolved[PATH_MAX] = {};
    if (!::realpath(device.c_str(), resolved)) {
      return false;
    }

    // Extract the tty device name (e.g. "ttyACM0" from "/dev/ttyACM0").
    std::string resolved_str(resolved);
    const auto slash = resolved_str.rfind('/');
    if (slash == std::string::npos) { return false; }
    const auto tty_name = resolved_str.substr(slash + 1);

    // Read VID/PID from sysfs.  Non-USB devices (e.g. /dev/ttyAMA0)
    // won't have these files, so we return false -- a real fdcanusb
    // is always a USB device.
    const auto read_hex = [&](const std::string& field) -> int {
      const auto path =
          "/sys/class/tty/" + tty_name + "/device/../" + field;
      std::ifstream f(path);
      if (!f.is_open()) { return -1; }
      int value = 0;
      f >> std::hex >> value;
      return f.fail() ? -1 : value;
    };

    const int vid = read_hex("idVendor");
    const int pid = read_hex("idProduct");
    if (vid < 0 || pid < 0) { return false; }

    // Known fdcanusb VID/PID pairs.
    if (vid == 0x0483 && pid == 0x5740) { return true; }
    if (vid == 0x1209 && pid == 0x2323) { return true; }

    return false;

#elif defined(MJBOTS_MOTEUS_ENABLE_IOKIT)
    return DetectIsFdcanusbIOKit(device);
#else
    // On platforms without detection support, assume fdcanusb.
    (void)device;
    return true;
#endif
  }

#ifdef MJBOTS_MOTEUS_ENABLE_IOKIT
  static bool DetectIsFdcanusbIOKit(const std::string& device) {
    // IOKit-based detection for macOS.  Consumer must link
    // -framework IOKit -framework CoreFoundation.
    io_iterator_t iter = 0;
    CFMutableDictionaryRef match =
        IOServiceMatching(kIOSerialBSDServiceValue);
    if (!match) { return true; }

    kern_return_t kr =
        IOServiceGetMatchingServices(kIOMainPortDefault, match, &iter);
    if (kr != KERN_SUCCESS) { return true; }

    bool result = true;
    io_service_t service;
    while ((service = IOIteratorNext(iter)) != 0) {
      CFTypeRef path_ref = IORegistryEntryCreateCFProperty(
          service, CFSTR(kIOCalloutDeviceKey),
          kCFAllocatorDefault, 0);
      if (!path_ref) {
        IOObjectRelease(service);
        continue;
      }

      char path_buf[PATH_MAX] = {};
      bool matched = false;
      if (CFGetTypeID(path_ref) == CFStringGetTypeID()) {
        CFStringGetCString(static_cast<CFStringRef>(path_ref),
                           path_buf, sizeof(path_buf),
                           kCFStringEncodingUTF8);
        matched = (device == path_buf);
      }
      CFRelease(path_ref);

      if (!matched) {
        IOObjectRelease(service);
        continue;
      }

      // Walk up to find USB device parent with VID/PID.
      io_service_t parent = service;
      IOObjectRetain(parent);
      while (parent != 0) {
        CFTypeRef vid_ref = IORegistryEntryCreateCFProperty(
            parent, CFSTR("idVendor"), kCFAllocatorDefault, 0);
        CFTypeRef pid_ref = IORegistryEntryCreateCFProperty(
            parent, CFSTR("idProduct"), kCFAllocatorDefault, 0);
        if (vid_ref && pid_ref) {
          int vid = 0, pid = 0;
          CFNumberGetValue(static_cast<CFNumberRef>(vid_ref),
                           kCFNumberIntType, &vid);
          CFNumberGetValue(static_cast<CFNumberRef>(pid_ref),
                           kCFNumberIntType, &pid);
          CFRelease(vid_ref);
          CFRelease(pid_ref);
          result = (vid == 0x0483 && pid == 0x5740) ||
                   (vid == 0x1209 && pid == 0x2323);
          IOObjectRelease(parent);
          IOObjectRelease(service);
          IOObjectRelease(iter);
          return result;
        }
        if (vid_ref) { CFRelease(vid_ref); }
        if (pid_ref) { CFRelease(pid_ref); }

        io_service_t next = 0;
        kr = IORegistryEntryGetParentEntry(parent, kIOServicePlane, &next);
        IOObjectRelease(parent);
        parent = (kr == KERN_SUCCESS) ? next : 0;
      }

      IOObjectRelease(service);
      break;
    }

    IOObjectRelease(iter);
    return result;
  }
#endif

  static uint8_t ComputeCrc8(const char* data, size_t len) {
    static constexpr uint8_t kCrc8Table[16] = {
        0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
        0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
    };
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
      const uint8_t b = static_cast<uint8_t>(data[i]);
      crc = kCrc8Table[((crc >> 4) ^ (b >> 4)) & 0x0f] ^ (crc << 4);
      crc = kCrc8Table[((crc >> 4) ^ (b & 0x0f)) & 0x0f] ^ (crc << 4);
    }
    return crc;
  }

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) override {
    auto copy = std::atomic_load(&UNPROTECTED_event_loop_);
    FailIf(!copy, "unexpected null event loop");

    if (!uart_mode_) {
      copy->Post(
          std::bind(&Fdcanusb::CHILD_Cycle,
                    this, frames, size, replies, completed_callback));
    } else {
      copy->Post(
          std::bind(&Fdcanusb::CHILD_CycleUart,
                    this, frames, size, replies, completed_callback));
    }
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

    if (options_.auto_detect && !options_.uart_mode) {
      if (!DetectIsFdcanusb(device)) {
        uart_mode_ = true;
        checksum_active_ = true;
      }
    }

    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    FailIfErrno(fd == -1);

#ifdef __linux__
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
      toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
      toptions.c_cflag |= (CLOCAL | CREAD);
      toptions.c_cflag &= ~CRTSCTS;

      const auto speed = BaudToSpeed(options_.baudrate);
      FailIfErrno(::cfsetispeed(&toptions, speed) < 0);
      FailIfErrno(::cfsetospeed(&toptions, speed) < 0);

      FailIfErrno(::tcsetattr(fd, TCSANOW, &toptions) < 0);
      FailIfErrno(::tcsetattr(fd, TCSAFLUSH, &toptions) < 0);
    }
#elif __APPLE__
    {
      termios toptions{};
      FailIfErrno(::tcgetattr(fd, &toptions) < 0);
      toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      toptions.c_oflag &= ~OPOST;
      toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
      toptions.c_cflag |= (CLOCAL | CREAD);
      toptions.c_cflag &= ~CRTSCTS;

      const auto speed = BaudToSpeed(options_.baudrate);
      FailIfErrno(::cfsetispeed(&toptions, speed) < 0);
      FailIfErrno(::cfsetospeed(&toptions, speed) < 0);

      FailIfErrno(::tcsetattr(fd, TCSANOW, &toptions) < 0);
      FailIfErrno(::tcsetattr(fd, TCSAFLUSH, &toptions) < 0);
    }
#elif _WIN32
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
      int /* expected_ok_count */,
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

  void CHILD_ProcessLine(const std::string& line_in,
                         std::vector<CanFdFrame>* replies,
                         int* ok_count,
                         std::vector<int>* expected_reply_count) {
    std::string line = line_in;

    // Validate and strip checksum if present.
    {
      const auto star_pos = line.rfind('*');
      if (star_pos != std::string::npos && star_pos + 3 <= line.size()) {
        const int hi = ParseHexNybble(line[star_pos + 1]);
        const int lo = ParseHexNybble(line[star_pos + 2]);
        if (hi >= 0 && lo >= 0) {
          const uint8_t claimed = static_cast<uint8_t>((hi << 4) | lo);
          const uint8_t computed = ComputeCrc8(line.data(), star_pos);
          if (claimed != computed) {
            // Invalid checksum, discard the line.
            return;
          }
          // Strip checksum suffix and trailing spaces.
          line = line.substr(0, star_pos);
          while (!line.empty() && line.back() == ' ') {
            line.pop_back();
          }
        }
      } else if (checksum_active_) {
        // In checksum mode, lines without a valid checksum are
        // discarded.
        return;
      }
    }

    if (line == "OK") {
      (*ok_count)++;
      return;
    }

    // Handle ERR responses.
    if (line.size() >= 3 && line.substr(0, 3) == "ERR") {
      if (line.find("checksum") != std::string::npos) {
        checksum_active_ = true;
      }
      // ERR is not counted as an OK -- the command was not
      // processed.  The cycle will time out and the retry logic
      // will re-send (now with checksums enabled if appropriate).
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
      if (this_frame.source <
          static_cast<int>(expected_reply_count->size())) {
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
    if (checksum_active_) {
      p(" ");
      const uint8_t crc = ComputeCrc8(buf, p.size());
      p("*%02X", crc);
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

  void CHILD_CycleUart(const CanFdFrame* frames,
                       size_t size,
                       std::vector<CanFdFrame>* replies,
                       CompletionCallback completed_callback) {
    if (replies) { replies->clear(); }

    // Flush any stale data.
    CHILD_CheckReplies(replies, kFlush, 0, nullptr);

    for (size_t i = 0; i < size; i++) {
      const auto& frame = frames[i];
      bool ok_received = false;

      int64_t timeout_ns = options_.min_ok_wait_ns;

      for (int attempt = 0; attempt <= options_.max_retries; attempt++) {
        int ok_count = 0;

        CHILD_SendCanFdFrame(frame);
        CHILD_FlushTransmit();

        // Wait for the OK acknowledgment.
        CHILD_WaitForOk(&ok_count, replies, timeout_ns);

        if (ok_count > 0) {
          ok_received = true;
          break;
        }

        // Timed out or ERR received -- increase timeout and retry.
        timeout_ns = timeout_ns * 3 / 2;
      }

      if (ok_received && frame.reply_required) {
        // Wait for the reply using the normal rcv timeout.
        std::vector<int> reply_count(frame.destination + 1, 0);
        reply_count[frame.destination] = 1;
        CHILD_CheckReplies(replies, kWait, 0, &reply_count);
      }
    }

    Post(std::bind(completed_callback, 0));
  }

  void CHILD_WaitForOk(int* ok_count,
                       std::vector<CanFdFrame>* replies,
                       int64_t timeout_ns) {
    const auto start = GetNow();
    const auto end_time = start + timeout_ns;

    struct pollfd fds[1] = {};
    fds[0].fd = CHILD_GetReadFd();
    fds[0].events = POLLIN;

    while (true) {
      const auto now = GetNow();
      fds[0].revents = 0;

      const auto to_sleep_ns = std::max<int64_t>(0, end_time - now);

      struct timespec tmo = {};
      tmo.tv_sec = to_sleep_ns / 1000000000;
      tmo.tv_nsec = to_sleep_ns % 1000000000;

#ifdef __APPLE__
      const int poll_ret = ::poll(
          &fds[0], 1, static_cast<int>(to_sleep_ns / 1000000));
#else
      const int poll_ret = ::ppoll(&fds[0], 1, &tmo, nullptr);
#endif

      if (poll_ret < 0) {
        if (errno == EINTR) { continue; }
        FailIfErrno(true);
      }
      if (poll_ret == 0) { return; }

      const auto consume_count = CHILD_ConsumeData(
          replies, 1, nullptr);
      *ok_count += consume_count.ok;
      if (*ok_count > 0) { return; }
    }
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

  static speed_t BaudToSpeed(int baudrate) {
    switch (baudrate) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
#ifdef B460800
      case 460800: return B460800;
#endif
#ifdef B500000
      case 500000: return B500000;
#endif
#ifdef B576000
      case 576000: return B576000;
#endif
#ifdef B921600
      case 921600: return B921600;
#endif
#ifdef B1000000
      case 1000000: return B1000000;
#endif
#ifdef B1500000
      case 1500000: return B1500000;
#endif
#ifdef B2000000
      case 2000000: return B2000000;
#endif
#ifdef B3000000
      case 3000000: return B3000000;
#endif
#ifdef B4000000
      case 4000000: return B4000000;
#endif
      default: return B921600;
    }
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

  // When true, non-pipelined operation with per-frame retry is used.
  bool uart_mode_ = false;

  // When true, CRC-8 checksums are appended to sent lines and
  // validated on received lines.
  bool checksum_active_ = false;
};


#ifdef MJBOTS_MOTEUS_ENABLE_SOCKETCAN
class Socketcan : public details::TimeoutTransport {
 public:
  struct Options : details::TimeoutTransport::Options {
    std::string ifname = "can0";
    bool ignore_errors = false;

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
    send_frame.len = RoundUpDlc(frame.size);
    std::memcpy(send_frame.data, frame.data, frame.size);
    if (send_frame.len != frame.size) {
      std::memset(&send_frame.data[frame.size], 0x50,
                  send_frame.len - frame.size);
    }

    using F = CanFdFrame;

    send_frame.flags =
        ((frame.fdcan_frame == F::kDefault ||
          frame.fdcan_frame == F::kForceOn) ? CANFD_FDF : 0) |
        (((frame.brs == F::kDefault && !options_.disable_brs) ||
          frame.brs == F::kForceOn) ? CANFD_BRS : 0);

    const auto write_result = ::write(socket_, &send_frame, sizeof(send_frame));
    if (!options_.ignore_errors) {
      FailIf(write_result < 0, "error writing CAN");
    }
  }

  virtual ConsumeCount CHILD_ConsumeData(
      std::vector<CanFdFrame>* replies,
      int /* expected_ok_count */,
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
      if (this_frame.source <
          static_cast<int>(expected_reply_count->size())) {
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
#endif  // MJBOTS_MOTEUS_ENABLE_SOCKETCAN

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

    Argument(const std::string& name_in,
             int nargs_in,
             const std::string& help_in)
        : name(name_in),
          nargs(nargs_in),
          help(help_in) {}
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

    {
      auto it = std::find(args.begin(), args.end(), "--fdcanusb-baudrate");
      if (it != args.end()) {
        if ((it + 1) != args.end()) {
          options.baudrate = std::stoi(*(it + 1));
          args.erase(it, it + 2);
        } else {
          throw std::runtime_error("--fdcanusb-baudrate requires a value");
        }
      }
    }

    auto result = std::make_shared<Fdcanusb>(device, options);
    return TransportArgPair(result, args);
  }

  virtual std::vector<Argument> cmdline_arguments() override {
    return {
      { "--fdcanusb", 1, "path to fdcanusb or UART device" },
      { "--fdcanusb-baudrate", 1, "serial baud rate (default 921600)" },
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

#ifdef MJBOTS_MOTEUS_ENABLE_SOCKETCAN
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
    {
      auto it = std::find(args.begin(), args.end(), "--socketcan-ignore-errors");
      if (it != args.end()) {
        options.ignore_errors = true;
        args.erase(it);
      }
    }

    auto result = std::make_shared<Socketcan>(options);
    return TransportArgPair(result, args);
  }

  virtual std::vector<Argument> cmdline_arguments() override {
    return {
      { "--socketcan-iface", 1, "socketcan iface name" },
      { "--socketcan-ignore-errors", 0, "ignore errors sending socketcan frames" },
      { "--can-disable-brs", 0, "do not set BRS" },
    };
  }

  virtual bool is_args_set(const std::vector<std::string>& args) override {
    for (const auto& arg : args) {
      if (arg == "--socketcan-iface") { return true; }
      if (arg == "--socketcan-ignore-errors") { return true; }
    }
    return false;
  }
};
#endif  // MJBOTS_MOTEUS_ENABLE_SOCKETCAN

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

    result.push_back(TransportFactory::Argument(
                         "--force-transport", 1,
                         "force the given transport type to be used"));
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
#ifdef MJBOTS_MOTEUS_ENABLE_SOCKETCAN
    Register<SocketcanFactory>();
#endif
  }

  std::vector<std::shared_ptr<TransportFactory>> items_;
};

}
}
