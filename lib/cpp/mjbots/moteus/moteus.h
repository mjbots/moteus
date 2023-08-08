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

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus_tokenizer.h"
#include "moteus_protocol.h"

namespace mjbots {
namespace moteus {

using CompletionCallback = std::function<void()>;

/// This is a single CAN-FD frame, its headers, and other associated
/// meta-data, like which bus the message was sent or received from in
/// multi-bus systems.
struct Command {
  int destination = 1;
  int source = 0;
  bool reply_required = false;
  uint8_t data[64] = {};
  uint8_t size = 0;

  uint16_t can_prefix = 0x0000;  // A 13 bit CAN prefix

  // If true, then the ID used is not calculated from destination and
  // source, but is instead determined directly from arbitration_id.
  bool raw = false;

  uint32_t arbitration_id = 0;
  int bus = 0;
};

class TransportImpl {
 public:
  virtual ~TransportImpl() {}

  /// Start sending all the commands in the @p commands / @p size list.
  ///
  /// Any replies are collated in the @p replies result.
  ///
  /// Upon completion, invoke @p completed_callback from an arbitrary
  /// thread that may or may not be the calling one and may or may not
  /// be invoked rentrantly from within this call.
  virtual void Cycle(const Command* commands,
                     size_t size,
                     std::vector<Command>* replies,
                     CompletionCallback completed_callback) = 0;
};

class Transport {
 public:
  Transport(std::shared_ptr<TransportImpl> impl) : impl_(impl) {}

  TransportImpl* impl() { return impl_.get(); }

  void Cycle(const Command* commands,
             size_t size,
             std::vector<Command>* replies,
             CompletionCallback completed_callback) {
    impl_->Cycle(commands, size, replies, completed_callback);
  }

  void BlockingCycle(const Command* commands,
                     size_t size,
                     std::vector<Command>* replies) {
    std::atomic<bool> done{false};

    std::mutex m;
    std::condition_variable cv;

    impl_->Cycle(commands, size, replies, [&]() {
      std::unique_lock lock(m);
      done.store(true);
      cv.notify_one();
    });

    std::unique_lock lock(m);
    cv.wait(lock, [&]() { return done.load(); });
  }

 private:
  std::shared_ptr<TransportImpl> impl_;
};

class Fdcanusb : public TransportImpl {
 public:
  struct Options {
    bool disable_brs = false;

    uint32_t timeout_ns = 0;

    uint32_t min_ok_wait_ns = 1000000;
    uint32_t min_rcv_wait_ns = 2000000;

    uint32_t rx_extra_wait_ns = 250000;

    Options() {}
  };

  // If @p device is empty, attempt to auto-detect a fdcanusb in the
  // system.
  Fdcanusb(const std::string& device_in, const Options& options = {})
      : options_(options) {
    std::string device = device_in;
    if (device.empty()) {
      // TODO: win32/macos/vid/pid
      device = "/dev/fdcanusb";
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    FailIfErrno(fd_ == -1);

#ifndef _WIN32
    {
      struct serial_struct serial;
      FailIfErrno(::ioctl(fd_, TIOCGSERIAL, &serial) < 0);
      serial.flags |= ASYNC_LOW_LATENCY;
      FailIfErrno(::ioctl(fd_, TIOCSSERIAL, &serial) < 0);
    }
#else  // _WIN32
    {
      COMMTIMEOUTS new_timeouts = {MAXDWORD, 0, 0, 0, 0};
      SetCommTimeouts(fd_, &new_timeouts);
    }
#endif

    thread_ = std::thread(std::bind(&Fdcanusb::CHILD_Run, this));
  }

  virtual ~Fdcanusb() {
    {
      std::unique_lock lock(something_mutex_);
      done_ = true;
      do_something_ = true;
      something_cv_.notify_one();
    }
    thread_.join();
  }

  virtual void Cycle(const Command* commands,
                     size_t size,
                     std::vector<Command>* replies,
                     std::function<void()> completed_callback) {
    std::unique_lock lock(something_mutex_);
    work_ = std::bind(&Fdcanusb::CHILD_Cycle,
                      this, commands, size, replies, completed_callback);
    do_something_ = true;
    something_cv_.notify_one();
  }

 private:
  static void Fail(const std::string& str) {
    throw std::runtime_error(str);
  }

  static void FailIfErrno(bool terminate) {
    if (terminate) {
      Fail(::strerror(errno));
    }
  }

  void CHILD_Run() {
    std::unique_lock lock(something_mutex_);

    while (true) {
      something_cv_.wait(lock, [&]() { return do_something_; });

      if (done_) {
        return;
      }
      if (work_) {
        work_();
        work_ = {};
      }
      do_something_ = false;
    }
  }

  void CHILD_Cycle(const Command* commands,
                   size_t size,
                   std::vector<Command>* replies,
                   CompletionCallback completed_callback) {
    replies->clear();
    for (size_t i = 0; i < size; i++) {
      CHILD_CheckReplies(replies, kNoWait, 0, 0);
      CHILD_SendCommand(commands[i]);
      CHILD_CheckReplies(replies,
                         kWait,
                         1,
                         commands[i].reply_required ? 1 : 0);
    }
    completed_callback();
  }

  enum ReadDelay {
    kNoWait,
    kWait,
  };

  void CHILD_CheckReplies(std::vector<Command>* replies, ReadDelay read_delay,
                          int expected_ok_count,
                          int expected_rcv_count) {
    const auto start = GetNow();
    auto end_time =
        start +
        (read_delay == kWait ?
         std::max(expected_ok_count != 0 ? options_.min_ok_wait_ns : 0,
                  expected_rcv_count != 0 ? options_.min_rcv_wait_ns : 0) : 0);

    struct pollfd fds[1] = {};
    fds[0].fd = fd_;
    fds[0].events = POLLIN;

    int ok_count = 0;
    int rcv_count = 0;

    while (true) {
      const auto now = GetNow();
      fds[0].revents = 0;

      struct timespec tmo = {};
      tmo.tv_sec = 0;
      tmo.tv_nsec = std::max<int64_t>(0, end_time - now);

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
          fd_, &line_buffer_[line_buffer_pos_], to_read);
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
  ConsumeCount CHILD_ConsumeLines(std::vector<Command>* replies) {
    const auto start_size = replies->size();
    ConsumeCount result;
    while (CHILD_ConsumeLine(replies, &result.ok)) {}
    result.rcv = replies->size() - start_size;
    return result;
  }

  bool CHILD_ConsumeLine(std::vector<Command>* replies, int* ok_count) {
    int line_end = [&]() {
      for (int i = 0; i < line_buffer_pos_; i++) {
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

  void CHILD_ProcessLine(const std::string& line, std::vector<Command>* replies,
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

    Command this_command;
    this_command.arbitration_id = std::stoul(address, nullptr, 16);
    this_command.destination = this_command.arbitration_id & 0x7f;
    this_command.source = (this_command.arbitration_id >> 8) & 0x7f;
    this_command.can_prefix = (this_command.arbitration_id >> 16);

    this_command.size = ParseCanData(data, this_command.data);

    replies->emplace_back(std::move(this_command));
  }

  void CHILD_SendCommand(const Command& command) {
    char buf[256] = {};
    size_t pos = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    auto fmt = [&](const char* fmt, auto ...args) {
      pos += snprintf(&buf[pos], sizeof(buf) - pos - 1, fmt, args...);
    };
#pragma GCC diagnostic pop

    fmt("can send %04x ", command.arbitration_id);
    for (size_t i = 0; i < command.size; i++) {
      fmt("%02x", static_cast<int>(command.data[i]));
    }

    if (options_.disable_brs) {
      fmt(" b");
    }
    fmt("\n");

    for (int n = 0; n < pos; ) {
      int ret = ::write(fd_, &buf[n], pos - n);
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
  int fd_ = -1;

  // The following variables are controlled by 'something_mutex'.
  std::mutex something_mutex_;
  std::condition_variable something_cv_;
  bool do_something_ = false;
  bool done_ = false;
  std::function<void()> work_;

  // The following variables are only used in the child.
  char line_buffer_[4096] = {};
  size_t line_buffer_pos_ = 0;
};

class Controller {
 public:
  struct Options {
    int id = 1;
    int source = 0;
    int bus = 0;

    Query::Format query_format;
    PositionMode::Format position_format;
    VFOCMode::Format vfoc_format;
    CurrentMode::Format current_format;
    StayWithinMode::Format stay_within_format;

    uint32_t can_prefix = 0x0000;
    bool default_query = true;

    double trajectory_period_s = 0.01;

    std::shared_ptr<Transport> transport;

    Options() {}
  };

  Controller(const Options& options = {}) {
    transport_ = options.transport;

    WriteCanFrame query_write(&query_frame_);
    Query::Make(&query_write, options_.query_format);
  }

  Transport* transport() {
    if (!transport_) {
      transport_ = MakeDefaultTransport();
    }

    return transport_.get();
  }

  static std::shared_ptr<Transport> MakeDefaultTransport() {
    // For now, we only know about trying to find a system Fdcanusb.
    return std::make_shared<Transport>(std::make_shared<Fdcanusb>(""));
  }

  struct Result {
    Command can_frame;
    Query::Result values;
  };

  /// Simply query the current state of the controller.
  Result Query() {
    return {};
  }

  Command MakeStop() {
    return MakeCommand(StopMode(), {}, {});
  }

  std::optional<Result> SetStop() {
    return ExecuteSingleCommand(MakeStop());
  }

  Command MakeBrake() {
    return MakeCommand(BrakeMode(), {}, {});
  }

  std::optional<Result> SetBrake() {
    return ExecuteSingleCommand(MakeBrake());
  }

  Command MakePosition(const PositionMode::Command& cmd) {
    return MakeCommand(PositionMode(), cmd, options_.position_format);
  }

  /// Send a position command, if default_query == true, then block
  /// until a response is available and return it.
  std::optional<Result> SetPosition(const PositionMode::Command& cmd) {
    return ExecuteSingleCommand(MakePosition(cmd));
  }

  /// Repeatedly send a position command until the reported
  /// trajectory_complete flag is true.  This will always enable the
  /// default query and return the result of the final such response.
  std::optional<Result> SetTrajectory(const PositionMode::Command&) {
    return {};
  }

  Command MakeVFOC(const VFOCMode::Command& cmd) {
    return MakeCommand(VFOCMode(), cmd, options_.vfoc_format);
  }

  std::optional<Result> SetVfoc(const VFOCMode::Command& cmd) {
    return ExecuteSingleCommand(MakeVFOC(cmd));
  }

  Command MakeCurrent(const CurrentMode::Command& cmd) {
    return MakeCommand(CurrentMode(), cmd, options_.current_format);
  }

  std::optional<Result> SetCurrent(const CurrentMode::Command& cmd) {
    return ExecuteSingleCommand(MakeCurrent(cmd));
  }

  Command MakeStayWithin(const StayWithinMode::Command& cmd) {
    return MakeCommand(StayWithinMode(), cmd, options_.stay_within_format);
  }

  std::optional<Result> SetStayWithin(const StayWithinMode::Command& cmd) {
    return ExecuteSingleCommand(MakeStayWithin(cmd));
  }

  std::optional<Result> ExecuteSingleCommand(const Command& cmd) {
    std::vector<Command> replies;
    transport()->BlockingCycle(&cmd, 1, &replies);
    // Pick off the last reply we got from our target ID.
    for (auto it = replies.rbegin(); it != replies.rend(); ++it) {
      if (it->source == options_.id) {

        Result result;
        result.can_frame = *it;
        result.values = Query::Parse(it->data, it->size);
        return result;
      }
    }

    // We didn't get anything.
    return {};
  }

 private:
  template <typename CommandType>
  Command MakeCommand(const CommandType&,
                      const typename CommandType::Command& cmd,
                      const typename CommandType::Format& fmt) {
    Command result;
    result.destination = options_.id;
    result.reply_required = options_.default_query;

    WriteCanFrame write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (options_.default_query) {
      std::memcpy(&result.data[result.size],
                  &query_frame_.data[0],
                  query_frame_.size);
      result.size += query_frame_.size;
    }

    result.arbitration_id =
        (result.destination) |
        (result.source << 8) |
        (result.reply_required ? 0x8000 : 0x0000) |
        (options_.can_prefix << 16);
    result.bus = options_.bus;

    return result;
  }

  const Options options_;
  std::shared_ptr<Transport> transport_;
  CanFrame query_frame_;
};


}  // namespace moteus
}  // namespace mjbots
