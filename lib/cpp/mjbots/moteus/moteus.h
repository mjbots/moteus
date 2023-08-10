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
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus_tokenizer.h"
#include "moteus_protocol.h"

namespace mjbots {
namespace moteus {

using CompletionCallback = std::function<void(int /* errno */)>;

class Transport {
 public:
  virtual ~Transport() {}

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

  /// The same operation as above, but block until it is complete.
  ///
  /// A default implementation is provided which delegates to the
  /// asynchronous version.
  virtual void BlockingCycle(const Command* commands,
                             size_t size,
                             std::vector<Command>* replies) {
    std::atomic<bool> done{false};

    std::recursive_mutex m;
    std::condition_variable_any cv;

    std::unique_lock lock(m);

    this->Cycle(commands, size, replies, [&](int) {
      std::unique_lock lock(m);
      done.store(true);
      cv.notify_one();
    });

    cv.wait(lock, [&]() { return done.load(); });
  }
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
      std::unique_lock lock(something_mutex_);
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

  virtual void Cycle(const Command* commands,
                     size_t size,
                     std::vector<Command>* replies,
                     CompletionCallback completed_callback) override {
    std::unique_lock lock(something_mutex_);
    work_ = std::bind(&Fdcanusb::CHILD_Cycle,
                      this, commands, size, replies, completed_callback);
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

 private:
  void Open(const std::string& device_in) {
    std::string device = device_in;
    if (device.empty()) {
      // TODO: win32/macos/vid/pid
      device = "/dev/fdcanusb";
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
    if (replies) { replies->clear(); }
    for (size_t i = 0; i < size; i++) {
      CHILD_CheckReplies(replies, kNoWait, 0, 0);
      CHILD_SendCommand(commands[i]);
      CHILD_CheckReplies(replies,
                         kWait,
                         1,
                         commands[i].reply_required ? 1 : 0);
    }
    completed_callback(0);
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
  ConsumeCount CHILD_ConsumeLines(std::vector<Command>* replies) {
    const auto start_size = replies ? replies->size() : 0;
    ConsumeCount result;
    while (CHILD_ConsumeLine(replies, &result.ok)) {}
    result.rcv = replies ? (replies->size() - start_size) : 0;
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

    while (true) {
      const auto maybe_flags = tokenizer.next();
      if (maybe_flags.empty()) { break; }
      if (maybe_flags.size() != 1) { continue; }
      for (const char c : maybe_flags) {
        if (c == 'b') { this_command.brs = Command::kForceOff; }
        if (c == 'B') { this_command.brs = Command::kForceOn; }
        if (c == 'f') { this_command.fdcan_frame = Command::kForceOff; }
        if (c == 'F') { this_command.fdcan_frame = Command::kForceOn; }
      }
    }

    if (replies) {
      replies->emplace_back(std::move(this_command));
    }
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

    if (options_.disable_brs || command.brs == Command::kForceOff) {
      fmt(" b");
    } else if (command.brs == Command::kForceOn) {
      fmt(" B");
    }
    if (command.fdcan_frame == Command::kForceOff) {
      fmt(" f");
    } else if (command.fdcan_frame == Command::kForceOn) {
      fmt(" F");
    }
    fmt("\n");

    for (int n = 0; n < pos; ) {
      int ret = ::write(write_fd_, &buf[n], pos - n);
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

    WriteCanData query_write(&query_frame_);
    Query::Make(&query_write, options_.query_format);
  }

  Transport* transport() {
    if (!transport_) {
      transport_ = MakeSingletonTransport();
    }

    return transport_.get();
  }

  static std::shared_ptr<Transport> MakeSingletonTransport() {
    static std::shared_ptr<Transport> g_transport;

    if (g_transport) { return g_transport; }

    // For now, we only know about trying to find a system Fdcanusb.
    g_transport = std::make_shared<Fdcanusb>("");

    return g_transport;
  }

  struct Result {
    Command can_frame;
    Query::Result values;
  };


  /////////////////////////////////////////
  // Query

  Command MakeQuery() {
    return MakeCommand(EmptyMode(), {}, {});
  }

  std::optional<Result> Query() {
    return ExecuteSingleCommand(MakeQuery());
  }

  void AsyncQuery(Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeQuery(), result, callback);
  }


  /////////////////////////////////////////
  // StopMode

  Command MakeStop() {
    return MakeCommand(StopMode(), {}, {});
  }

  std::optional<Result> SetStop() {
    return ExecuteSingleCommand(MakeStop());
  }

  void AsyncStop(Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeStop(), result, callback);
  }


  /////////////////////////////////////////
  // BrakeMode

  Command MakeBrake() {
    return MakeCommand(BrakeMode(), {}, {});
  }

  std::optional<Result> SetBrake() {
    return ExecuteSingleCommand(MakeBrake());
  }

  void AsyncBrake(Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeBrake(), result, callback);
  }


  /////////////////////////////////////////
  // PositionMode

  Command MakePosition(const PositionMode::Command& cmd) {
    return MakeCommand(PositionMode(), cmd, options_.position_format);
  }

  std::optional<Result> SetPosition(const PositionMode::Command& cmd) {
    return ExecuteSingleCommand(MakePosition(cmd));
  }

  void AsyncPosition(const PositionMode::Command& cmd,
                     Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakePosition(cmd), result, callback);
  }

  /// Repeatedly send a position command until the reported
  /// trajectory_complete flag is true.  This will always enable the
  /// default query and return the result of the final such response.
  std::optional<Result> SetTrajectory(const PositionMode::Command&) {
    return {};
  }


  /////////////////////////////////////////
  // VFOCMode

  Command MakeVFOC(const VFOCMode::Command& cmd) {
    return MakeCommand(VFOCMode(), cmd, options_.vfoc_format);
  }

  std::optional<Result> SetVFOC(const VFOCMode::Command& cmd) {
    return ExecuteSingleCommand(MakeVFOC(cmd));
  }

  void AsyncVFOC(const VFOCMode::Command& cmd,
                 Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeVFOC(cmd), result, callback);
  }


  /////////////////////////////////////////
  // CurrentMode

  Command MakeCurrent(const CurrentMode::Command& cmd) {
    return MakeCommand(CurrentMode(), cmd, options_.current_format);
  }

  std::optional<Result> SetCurrent(const CurrentMode::Command& cmd) {
    return ExecuteSingleCommand(MakeCurrent(cmd));
  }

  void AsyncCurrent(const CurrentMode::Command& cmd,
                    Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeCurrent(cmd), result, callback);
  }


  /////////////////////////////////////////
  // StayWithinMode

  Command MakeStayWithin(const StayWithinMode::Command& cmd) {
    return MakeCommand(StayWithinMode(), cmd, options_.stay_within_format);
  }

  std::optional<Result> SetStayWithin(const StayWithinMode::Command& cmd) {
    return ExecuteSingleCommand(MakeStayWithin(cmd));
  }

  void AsyncStayWithin(const StayWithinMode::Command& cmd,
                       Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeStayWithin(cmd), result, callback);
  }


  /////////////////////////////////////////
  // Diagnostic channel operations

  enum DiagnosticReplyMode {
    kExpectOK,
    kExpectSingleLine,
  };

  std::string DiagnosticCommand(const std::string& message,
                                DiagnosticReplyMode reply_mode = kExpectOK) {
    // First, write everything asked.
    std::string remaining = message + "\n";
    while (remaining.size()) {
      DiagnosticWrite::Command write;
      write.data = remaining.data();
      const auto to_write = std::min<size_t>(48, remaining.size());
      write.size = to_write;

      auto command = DefaultCommand(kNoReply);
      WriteCanData write_frame(command.data, &command.size);
      DiagnosticWrite::Make(&write_frame, write, {});

      transport()->BlockingCycle(&command, 1, nullptr);

      remaining = remaining.substr(to_write);
    }

    // Now read either until we get an OK line, or a single line
    // depending upon our criteria.
    std::ostringstream output;

    while (true) {
      DiagnosticRead::Command read;
      auto command = DefaultCommand(kReplyRequired);
      WriteCanData write_frame(command.data, &command.size);
      DiagnosticRead::Make(&write_frame, read, {});

      std::vector<Command> replies;
      transport()->BlockingCycle(&command, 1, &replies);

      for (const auto& reply : replies) {
        if (reply.source != options_.id ||
            reply.can_prefix != options_.can_prefix) {
          continue;
        }

        const auto parsed = DiagnosticResponse::Parse(reply.data, reply.size);
        if (parsed.channel != 1) { continue; }

        output.write(reinterpret_cast<const char*>(parsed.data), parsed.size);
      }

      if (reply_mode == kExpectSingleLine) {
        const auto first_newline = output.str().find_first_of("\r\n");
        if (first_newline != std::string::npos) {
          return output.str().substr(0, first_newline);
        }
      } else if (reply_mode == kExpectOK) {
        // We are looking for "[\r\n]?OK[\r\n]" to determine what to
        // consider done.
        const auto str = output.str();

        // TODO: We could only look at the end to avoid the O(n^2)
        // nature of this stupid approach.

        for (size_t i = 0; i + 3 < str.size(); i++) {
          if ((i + 4 < str.size() &&
               ((str[i] == '\r' || str[i] == '\n') &&
                str.substr(i + 1, 2) == "OK" &&
                (str[i + 3] == '\r' || str[i + 3] == '\n'))) ||
              (str.substr(i, 2) == "OK" &&
               (str[i + 2] == '\r' || str[i + 2] == '\n'))) {
            return str.substr(0, i + 1);
          }
        }
      }
    }
  }

  void DiagnosticFlush() {
    // TODO
  }


  //////////////////////////////////////////////////

  std::optional<Result> ExecuteSingleCommand(const Command& cmd) {
    std::vector<Command> replies;

    transport()->BlockingCycle(&cmd, 1, &replies);

    return FindResult(replies);
  }

  void AsyncStartSingleCommand(const Command& cmd,
                               Result* result,
                               CompletionCallback callback) {
    auto context = std::make_shared<std::vector<Command>>();
    transport()->Cycle(
        &cmd,
        1,
        context.get(),
        [context, callback, result, this](int error) {
          auto maybe_result = this->FindResult(*context);
          if (maybe_result) { *result = *maybe_result; }
          callback(!options_.default_query ? 0 :
                   !!maybe_result ? 0 : ETIMEDOUT);
        });
  }

 private:
  std::optional<Result> FindResult(const std::vector<Command>& replies) const {
    // Pick off the last reply we got from our target ID.
    for (auto it = replies.rbegin(); it != replies.rend(); ++it) {
      if (it->source == options_.id &&
          it->destination == options_.source &&
          it->can_prefix == options_.can_prefix) {

        Result result;
        result.can_frame = *it;
        result.values = Query::Parse(it->data, it->size);
        return result;
      }
    }

    // We didn't get anything.
    return {};
  }

  enum ReplyMode {
    kNoReply,
    kReplyRequired,
  };

  Command DefaultCommand(ReplyMode reply_mode = kReplyRequired) {
    Command result;
    result.destination = options_.id;
    result.reply_required = (reply_mode == kReplyRequired);

    result.arbitration_id =
        (result.destination) |
        (result.source << 8) |
        (result.reply_required ? 0x8000 : 0x0000) |
        (options_.can_prefix << 16);
    result.bus = options_.bus;

    return result;
  }

  template <typename CommandType>
  Command MakeCommand(const CommandType&,
                      const typename CommandType::Command& cmd,
                      const typename CommandType::Format& fmt) {
    auto result = DefaultCommand(
        options_.default_query ? kReplyRequired : kNoReply);

    WriteCanData write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (options_.default_query) {
      std::memcpy(&result.data[result.size],
                  &query_frame_.data[0],
                  query_frame_.size);
      result.size += query_frame_.size;
    }

    return result;
  }

  const Options options_;
  std::shared_ptr<Transport> transport_;
  CanData query_frame_;
};


}  // namespace moteus
}  // namespace mjbots
