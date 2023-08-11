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

/// Turn async methods into synchronous ones.
class BlockingCallback {
 public:
  /// Pass the result of this to a singular async call.
  CompletionCallback callback() {
    return [&](int v) {
      std::unique_lock lock(mutex_);
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

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) override {
    std::unique_lock lock(something_mutex_);
    work_ = std::bind(&Fdcanusb::CHILD_Cycle,
                      this, frames, size, replies, completed_callback);
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

  void CHILD_Cycle(const CanFdFrame* frames,
                   size_t size,
                   std::vector<CanFdFrame>* replies,
                   CompletionCallback completed_callback) {
    if (replies) { replies->clear(); }
    for (size_t i = 0; i < size; i++) {
      CHILD_CheckReplies(replies, kNoWait, 0, 0);
      CHILD_SendCanFdFrame(frames[i]);
      CHILD_CheckReplies(replies,
                         kWait,
                         1,
                         frames[i].reply_required ? 1 : 0);
    }
    completed_callback(0);
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

  void CHILD_SendCanFdFrame(const CanFdFrame& frame) {
    char buf[256] = {};
    size_t pos = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    auto fmt = [&](const char* fmt, auto ...args) {
      pos += snprintf(&buf[pos], sizeof(buf) - pos - 1, fmt, args...);
    };
#pragma GCC diagnostic pop

    fmt("can send %04x ", frame.arbitration_id);
    for (size_t i = 0; i < frame.size; i++) {
      fmt("%02x", static_cast<int>(frame.data[i]));
    }

    if (options_.disable_brs || frame.brs == CanFdFrame::kForceOff) {
      fmt(" b");
    } else if (frame.brs == CanFdFrame::kForceOn) {
      fmt(" B");
    }
    if (frame.fdcan_frame == CanFdFrame::kForceOff) {
      fmt(" f");
    } else if (frame.fdcan_frame == CanFdFrame::kForceOn) {
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


/// This is the primary interface to a moteus controller.  One
/// instance of this class should be created per controller that is
/// commanded or monitored.
///
/// The primary control functions each have 3 possible forms:
///
///  1. A "Make" variant which constructs a CanFdFrame to be used in a
///     later call to Transport::Cycle.
///
///  2. A "Set" variant which sends a command to the controller and
///     waits for a response in a blocking manner.
///
///  3. An "Async" variant which starts the process of sending a
///     command and potentially waiting for a response.  When this
///     operation is finished, a user-provided callback is invoked.
///     This callback may be called either:
///      a) from an arbitrary thread
///      b) recursively from the calling thread before returning
///
/// While any async operation is outstanding, it is undefined behavior
/// to start another async operation or execute a blocking operation.
class Controller {
 public:
  struct Options {
    // The ID of the servo to communicate with.
    int id = 1;

    // The source ID to use for the commanding node (i.e. the host or
    // master).
    int source = 0;

    // Which CAN bus to send commands on and look for responses on.
    // This may not be used on all transports.
    int bus = 0;

    // For each possible primary command, the resolution for all
    // command fields is fixed at construction time.  If the
    // resolution needs to be changed, either a separate 'Controller'
    // instance should be created, or the 'ExecuteSingleCommand'
    // formulation should be used.
    Query::Format query_format;
    PositionMode::Format position_format;
    VFOCMode::Format vfoc_format;
    CurrentMode::Format current_format;
    StayWithinMode::Format stay_within_format;

    // Use the given prefix for all CAN IDs.
    uint32_t can_prefix = 0x0000;

    // Request the configured set of registers as a query with every
    // command.
    bool default_query = true;

    double trajectory_period_s = 0.01;

    // Specify a transport to be used.  If left unset, a global common
    // transport will be constructed to be shared with all Controller
    // instances in this process.  That will attempt to auto-detect a
    // reasonable transport on the system.
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
    CanFdFrame frame;
    Query::Result values;
  };


  /////////////////////////////////////////
  // Query

  CanFdFrame MakeQuery() {
    return MakeFrame(EmptyMode(), {}, {});
  }

  std::optional<Result> Query() {
    return ExecuteSingleCommand(MakeQuery());
  }

  void AsyncQuery(Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeQuery(), result, callback);
  }


  /////////////////////////////////////////
  // StopMode

  CanFdFrame MakeStop() {
    return MakeFrame(StopMode(), {}, {});
  }

  std::optional<Result> SetStop() {
    return ExecuteSingleCommand(MakeStop());
  }

  void AsyncStop(Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeStop(), result, callback);
  }


  /////////////////////////////////////////
  // BrakeMode

  CanFdFrame MakeBrake() {
    return MakeFrame(BrakeMode(), {}, {});
  }

  std::optional<Result> SetBrake() {
    return ExecuteSingleCommand(MakeBrake());
  }

  void AsyncBrake(Result* result, CompletionCallback callback) {
    AsyncStartSingleCommand(MakeBrake(), result, callback);
  }


  /////////////////////////////////////////
  // PositionMode

  CanFdFrame MakePosition(const PositionMode::Command& cmd) {
    return MakeFrame(PositionMode(), cmd, options_.position_format);
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

  CanFdFrame MakeVFOC(const VFOCMode::Command& cmd) {
    return MakeFrame(VFOCMode(), cmd, options_.vfoc_format);
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

  CanFdFrame MakeCurrent(const CurrentMode::Command& cmd) {
    return MakeFrame(CurrentMode(), cmd, options_.current_format);
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

  CanFdFrame MakeStayWithin(const StayWithinMode::Command& cmd) {
    return MakeFrame(StayWithinMode(), cmd, options_.stay_within_format);
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
    BlockingCallback cbk;
    std::string response;
    AsyncDiagnosticCommand(message, &response, cbk.callback(), reply_mode);
    cbk.Wait();
    return response;
  }

  void AsyncDiagnosticCommand(const std::string& message,
                              std::string* result,
                              CompletionCallback callback,
                              DiagnosticReplyMode reply_mode = kExpectOK) {
    auto context = std::make_shared<AsyncDiagnosticCommandContext>();
    context->result = result;
    context->remaining_command = message + "\n";
    context->controller = this;
    context->callback = callback;
    context->reply_mode = reply_mode;

    context->Start();
  }

  void DiagnosticWrite(const std::string& message, int channel) {
    BlockingCallback cbk;
    AsyncDiagnosticWrite(message, channel, cbk.callback());
    cbk.Wait();
  }

  void AsyncDiagnosticWrite(const std::string& message,
                            int channel,
                            CompletionCallback callback) {
    auto context = std::make_shared<AsyncDiagnosticWriteContext>();
    context->message = message;
    context->channel = channel;
    context->controller = this;
    context->callback = callback;

    context->Start();
  }

  std::string DiagnosticRead(int channel) {
    std::string response;
    BlockingCallback cbk;
    AsyncDiagnosticRead(&response, channel, cbk.callback());
    cbk.Wait();
    return response;
  }

  void AsyncDiagnosticRead(std::string* response,
                           int channel,
                           CompletionCallback callback) {
    DiagnosticRead::Command read;
    read.channel = channel;
    read.max_length = 48;

    auto frame = DefaultFrame(kReplyRequired);
    WriteCanData write_frame(frame.data, &frame.size);
    DiagnosticRead::Make(&write_frame, read, {});

    struct Context {
      std::string* response = nullptr;
      std::vector<CanFdFrame> replies;
      CompletionCallback callback;
    };
    auto context = std::make_shared<Context>();
    context->response = response;
    context->callback = callback;

    transport()->Cycle(
        &frame, 1, &context->replies,
        [context, this](int v) {
          for (const auto& frame : context->replies) {
            if (frame.destination != options_.source ||
                frame.source != options_.id ||
                frame.can_prefix != options_.can_prefix) {
              continue;
            }
            auto maybe_data = DiagnosticResponse::Parse(frame.data, frame.size);
            *context->response = std::string(
                reinterpret_cast<const char*>(maybe_data.data), maybe_data.size);
            context->callback(0);
            return;
          }

          context->callback(ETIMEDOUT);
          return;
        });
  }

  void DiagnosticFlush(int channel = 1) {
    // Read until nothing is left.
    while (true) {
      const auto response = DiagnosticRead(channel);
      if (response.empty()) { return; }
    }
  }


  /////////////////////////////////////////
  // Schema version checking

  CanFdFrame MakeSchemaVersionQuery() {
    GenericQuery::Format query;
    query.values[0].register_number = Register::kRegisterMapVersion;
    query.values[0].resolution = kInt32;

    return MakeFrame(GenericQuery(), {}, query);
  }

  void VerifySchemaVersion() {
    const auto result = ExecuteSingleCommand(MakeSchemaVersionQuery());
    if (!result) {
      throw std::runtime_error("No response to schema version query");
    }
    CheckRegisterMapVersion(*result);
  }

  void AsyncVerifySchemaVersion(CompletionCallback callback) {
    auto result = std::make_shared<Result>();

    AsyncStartSingleCommand(
        MakeSchemaVersionQuery(),
        result.get(),
        [result, callback](int value) {
          CheckRegisterMapVersion(*result);
          callback(value);
        });
  }

  //////////////////////////////////////////////////

  std::optional<Result> ExecuteSingleCommand(const CanFdFrame& cmd) {
    std::vector<CanFdFrame> replies;

    transport()->BlockingCycle(&cmd, 1, &replies);

    return FindResult(replies);
  }

  void AsyncStartSingleCommand(const CanFdFrame& cmd,
                               Result* result,
                               CompletionCallback callback) {
    auto context = std::make_shared<std::vector<CanFdFrame>>();
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
  // A helper context to maintain asynchronous state while performing
  // diagnostic channel commands.
  struct AsyncDiagnosticCommandContext
      : public std::enable_shared_from_this<AsyncDiagnosticCommandContext> {
    std::string* result = nullptr;
    CompletionCallback callback;
    DiagnosticReplyMode reply_mode = {};

    Controller* controller = nullptr;

    std::vector<CanFdFrame> replies;

    std::string remaining_command;
    std::string current_line;
    std::ostringstream output;

    void Start() {
      DoWrite();
    }

    void Callback(int error) {
      if (error != 0) {
        callback(error);
        return;
      }

      if (ProcessReplies()) {
        callback(0);
        return;
      }

      if (remaining_command.size()) {
        DoWrite();
      } else {
        DoRead();
      }
    }

    void DoWrite() {
      DiagnosticWrite::Command write;
      write.data = remaining_command.data();
      const auto to_write = std::min<size_t>(48, remaining_command.size());
      write.size = to_write;

      auto frame = controller->DefaultFrame(kNoReply);
      WriteCanData write_frame(frame.data, &frame.size);
      DiagnosticWrite::Make(&write_frame, write, {});

      controller->transport()->Cycle(
          &frame, 1, nullptr,
          [s=shared_from_this(), to_write](int v) {
            s->remaining_command = s->remaining_command.substr(to_write);
            s->Callback(v);
          });
    }

    void DoRead() {
      DiagnosticRead::Command read;
      auto frame = controller->DefaultFrame(kReplyRequired);
      WriteCanData write_frame(frame.data, &frame.size);
      DiagnosticRead::Make(&write_frame, read, {});

      controller->transport()->Cycle(
          &frame, 1, &replies,
          [s=shared_from_this()](int v) {
            s->Callback(v);
          });
    }

    bool ProcessReplies() {
      for (const auto& reply : replies) {
        if (reply.source != controller->options_.id ||
            reply.can_prefix != controller->options_.can_prefix) {
          continue;
        }

        const auto parsed = DiagnosticResponse::Parse(reply.data, reply.size);
        if (parsed.channel != 1) { continue; }

        current_line += std::string(
            reinterpret_cast<const char*>(parsed.data), parsed.size);
      }

      size_t first_newline = std::string::npos;
      while ((first_newline = current_line.find_first_of("\r\n"))
             != std::string::npos) {
        const auto this_line = current_line.substr(0, first_newline);
        if (reply_mode == kExpectSingleLine) {
          *result = this_line;
          return true;
        } else if (this_line == "OK") {
          *result = output.str();
          return true;
        } else {
          output.write(current_line.data(), first_newline + 1);
          current_line = current_line.substr(first_newline + 1);
        }
      }
      replies.clear();
      return false;
    }
  };

  struct AsyncDiagnosticWriteContext
      : public std::enable_shared_from_this<AsyncDiagnosticWriteContext> {
    std::string message;
    int channel = 0;
    Controller* controller = nullptr;
    CompletionCallback callback;

    void Start() {
      DoWrite();
    }

    void DoWrite() {
      DiagnosticWrite::Command write;
      write.channel = channel;
      write.data = message.data();
      const auto to_write = std::min<size_t>(48, message.size());
      write.size = to_write;

      auto frame = controller->DefaultFrame(kNoReply);
      WriteCanData write_frame(frame.data, &frame.size);
      DiagnosticWrite::Make(&write_frame, write, {});

      controller->transport()->Cycle(
          &frame, 1, nullptr,
          [s=shared_from_this(), to_write](int v) {
            s->message = s->message.substr(to_write);
            s->Callback(v);
          });
    }

    void Callback(int v) {
      if (message.empty()) {
        callback(v);
      } else {
        DoWrite();
      }
    }
  };

  static void CheckRegisterMapVersion(const Result& result) {
    if (result.values.extra[0].register_number !=
        Register::kRegisterMapVersion) {
      throw std::runtime_error("Malformed response to schema version query");
    }

    const auto int_version = static_cast<int>(result.values.extra[0].value);
    if (kCurrentRegisterMapVersion != int_version) {
      std::ostringstream ostr;
      ostr << "Register map version mismatch device is "
           << int_version
           << " but library requires "
           << kCurrentRegisterMapVersion;

      throw std::runtime_error(ostr.str());
    }
  }

  std::optional<Result> FindResult(const std::vector<CanFdFrame>& replies) const {
    // Pick off the last reply we got from our target ID.
    for (auto it = replies.rbegin(); it != replies.rend(); ++it) {
      if (it->source == options_.id &&
          it->destination == options_.source &&
          it->can_prefix == options_.can_prefix) {

        Result result;
        result.frame = *it;
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

  CanFdFrame DefaultFrame(ReplyMode reply_mode = kReplyRequired) {
    CanFdFrame result;
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
  CanFdFrame MakeFrame(const CommandType&,
                       const typename CommandType::Command& cmd,
                       const typename CommandType::Format& fmt) {
    auto result = DefaultFrame(
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
