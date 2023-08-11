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

#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "moteus_protocol.h"
#include "moteus_tokenizer.h"
#include "moteus_transport.h"

namespace mjbots {
namespace moteus {


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
      transport_ = MakeSingletonTransport({});
    }

    return transport_.get();
  }

  static std::shared_ptr<Transport> MakeSingletonTransport(
      const std::vector<std::string>& args) {
    static std::shared_ptr<Transport> g_transport;

    if (g_transport) { return g_transport; }

    // For now, we only know about trying to find a system Fdcanusb.
    std::vector<char*> argv;
    g_transport = TransportRegistry::singleton().make(args);

    return g_transport;
  }

  struct Result {
    CanFdFrame frame;
    Query::Result values;
  };


  /////////////////////////////////////////
  // Query

  CanFdFrame MakeQuery(const Query::Format* format_override = nullptr) {
    return MakeFrame(EmptyMode(), {}, {}, format_override);
  }

  std::optional<Result> Query(const Query::Format* format_override = nullptr) {
    return ExecuteSingleCommand(MakeQuery(format_override));
  }

  void AsyncQuery(Result* result, CompletionCallback callback,
                  const Query::Format* format_override = nullptr) {
    AsyncStartSingleCommand(MakeQuery(format_override), result, callback);
  }


  /////////////////////////////////////////
  // StopMode

  CanFdFrame MakeStop(const Query::Format* query_override = nullptr) {
    return MakeFrame(StopMode(), {}, {}, query_override);
  }

  std::optional<Result> SetStop(const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeStop(query_override));
  }

  void AsyncStop(Result* result, CompletionCallback callback,
                 const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(MakeStop(query_override), result, callback);
  }


  /////////////////////////////////////////
  // BrakeMode

  CanFdFrame MakeBrake(const Query::Format* query_override = nullptr) {
    return MakeFrame(BrakeMode(), {}, {}, query_override);
  }

  std::optional<Result> SetBrake(const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeBrake(query_override));
  }

  void AsyncBrake(Result* result, CompletionCallback callback,
                  const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(MakeBrake(query_override), result, callback);
  }


  /////////////////////////////////////////
  // PositionMode

  CanFdFrame MakePosition(const PositionMode::Command& cmd,
                          const PositionMode::Format* command_override = nullptr,
                          const Query::Format* query_override = nullptr) {
    return MakeFrame(
        PositionMode(), cmd,
        (command_override == nullptr ?
         options_.position_format : *command_override),
        query_override);
  }

  std::optional<Result> SetPosition(
      const PositionMode::Command& cmd,
      const PositionMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakePosition(cmd, command_override, query_override));
  }

  void AsyncPosition(const PositionMode::Command& cmd,
                     Result* result, CompletionCallback callback,
                     const PositionMode::Format* command_override = nullptr,
                     const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(MakePosition(cmd, command_override, query_override),
                            result, callback);
  }

  /// Repeatedly send a position command until the reported
  /// trajectory_complete flag is true.  This will always enable a
  /// query and will return the result of the final such response.
  std::optional<Result> SetPositionWaitComplete(
      const PositionMode::Command& cmd,
      double period_s,
      const PositionMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    Query::Format query_format =
        query_override == nullptr ? options_.query_format : *query_override;
    query_format.trajectory_complete = kInt8;

    bool first = true;
    while (true) {
      auto maybe_result = SetPosition(cmd, command_override, &query_format);
      if (!first) {
        if (!!maybe_result && maybe_result->values.trajectory_complete) {
          return *maybe_result;
        }
      } else if (maybe_result) {
        first = false;
      }

      ::usleep(static_cast<int>(period_s * 1e6));
    }

    return {};
  }


  /////////////////////////////////////////
  // VFOCMode

  CanFdFrame MakeVFOC(const VFOCMode::Command& cmd,
                      const VFOCMode::Format* command_override = nullptr,
                      const Query::Format* query_override = nullptr) {
    return MakeFrame(
        VFOCMode(), cmd,
        command_override == nullptr ? options_.vfoc_format : *command_override,
        query_override);
  }

  std::optional<Result> SetVFOC(const VFOCMode::Command& cmd,
                                const VFOCMode::Format* command_override = nullptr,
                                const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(MakeVFOC(cmd, command_override, query_override));
  }

  void AsyncVFOC(const VFOCMode::Command& cmd,
                 Result* result, CompletionCallback callback,
                 const VFOCMode::Format* command_override = nullptr,
                 const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(MakeVFOC(cmd, command_override, query_override),
                            result, callback);
  }


  /////////////////////////////////////////
  // CurrentMode

  CanFdFrame MakeCurrent(const CurrentMode::Command& cmd,
                         const CurrentMode::Format* command_override = nullptr,
                         const Query::Format* query_override = nullptr) {
    return MakeFrame(CurrentMode(), cmd,
                     (command_override == nullptr ?
                      options_.current_format : *command_override),
                     query_override);
  }

  std::optional<Result> SetCurrent(
      const CurrentMode::Command& cmd,
      const CurrentMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakeCurrent(cmd, command_override, query_override));
  }

  void AsyncCurrent(const CurrentMode::Command& cmd,
                    Result* result, CompletionCallback callback,
                    const CurrentMode::Format* command_override = nullptr,
                    const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(MakeCurrent(cmd, command_override, query_override),
                            result, callback);
  }


  /////////////////////////////////////////
  // StayWithinMode

  CanFdFrame MakeStayWithin(
      const StayWithinMode::Command& cmd,
      const StayWithinMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    return MakeFrame(StayWithinMode(), cmd,
                     (command_override == nullptr ?
                      options_.stay_within_format : *command_override),
                     query_override);
  }

  std::optional<Result> SetStayWithin(
      const StayWithinMode::Command& cmd,
      const StayWithinMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakeStayWithin(cmd, command_override, query_override));
  }

  void AsyncStayWithin(const StayWithinMode::Command& cmd,
                       Result* result, CompletionCallback callback,
                       const StayWithinMode::Format* command_override = nullptr,
                       const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(
        MakeStayWithin(cmd, command_override, query_override), result, callback);
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

  static std::string FinalName(const std::string& name) {
    const size_t pos = name.find_last_of("/");
    if (pos != std::string::npos) { return name.substr(pos + 1); }
    return name;
  }

  static void DefaultArgProcess(int argc, char** argv) {
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) { args.push_back(argv[i]); }

    if (std::find(args.begin(), args.end(), "--help") != args.end()) {
      std::cout << "Usage: " << FinalName(args[0]) << "\n";
      auto help_strs =
          moteus::TransportRegistry::singleton().cmdline_arguments();
      help_strs.insert(help_strs.begin(),
                       {"--help", "Display this usage message"});

      int max_item = 0;
      for (const auto& item : help_strs) {
        max_item = std::max<int>(max_item, item.name.size());
      }
      max_item = std::min(30, max_item);

      for (const auto& help_item : help_strs) {
        std::cout
            << "   " << help_item.name << "  "
            << std::string(std::max<int>(0, max_item - help_item.name.size()), ' ')
            << help_item.help << "\n";
      }

      std::exit(0);
    }

    MakeSingletonTransport(args);
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
                       const typename CommandType::Format& fmt,
                       const Query::Format* query_format_override = nullptr) {
    auto result = DefaultFrame(
        options_.default_query ? kReplyRequired : kNoReply);

    WriteCanData write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (options_.default_query || query_format_override) {
      if (query_format_override == nullptr) {
        std::memcpy(&result.data[result.size],
                    &query_frame_.data[0],
                    query_frame_.size);
        result.size += query_frame_.size;
      } else {
        Query::Make(&write_frame, *query_format_override);
      }
    }

    return result;
  }

  const Options options_;
  std::shared_ptr<Transport> transport_;
  CanData query_frame_;
};


}  // namespace moteus
}  // namespace mjbots
