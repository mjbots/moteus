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
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "moteus_optional.h"
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
    // command fields has a default set at construction time.
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

    int64_t diagnostic_retry_sleep_ns = 200000;

    // Specify a transport to be used.  If left unset, a global common
    // transport will be constructed to be shared with all Controller
    // instances in this process.  That will attempt to auto-detect a
    // reasonable transport on the system.
    std::shared_ptr<Transport> transport;

    Options() {}
  };

  Controller(const Options& options = {}) : options_(options) {
    transport_ = options.transport;

    WriteCanData query_write(&query_frame_);
    query_reply_size_ = Query::Make(&query_write, options_.query_format);
  }

  const Options& options() const { return options_; }

  Transport* transport() {
    if (!transport_) {
      transport_ = MakeSingletonTransport({});
    }

    return transport_.get();
  }

  /// Make a transport given the cmdline arguments.
  static std::shared_ptr<Transport> MakeSingletonTransport(
      const std::vector<std::string>& args) {
    auto result = ArgumentProcessHelper(args);
    return result.first;
  }

  /// Require that a default global transport have already been
  /// created and return it.
  static std::shared_ptr<Transport> RequireSingletonTransport() {
    auto& g_transport = *GlobalTransport();
    if (!g_transport) {
      throw std::logic_error("Unexpectedly cannot find global transport");
    }
    return g_transport;
  }

  struct Result {
    CanFdFrame frame;
    Query::Result values;
  };


  /////////////////////////////////////////
  // Query

  CanFdFrame MakeQuery(const Query::Format* format_override = nullptr) {
    // We force there to always be an override for the query format,
    // because if we're directly asking for a query, we should get it
    // no matter the Options::default_query state.
    return MakeFrame(EmptyMode(), {}, {},
                     format_override == nullptr ?
                     &options_.query_format : format_override);
  }

  // This is prefixed "Set" despite the fact that it sets nothing
  // because (a) it is consistent with the other methods with
  // "Make/Set/Async" prefixes, and (b) it would otherwise shadow the
  // mjbots::moteus::Query structure.
  Optional<Result> SetQuery(const Query::Format* format_override = nullptr) {
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

  Optional<Result> SetStop(const Query::Format* query_override = nullptr) {
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

  Optional<Result> SetBrake(const Query::Format* query_override = nullptr) {
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

  Optional<Result> SetPosition(
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
  Optional<Result> SetPositionWaitComplete(
      const PositionMode::Command& cmd,
      double period_s,
      const PositionMode::Format* command_override = nullptr,
      const Query::Format* query_override = nullptr) {
    Query::Format query_format =
        query_override == nullptr ? options_.query_format : *query_override;
    query_format.trajectory_complete = kInt8;

    int count = 2;
    while (true) {
      auto maybe_result = SetPosition(cmd, command_override, &query_format);
      if (!!maybe_result) { count = std::max(count - 1, 0); }

      if (count == 0 &&
          !!maybe_result &&
          maybe_result->values.trajectory_complete) {
        return *maybe_result;
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

  Optional<Result> SetVFOC(const VFOCMode::Command& cmd,
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

  Optional<Result> SetCurrent(
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

  Optional<Result> SetStayWithin(
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
  // OutputNearest

  CanFdFrame MakeOutputNearest(const OutputNearest::Command& cmd,
                               const OutputNearest::Format* command_override = nullptr,
                               const Query::Format* query_override = nullptr) {
    return MakeFrame(OutputNearest(), cmd,
                     (command_override == nullptr ?
                      OutputNearest::Format() : *command_override),
                     query_override);
  }

  Optional<Result> SetOutputNearest(const OutputNearest::Command& cmd,
                                    const OutputNearest::Format* command_override = nullptr,
                                    const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakeOutputNearest(cmd, command_override, query_override));
  }

  void AsyncOutputNearest(const OutputNearest::Command& cmd,
                          Result* result, CompletionCallback callback,
                          const OutputNearest::Format* command_override = nullptr,
                          const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(
        MakeOutputNearest(cmd, command_override, query_override),
        result, callback);
  }


  /////////////////////////////////////////
  // OutputExact

  CanFdFrame MakeOutputExact(const OutputExact::Command& cmd,
                             const OutputExact::Format* command_override = nullptr,
                             const Query::Format* query_override = nullptr) {
    return MakeFrame(OutputExact(), cmd,
                     (command_override == nullptr ?
                      OutputExact::Format() : *command_override),
                     query_override);
  }

  Optional<Result> SetOutputExact(const OutputExact::Command& cmd,
                                  const OutputExact::Format* command_override = nullptr,
                                  const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakeOutputExact(cmd, command_override, query_override));
  }

  void AsyncOutputExact(const OutputExact::Command& cmd,
                        Result* result, CompletionCallback callback,
                        const OutputExact::Format* command_override = nullptr,
                        const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(
        MakeOutputExact(cmd, command_override, query_override),
        result, callback);
  }


  /////////////////////////////////////////
  // RequireReindex

  CanFdFrame MakeRequireReindex(const RequireReindex::Command& cmd = {},
                                const RequireReindex::Format* command_override = nullptr,
                                const Query::Format* query_override = nullptr) {
    return MakeFrame(RequireReindex(), cmd,
                     (command_override == nullptr ?
                      RequireReindex::Format() : *command_override),
                     query_override);
  }

  Optional<Result> SetRequireReindex(const RequireReindex::Command& cmd,
                                     const RequireReindex::Format* command_override = nullptr,
                                     const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakeRequireReindex(cmd, command_override, query_override));
  }

  void AsyncRequireReindex(const RequireReindex::Command& cmd,
                           Result* result, CompletionCallback callback,
                           const RequireReindex::Format* command_override = nullptr,
                           const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(
        MakeRequireReindex(cmd, command_override, query_override),
        result, callback);
  }


  /////////////////////////////////////////
  // ClockTrim

  CanFdFrame MakeClockTrim(const ClockTrim::Command& cmd,
                           const ClockTrim::Format* command_override = nullptr,
                           const Query::Format* query_override = nullptr) {
    return MakeFrame(ClockTrim(), cmd,
                     (command_override == nullptr ?
                      ClockTrim::Format() : *command_override),
                     query_override);
  }

  Optional<Result> SetClockTrim(const ClockTrim::Command& cmd,
                                const ClockTrim::Format* command_override = nullptr,
                                const Query::Format* query_override = nullptr) {
    return ExecuteSingleCommand(
        MakeClockTrim(cmd, command_override, query_override));
  }

  void AsyncClockTrim(const ClockTrim::Command& cmd,
                      Result* result, CompletionCallback callback,
                      const ClockTrim::Format* command_override = nullptr,
                      const Query::Format* query_override = nullptr) {
    AsyncStartSingleCommand(
        MakeClockTrim(cmd, command_override, query_override),
        result, callback);
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
    context->transport = transport();
    context->callback = callback;
    context->reply_mode = reply_mode;

    context->Start();
  }

  void DiagnosticWrite(const std::string& message, int channel = 1) {
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
    context->transport = transport();
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

    output_frame_ = DefaultFrame(kReplyRequired);
    WriteCanData write_frame(output_frame_.data, &output_frame_.size);
    DiagnosticRead::Make(&write_frame, read, {});

    struct Context {
      std::string* response = nullptr;
      std::vector<CanFdFrame> replies;
      CompletionCallback callback;
      Transport* transport = nullptr;
    };
    auto context = std::make_shared<Context>();
    context->response = response;
    context->callback = callback;
    context->transport = transport();

    context->transport->Cycle(
        &output_frame_, 1, &context->replies,
        [context, this](int) {
          std::string response;
          bool any_response = false;

          for (const auto& frame : context->replies) {
            if (frame.destination != options_.source ||
                frame.source != options_.id ||
                frame.can_prefix != options_.can_prefix) {
              continue;
            }
            auto maybe_data = DiagnosticResponse::Parse(frame.data, frame.size);
            response += std::string(
                reinterpret_cast<const char*>(maybe_data.data), maybe_data.size);
            any_response = true;
          }

          if (any_response) {
            *context->response = response;
            context->transport->Post(std::bind(context->callback, 0));
            return;
          }

          context->transport->Post(std::bind(context->callback, ETIMEDOUT));
          return;
        });
  }

  void DiagnosticFlush(int channel = 1, double timeout_s = 0.2) {
    // Read until nothing is left or the timeout hits.
    const auto timeout = timeout_s * 1000000000ll;
    const auto start = Fdcanusb::GetNow();
    auto end_time = start + timeout;

    while (true) {
      const auto response = DiagnosticRead(channel);
      const auto now = Fdcanusb::GetNow();
      if (!response.empty()) {
        // Every time we get something, bump out timeout further into
        // the future.
        end_time = now + timeout;
        continue;
      }
      if (now > end_time) {
        break;
      }
      ::usleep(options_.diagnostic_retry_sleep_ns / 1000);
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
    auto t = transport();

    AsyncStartSingleCommand(
        MakeSchemaVersionQuery(),
        result.get(),
        [result, callback, t](int value) {
          CheckRegisterMapVersion(*result);
          t->Post(std::bind(callback, value));
        });
  }

  //////////////////////////////////////////////////

  Optional<Result> ExecuteSingleCommand(const CanFdFrame& cmd) {
    single_command_replies_.resize(0);

    transport()->BlockingCycle(&cmd, 1, &single_command_replies_);

    return FindResult(single_command_replies_);
  }

  void AsyncStartSingleCommand(const CanFdFrame& cmd,
                               Result* result,
                               CompletionCallback callback) {
    auto t = transport();
    output_frame_ = cmd;
    t->Cycle(
        &output_frame_,
        1,
        &single_command_replies_,
        [callback, result, this, t](int) {
          auto maybe_result = this->FindResult(single_command_replies_);
          if (maybe_result) { *result = *maybe_result; }

          t->Post(
              std::bind(
                  callback, !options_.default_query ? 0 :
                  !!maybe_result ? 0 : ETIMEDOUT));
        });
  }

  static std::string FinalName(const std::string& name) {
    const size_t pos = name.find_last_of("/");
    if (pos != std::string::npos) { return name.substr(pos + 1); }
    return name;
  }

  /// This may be called to allow users to configure a default
  /// transport.  It handles "-h" and "--help" by printing and
  /// exiting, so is not suitable for cases where any other command
  /// line arguments need to be handled.
  static void DefaultArgProcess(int argc, char** argv) {
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) { args.push_back(argv[i]); }

    DefaultArgProcess(args);
  }

  /// The same as the above function, but accepts its arguments in a
  /// std::vector form.
  static void DefaultArgProcess(const std::vector<std::string>& args) {
    if (std::find(args.begin() + 1, args.end(), "--help") != args.end() ||
        std::find(args.begin() + 1, args.end(), "-h") != args.end()) {
      std::cout << "Usage: " << FinalName(args[0]) << "\n";
      auto help_strs =
          moteus::TransportRegistry::singleton().cmdline_arguments();
      help_strs.insert(help_strs.begin(),
                       {"--help", 0, "Display this usage message"});

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

    ArgumentProcessHelper(args);
  }

  /// Configure the default transport according to the given command
  /// line arguments.  Return the command line arguments with all
  /// processed commands removed.
  ///
  /// This is an optional call, and is only required if you want to
  /// give a user the ability to configure the default transport from
  /// the command line.
  ///
  /// "-h" and "--help" are not handled here in any way.  Thus this
  /// method can be used in applications that want to perform
  /// additional processing on the command line arguments after.
  static std::vector<std::string>
  ProcessTransportArgs(const std::vector<std::string>& args) {
    return ArgumentProcessHelper(args).second;
  }

  /// If your application wants to support configuring the default
  /// transport from the cmdline and also have application level
  /// options, this list can be used to populate --help content.
  static std::vector<TransportFactory::Argument> cmdline_arguments() {
    return TransportRegistry::singleton().cmdline_arguments();
  }

 private:
  static TransportFactory::TransportArgPair
  ArgumentProcessHelper(const std::vector<std::string>& args) {
    auto& g_transport = *GlobalTransport();
    if (g_transport) { return std::make_pair(g_transport, args); }

    auto result = TransportRegistry::singleton().make(args);
    g_transport = result.first;
    return result;
  }

  static std::shared_ptr<Transport>* GlobalTransport() {
    static std::shared_ptr<Transport> g_transport;
    return &g_transport;
  };

  // A helper context to maintain asynchronous state while performing
  // diagnostic channel commands.
  struct AsyncDiagnosticCommandContext
      : public std::enable_shared_from_this<AsyncDiagnosticCommandContext> {
    std::string* result = nullptr;
    CompletionCallback callback;
    DiagnosticReplyMode reply_mode = {};

    Controller* controller = nullptr;
    Transport* transport = nullptr;

    CanFdFrame output_frame_;
    std::vector<CanFdFrame> replies;

    int empty_replies = 0;
    std::string remaining_command;
    std::string current_line;
    std::ostringstream output;

    void Start() {
      DoWrite();
    }

    void Callback(int error) {
      if (error != 0) {
        transport->Post(std::bind(callback, error));
        return;
      }

      if (ProcessReplies()) {
        transport->Post(std::bind(callback, 0));
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

      output_frame_ = controller->DefaultFrame(kNoReply);
      WriteCanData write_frame(output_frame_.data, &output_frame_.size);
      DiagnosticWrite::Make(&write_frame, write, {});

      auto s = shared_from_this();
      controller->transport()->Cycle(
          &output_frame_, 1, nullptr,
          [s, to_write](int v) {
            s->remaining_command = s->remaining_command.substr(to_write);
            s->Callback(v);
          });
    }

    void DoRead() {
      if (empty_replies >= 5) {
        // We will call this a timeout.
        transport->Post(std::bind(callback, ETIMEDOUT));
        return;
      } else if (empty_replies >= 2) {
        // Sleep before each subsequent read.
        ::usleep(controller->options_.diagnostic_retry_sleep_ns / 1000);
      }

      DiagnosticRead::Command read;
      output_frame_ = controller->DefaultFrame(kReplyRequired);
      WriteCanData write_frame(output_frame_.data, &output_frame_.size);
      DiagnosticRead::Make(&write_frame, read, {});

      auto s = shared_from_this();

      transport->Cycle(
          &output_frame_, 1, &replies,
          [s](int v) {
            s->Callback(v);
          });
    }

    bool ProcessReplies() {
      for (const auto& reply : replies) {
        if (reply.source != controller->options_.id ||
            reply.destination != controller->options_.source ||
            reply.can_prefix != controller->options_.can_prefix) {
          continue;
        }

        const auto parsed = DiagnosticResponse::Parse(reply.data, reply.size);
        if (parsed.channel != 1) { continue; }

        if (parsed.size == 0) {
          empty_replies++;
        } else {
          empty_replies = 0;
        }

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
    Transport* transport = nullptr;
    CompletionCallback callback;

    CanFdFrame output_frame_;

    void Start() {
      DoWrite();
    }

    void DoWrite() {
      DiagnosticWrite::Command write;
      write.channel = channel;
      write.data = message.data();
      const auto to_write = std::min<size_t>(48, message.size());
      write.size = to_write;

      output_frame_ = controller->DefaultFrame(kNoReply);
      WriteCanData write_frame(output_frame_.data, &output_frame_.size);
      DiagnosticWrite::Make(&write_frame, write, {});

      auto s = shared_from_this();

      controller->transport()->Cycle(
          &output_frame_, 1, nullptr,
          [s, to_write](int v) {
            s->message = s->message.substr(to_write);
            s->Callback(v);
          });
    }

    void Callback(int v) {
      if (message.empty()) {
        transport->Post(std::bind(callback, v));
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

  Optional<Result> FindResult(const std::vector<CanFdFrame>& replies) const {
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
        query_format_override != nullptr ? kReplyRequired :
        options_.default_query ? kReplyRequired : kNoReply);

    WriteCanData write_frame(result.data, &result.size);
    result.expected_reply_size = CommandType::Make(&write_frame, cmd, fmt);

    if (query_format_override) {
      result.expected_reply_size =
          Query::Make(&write_frame, *query_format_override);
    } else if (options_.default_query) {
      std::memcpy(&result.data[result.size],
                  &query_frame_.data[0],
                  query_frame_.size);
      result.size += query_frame_.size;
      result.expected_reply_size = query_reply_size_;
    }

    return result;
  }

  const Options options_;
  std::shared_ptr<Transport> transport_;
  CanData query_frame_;
  uint8_t query_reply_size_ = 0;
  CanFdFrame output_frame_;

  // This is a member variable so we can avoid re-allocating it on
  // every call.
  std::vector<CanFdFrame> single_command_replies_;
};


}  // namespace moteus
}  // namespace mjbots
