// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

/// @file
///
/// Drive a dynamometer test fixture.

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signals2/signal.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/visitor.h"

#include "mjlib/io/now.h"
#include "mjlib/io/selector.h"

#include "mjlib/multiplex/asio_client.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"

#include "mjlib/telemetry/binary_schema_parser.h"
#include "mjlib/telemetry/binary_write_archive.h"
#include "mjlib/telemetry/file_writer.h"
#include "mjlib/telemetry/mapped_binary_reader.h"

#include "fw/math.h"

#include "utils/line_reader.h"
#include "utils/moteus_subset.h"
#include "utils/run_for.h"

namespace base = mjlib::base;
namespace io = mjlib::io;
namespace mp = mjlib::multiplex;

namespace moteus {
namespace tool {
namespace {

constexpr int kDebugTunnel = 1;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kNumTareSamples = 4;

struct Options {
  int fixture_id = 32;
  int dut_id = 1;
  std::string torque_transducer;
  double transducer_scale = -1.0;

  bool verbose = false;
  std::string log;
  double max_test_time_s = 1200.0;

  double max_torque_Nm = 0.5;


  // The different cycles we can do and their options.
  bool static_torque_ripple = false;
  double static_torque_ripple_speed = 0.01;

  bool pwm_cycle_overrun = false;

  // Different functional validation cycles.
  bool validate_pwm_mode = false;
  bool validate_current_mode = false;
  bool validate_position_basic = false;
  bool validate_position_pid = false;
  bool validate_position_lowspeed = false;
  bool validate_position_wraparound = false;
  bool validate_stay_within = false;
  bool validate_max_slip = false;
  bool validate_slip_stop_position = false;
  bool validate_slip_bounds = false;
  bool validate_dq_ilimit = false;
  bool validate_power_limit = false;
  bool validate_max_velocity = false;
  bool validate_rezero = false;
  bool validate_voltage_mode_control = false;
  bool validate_fixed_voltage_mode = false;
  bool validate_brake_mode = false;
  bool validate_velocity_accel_limits = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(fixture_id));
    a->Visit(MJ_NVP(dut_id));
    a->Visit(MJ_NVP(torque_transducer));
    a->Visit(MJ_NVP(transducer_scale));
    a->Visit(MJ_NVP(verbose));
    a->Visit(MJ_NVP(log));
    a->Visit(MJ_NVP(max_test_time_s));
    a->Visit(MJ_NVP(max_torque_Nm));

    a->Visit(MJ_NVP(static_torque_ripple));
    a->Visit(MJ_NVP(static_torque_ripple_speed));

    a->Visit(MJ_NVP(pwm_cycle_overrun));

    a->Visit(MJ_NVP(validate_pwm_mode));
    a->Visit(MJ_NVP(validate_current_mode));
    a->Visit(MJ_NVP(validate_position_basic));
    a->Visit(MJ_NVP(validate_position_pid));
    a->Visit(MJ_NVP(validate_position_lowspeed));
    a->Visit(MJ_NVP(validate_position_wraparound));
    a->Visit(MJ_NVP(validate_stay_within));
    a->Visit(MJ_NVP(validate_max_slip));
    a->Visit(MJ_NVP(validate_slip_stop_position));
    a->Visit(MJ_NVP(validate_slip_bounds));
    a->Visit(MJ_NVP(validate_dq_ilimit));
    a->Visit(MJ_NVP(validate_power_limit));
    a->Visit(MJ_NVP(validate_max_velocity));
    a->Visit(MJ_NVP(validate_rezero));
    a->Visit(MJ_NVP(validate_voltage_mode_control));
    a->Visit(MJ_NVP(validate_fixed_voltage_mode));
    a->Visit(MJ_NVP(validate_brake_mode));
    a->Visit(MJ_NVP(validate_velocity_accel_limits));
  }
};

double Median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  if (values.empty()) { return 0.0; }

  if ((values.size() % 2 == 1)) {
    return values[values.size()/2];
  } else {
    return 0.5*(values[values.size()/2-1] +
                values[values.size()/2]);
  }
}

struct TorqueTransducer {
  boost::posix_time::ptime timestamp;

  uint32_t time_code = 0;
  double raw_value = 0.0;
  double torque_Nm = 0.0;
  double temperature_C = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(time_code));
    a->Visit(MJ_NVP(raw_value));
    a->Visit(MJ_NVP(torque_Nm));
    a->Visit(MJ_NVP(temperature_C));
  }
};

void ExceptionRethrower(std::exception_ptr ptr) {
  if (ptr) {
    std::rethrow_exception(ptr);
  }
}

class LogRegistrar {
 public:
  LogRegistrar(mjlib::telemetry::FileWriter* file_writer)
      : file_writer_(file_writer) {}

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    const auto identifier = file_writer_->AllocateIdentifier(name);
    file_writer_->WriteSchema(
        identifier,
        mjlib::telemetry::BinarySchemaArchive::template schema<T>());
    signal->connect(std::bind(&LogRegistrar::HandleData<T>,
                              this, identifier, std::placeholders::_1));
  }

  template <typename T>
  void HandleData(mjlib::telemetry::FileWriter::Identifier identifier,
                  const T* data) {
    if (!file_writer_->IsOpen()) { return; }

    auto buffer = file_writer_->GetBuffer();
    mjlib::telemetry::BinaryWriteArchive(*buffer).Accept(data);
    file_writer_->WriteData({}, identifier, std::move(buffer));
  }

 private:
  mjlib::telemetry::FileWriter* const file_writer_;
};

double RelativeError(double a, double b) {
  return std::abs((a - b) / b);
}

std::string StringValue(const std::string& line_in) {
  const auto line = boost::trim_copy(line_in);
  std::size_t pos = line.find(' ');
  if (pos == std::string::npos) { return ""; }
  return line.substr(pos + 1);
}

class ServoStatsReader {
 public:
  ServoStatsReader(const std::string& schema) : schema_(schema) {}

  ServoStats Read(const std::string& data) {
    auto result = reader_.Read(data);

    // Here we verify that the final and total timer are always valid.
    if (result.final_timer == 0 ||
        result.total_timer == 0 ||
        result.final_timer > 3750 ||
        result.total_timer < 5000) {
      throw mjlib::base::system_error::einval(
          fmt::format("Invalid timer final={} total={}",
                      result.final_timer, result.total_timer));
    }

    return result;
  }

 private:
  const std::string schema_;
  mjlib::telemetry::BinarySchemaParser parser_{schema_, "servo_stats"};
  mjlib::telemetry::MappedBinaryReader<ServoStats> reader_{&parser_};
};

class Controller {
 public:
  Controller(const std::string& log_prefix,
             mjlib::telemetry::FileWriter* file_writer,
             mjlib::io::SharedStream stream,
             const Options& options)
      : log_prefix_(log_prefix),
        stream_(stream),
        file_writer_(file_writer),
        options_(options) {}

  std::string stats() const {
    return fmt::format("{:2d} {:6.3f}", servo_stats_.mode, servo_stats_.torque_Nm);
  }

  const ServoStats& servo_stats() const {
    return servo_stats_;
  }

  boost::asio::awaitable<void> OKReceived() {
    auto operation = [this](io::ErrorCallback callback) {
      BOOST_ASSERT(!ok_callback_);
      ok_callback_ = std::move(callback);
    };

    co_await async_initiate<
      decltype(boost::asio::use_awaitable),
      void(boost::system::error_code)>(operation, boost::asio::use_awaitable);
  }

  boost::asio::awaitable<void> SomethingReceived() {
    auto operation = [this](io::ErrorCallback callback) {
      BOOST_ASSERT(!received_callback_);
      received_callback_ = std::move(callback);
    };

    co_await async_initiate<
      decltype(boost::asio::use_awaitable),
      void(boost::system::error_code)>(operation, boost::asio::use_awaitable);
  }

  boost::asio::awaitable<void> Flush() {
    char buf[4096] = {};

    // Discard anything we receive for some amount of time.
    co_await RunFor(
        stream_->get_executor(),
        *stream_,
        [&]() -> boost::asio::awaitable<bool> {
          boost::system::error_code ec;
          co_await boost::asio::async_read(
              *stream_,
              boost::asio::buffer(buf),
              boost::asio::redirect_error(boost::asio::use_awaitable, ec));
          // We just ignore ec.
          co_return false;
        },
        mjlib::io::Now(stream_->get_executor().context()) +
        boost::posix_time::milliseconds(300));

    co_return;
  }

  boost::asio::awaitable<void> Start() {
    co_await WriteMessage("tel stop\nd stop\n");

    co_await Flush();

    boost::asio::co_spawn(
        stream_->get_executor(),
        std::bind(&Controller::Run, this),
        ExceptionRethrower);

    std::vector<std::string> names = {
      "motor_position",
      "servo_stats",
      "servo_control",
      "servo_cmd",
      "git",
    };

    for (const auto& name : names) {
      co_await WriteMessage(fmt::format("tel schema {}", name));
      while (true) {
        co_await SomethingReceived();
        if (log_ids_.count(name) != 0) { break; }
      }

      co_await Command(fmt::format("tel rate {} 50", name));

      // Now wait for this to have data.
      while (true) {
        co_await SomethingReceived();
        if (log_data_.count(name) != 0) { break; }
      }
    }

    co_return;
  }

  boost::asio::awaitable<void> Command(const std::string& message) {
    co_await WriteMessage(message);
    co_await OKReceived();
    co_return;
  }

  boost::asio::awaitable<void> WriteMessage(const std::string& message) {
    if (options_.verbose) {
      fmt::print("> {}\n", message);
    }
    co_await boost::asio::async_write(
        *stream_,
        boost::asio::buffer(message + "\n"),
        boost::asio::use_awaitable);
    co_return;
  }

  struct PidConstants {
    double kp = 1.0;
    double ki = 0.0;
    double ilimit = 0.0;
    double kd = 0.0;

    double position_min = std::numeric_limits<double>::quiet_NaN();
    double position_max = std::numeric_limits<double>::quiet_NaN();

    double max_position_slip = std::numeric_limits<double>::quiet_NaN();
    double max_power_W = 450.0;
    double max_velocity = 500.0;

    bool voltage_mode_control = false;

    bool fixed_voltage_mode = false;
    double fixed_voltage_control_V = 0.0;
  };

  boost::asio::awaitable<void> ConfigurePid(const PidConstants& pid) {
    co_await Command(
        fmt::format("conf set servo.pid_position.kp {}", pid.kp));
    co_await Command(
        fmt::format("conf set servo.pid_position.ki {}", pid.ki));
    co_await Command(
        fmt::format("conf set servo.pid_position.ilimit {}", pid.ilimit));
    co_await Command(
        fmt::format("conf set servo.pid_position.kd {}", pid.kd));

    co_await Command(
        fmt::format("conf set servopos.position_min {}", pid.position_min));
    co_await Command(
        fmt::format("conf set servopos.position_max {}", pid.position_max));

    co_await Command(
        fmt::format("conf set servo.max_position_slip {}",
                    pid.max_position_slip));

    co_await Command(
        fmt::format("conf set servo.max_power_W {}",
                    pid.max_power_W));

    co_await Command(
        fmt::format("conf set servo.max_velocity {}", pid.max_velocity));

    co_await Command(
        fmt::format("conf set servo.voltage_mode_control {}", pid.voltage_mode_control ? 1 : 0));

    co_await Command(
        fmt::format("conf set servo.fixed_voltage_mode {}", pid.fixed_voltage_mode ? 1 : 0));

    co_await Command(
        fmt::format("conf set servo.fixed_voltage_control_V {}", pid.fixed_voltage_control_V));

    co_return;
  }

  boost::asio::awaitable<void> Run() {
    try {
      co_await DoRun();
    } catch (mjlib::base::system_error& se) {
      se.code().Append("When processing " + log_prefix_);
      throw;
    }
    co_return;
  }

  boost::asio::awaitable<void> DoRun() {
    // Read from the stream, barfing on errors, and logging any binary
    // data that comes our way.
    while (true) {
      const auto maybe_line = co_await reader_.ReadLine();

      if (!maybe_line) {
        // We had a timeout, this should only happen here on error.
        throw mjlib::base::system_error::einval("Timeout");
      }
      const auto line = *maybe_line;
      if (boost::starts_with(line, "OK")) {
        if (ok_callback_) {
          boost::asio::post(
              stream_->get_executor(),
              std::bind(std::move(ok_callback_), mjlib::base::error_code()));
        }
        // We ignore these.
      } else if (boost::starts_with(line, "ERR")) {
        // This causes us to terminate immediately.
        throw base::system_error::einval(line);
      } else if (boost::starts_with(line, "schema ")) {
        const auto name = StringValue(line);

        const auto ident = file_writer_->AllocateIdentifier(log_prefix_ + name);
        log_ids_[name] = ident;
        const std::string schema = co_await reader_.ReadBinaryBlob();
        file_writer_->WriteSchema(ident, schema);

        if (name == "servo_stats") {
          servo_stats_reader_.emplace(schema);
        }
      } else if (boost::starts_with(line, "emit ")) {
        const auto name = StringValue(line);
        if (log_ids_.count(name)) {
          const auto ident = log_ids_.at(name);
          log_data_.insert(name);
          const auto data = co_await reader_.ReadBinaryBlob();
          file_writer_->WriteData({}, ident, data);
          if (name == "servo_stats") {
            HandleServoStats(servo_stats_reader_->Read(data));
          }
        }
      } else {
        fmt::print("Ignoring unknown line: {}\n", line);
      }

      if (received_callback_) {
        boost::asio::post(
            stream_->get_executor(),
            std::bind(std::move(received_callback_), mjlib::base::error_code()));
      }
    }

    co_return;
  }

  mjlib::io::AsyncStream& stream() { return *stream_; }

 private:
  void HandleServoStats(const ServoStats& servo_stats) {
    servo_stats_ = servo_stats;
    // By default, we throw an exception for any fault.
    if (servo_stats_.mode == ServoStats::kFault) {
      throw mjlib::base::system_error::einval(
          fmt::format("Fault: {} {}", log_prefix_,
                      static_cast<int>(servo_stats.fault)));
    }
  }

  const std::string log_prefix_;
  mjlib::io::SharedStream stream_;
  mjlib::telemetry::FileWriter* const file_writer_;
  const Options options_;

  LineReader reader_{stream_.get(), [&]() {
      LineReader::Options o;
      o.set_verbose(options_.verbose);
      return o;
    }()};

  std::map<std::string, uint64_t> log_ids_;
  std::set<std::string> log_data_;

  io::ErrorCallback received_callback_;
  io::ErrorCallback ok_callback_;

  std::optional<ServoStatsReader> servo_stats_reader_;
  ServoStats servo_stats_;
};

class Application {
 public:
  Application(boost::asio::io_context& context,
              io::Selector<mp::AsioClient>* selector,
              const Options& options)
      : executor_(context.get_executor()),
        client_selector_(selector),
        options_(options),
        file_writer_() {}

  void ErrorStop() {
    client_selector_->AsyncStart(
        std::bind(&Application::HandleStopClient, this, std::placeholders::_1));
  }

  void Start() {
    if (!options_.log.empty()) {
      file_writer_.Open(options_.log);
    }
    client_selector_->AsyncStart(
        std::bind(&Application::HandleClient, this, std::placeholders::_1));
  }

  void HandleStopClient(const base::error_code& ec) {
    fmt::print("HandleStopClient\n");
    base::FailIf(ec);
    client_ = client_selector_->selected();
    boost::asio::co_spawn(
        executor_,
        std::bind(&Application::TaskStop, this),
        ExceptionRethrower);
  }

  void HandleClient(const base::error_code& ec) {
    base::FailIf(ec);

    client_ = client_selector_->selected();

    boost::asio::co_spawn(
        executor_,
        std::bind(&Application::Task, this),
        [](std::exception_ptr ptr) {
          if (ptr) {
            std::rethrow_exception(ptr);
          }
          std::exit(0);
        });
  }

  boost::asio::awaitable<void> TaskStop() {
    fmt::print("TaskStop\n");
    fixture_.emplace(
        "fixture_",
        &file_writer_,
        client_->MakeTunnel(options_.fixture_id, kDebugTunnel, {}),
        options_);
    dut_.emplace(
        "dut_",
        &file_writer_,
        client_->MakeTunnel(options_.dut_id, kDebugTunnel, {}),
        options_);

    co_await fixture_->WriteMessage("d stop");
    co_await dut_->WriteMessage("d stop");

    co_await fixture_->Flush();
    co_await dut_->Flush();

    fmt::print("Sleeping\n");
    co_await Sleep(1.0);

    std::exit(1);

    co_return;
  }

  boost::asio::awaitable<void> Task() {
    co_await Init();

    const auto expiration =
        mjlib::io::Now(executor_.context()) +
        mjlib::base::ConvertSecondsToDuration(options_.max_test_time_s);

    co_await RunFor(
        executor_,
        fixture_->stream(),
        [&]() -> boost::asio::awaitable<bool> {
          std::exception_ptr eptr;
          try {
            co_await RunTestCycle();
          } catch (mjlib::base::system_error& se) {
            eptr = std::current_exception();
          }
          if (eptr) {
            std::rethrow_exception(eptr);
          }
          co_return false;
        },
        expiration);

    co_return;
  }

  boost::asio::awaitable<void> RunTestCycle() {
    if (options_.static_torque_ripple) {
      co_await RunStaticTorqueRipple();
    } else if (options_.pwm_cycle_overrun) {
      co_await RunPwmCycleOverrun();
    } else if (options_.validate_pwm_mode) {
      co_await ValidatePwmMode();
    } else if (options_.validate_current_mode) {
      co_await ValidateCurrentMode();
    } else if (options_.validate_position_pid) {
      co_await ValidatePositionPid();
    } else if (options_.validate_position_basic) {
      co_await ValidatePositionBasic();
    } else if (options_.validate_position_lowspeed) {
      co_await ValidatePositionLowspeed();
    } else if (options_.validate_position_wraparound) {
      co_await ValidatePositionWraparound();
    } else if (options_.validate_stay_within) {
      co_await ValidateStayWithin();
    } else if (options_.validate_max_slip) {
      co_await ValidateMaxSlip();
    } else if (options_.validate_slip_stop_position) {
      co_await ValidateSlipStopPosition();
    } else if (options_.validate_slip_bounds) {
      co_await ValidateSlipBounds();
    } else if (options_.validate_dq_ilimit) {
      co_await ValidateDqIlimit();
    } else if (options_.validate_power_limit) {
      co_await ValidatePowerLimit();
    } else if (options_.validate_max_velocity) {
      co_await ValidateMaxVelocity();
    } else if (options_.validate_rezero) {
      co_await ValidateRezero();
    } else if (options_.validate_voltage_mode_control) {
      co_await ValidateVoltageModeControl();
    } else if (options_.validate_fixed_voltage_mode) {
      co_await ValidateFixedVoltageMode();
    } else if (options_.validate_brake_mode) {
      co_await ValidateBrakeMode();
    } else if (options_.validate_velocity_accel_limits) {
      co_await ValidateVelocityAccelLimits();
    } else {
      fmt::print("No cycle selected\n");
    }
    co_return;
  }

  boost::asio::awaitable<void> Stop() {
    // Make sure everything is stopped.  We use WriteMessage here,
    // since it doesn't rely on the read loops operating.

    co_await dut_->WriteMessage("d stop");
    co_await fixture_->WriteMessage("d stop");
    co_await dut_->WriteMessage("tel stop");
    co_await fixture_->WriteMessage("tel stop");

    // Since we can't use the read loops, just wait a bit to make sure
    // those commands go out.
    co_await Sleep(1.0);

    co_return;
  }

  boost::asio::awaitable<void> CommandFixtureRigid() {
    Controller::PidConstants pid;
    pid.ki = 300.0;
    pid.kd = 0.60;
    pid.kp = 5.0;
    pid.ilimit = 0.3;

    co_await fixture_->ConfigurePid(pid);
  }

  boost::asio::awaitable<void> RunStaticTorqueRipple() {
    // Zero out the integrator.  We don't care about the other gains.
    co_await dut_->ConfigurePid(Controller::PidConstants());
    co_await CommandFixtureRigid();

    // Then start commanding the different torques on the dut servo,
    // each for a bit more than one revolution.
    for (double test_torque : {kNaN, 0.0, 0.05, -0.05, 0.1, -0.1, 0.2, -0.2}) {
      fmt::print("\nTesting torque: {}\n", test_torque);

      co_await fixture_->Command("d rezero");
      co_await dut_->Command("d rezero");

      // Start the fixture sweeping.
      co_await fixture_->Command(
          fmt::format("d pos nan {} {}",
                      options_.static_torque_ripple_speed,
                      options_.max_torque_Nm));


      if (std::isfinite(test_torque)) {
        co_await dut_->Command(fmt::format("d pos nan 0 {} p0 d0 f{}",
                                           std::abs(test_torque), test_torque));
      } else {
        co_await dut_->Command(fmt::format("d stop"));
      }

      // Wait for the time required by one full cycle plus a bit while
      // printing our status.
      StatusPrinter status_printer(
          this, fmt::format("STAT_TOR({})", test_torque));

      co_await Sleep(1.2 / options_.static_torque_ripple_speed);

      co_await dut_->Command("d stop");
      co_await fixture_->Command("d stop");
    }
  }

  boost::asio::awaitable<void> RunPwmCycleOverrun() {
    co_await fixture_->Command("d stop");
    co_await dut_->ConfigurePid(Controller::PidConstants());
    co_await dut_->Command("conf set servopos.position_min -3.0");
    co_await dut_->Command("conf set servopos.position_max 3.0");

    constexpr int kPulseCount = 4;
    constexpr double kPulseTorque = 1.5;

    for (int i = 0; i < kPulseCount; i++) {
      // Now, what we'll do is command a moderate constant torque with
      // no load presented by the fixture.  That should result in the
      // current loop maxing out, which if the PWM cycle isn't
      // configured correctly will fault.  If it is configured
      // correctly, we'll just zoom past the position limit and stop.
      fmt::print("About to pulse {}/{}\n", i + 1, kPulseCount);
      co_await Sleep(1.0);

      co_await dut_->Command("d rezero");
      co_await Sleep(0.1);

      co_await dut_->Command(
          fmt::format("d pos nan 0 {} p0 d0 f{}",
                      kPulseTorque,
                      kPulseTorque * ((i % 2) == 0 ? 1.0 : -1.0)));

      co_await Sleep(4.0);

      // Verify that we have not faulted.
      if (dut_->servo_stats().mode != ServoStats::kPosition) {
        throw mjlib::base::system_error::einval("DUT no longer in position mode");
      }

      co_await dut_->Command("d stop");
      co_await Sleep(0.5);
    }

    fmt::print("Test passed\n");

    co_return;
  }

  struct PwmResult {
    double phase = 0.0;
    double voltage = 0.0;

    ServoStats::Mode mode = ServoStats::kStopped;
    double d_A = 0.0;
    double q_A = 0.0;
    double fixture_speed = 0.0;
    double fixture_position = 0.0;
  };

  boost::asio::awaitable<PwmResult> RunPwmVoltage(double phase, double voltage) {
    co_await dut_->Command(fmt::format("d pwm {} {}",
                                       WrapZeroToTwoPi(phase), voltage));
    co_await Sleep(0.5);

    std::vector<double> d_A;
    std::vector<double> q_A;
    std::vector<double> velocity;
    std::vector<double> position;

    PwmResult result;
    result.phase = phase;
    result.voltage = voltage;
    result.mode = dut_->servo_stats().mode;

    for (int i = 0; i < 4; i++) {
      d_A.push_back(dut_->servo_stats().d_A);
      q_A.push_back(dut_->servo_stats().q_A);
      velocity.push_back(fixture_->servo_stats().velocity);
      position.push_back(fixture_->servo_stats().position);
      co_await Sleep(0.1);
    }

    result.d_A = Median(d_A);
    result.q_A = Median(q_A);
    result.fixture_speed = Median(velocity);
    result.fixture_position = Median(position);

    co_return result;
  }

  boost::asio::awaitable<void> ValidatePwmMode() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");
    co_await fixture_->Command("d rezero");

    fmt::print("Centering motor\n");
    // We command a sweep in order to ensure the motor is not locked
    // at the opposite phase.
    co_await dut_->Command("d pwm -2 0.35 2");
    co_await Sleep(1.0);

    fmt::print("Running voltage ramp\n");

    std::vector<PwmResult> ramp_results;

    // First, ramp up PWM gradually.  The measured current should
    // increase and should be mostly in the D phase and the motor
    // should not generally be spinning.
    for (double voltage : { 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35 }) {
      fmt::print("{}  \r", voltage);
      ::fflush(stdout);
      ramp_results.push_back(co_await RunPwmVoltage(0.0, voltage));
    }

    std::vector<PwmResult> slew_results;

    fmt::print("Running phase slew\n");

    // Then slew phase around.  We'll make sure that the current
    // remains largely in the D phase, and that the fixture does
    // indeed move around.
    const double kSlewVoltage = 0.35;
    for (double phase = 0.0; phase < 30.0; phase += 1.0) {
      fmt::print("{}  \r", phase);
      ::fflush(stdout);
      slew_results.push_back(
          co_await RunPwmVoltage(phase, kSlewVoltage));
    }

    co_await dut_->Command("d stop");

    // Print our results.
    fmt::print("\nRAMP\n");
    for (const auto& r : ramp_results) {
      fmt::print(" {} {} {} {} {}\n",
                 r.voltage, r.d_A, r.q_A, r.fixture_speed, r.fixture_position);
    }
    fmt::print("\n");
    fmt::print("SLEW\n");
    for (const auto& r : slew_results) {
      fmt::print(" {} {} {} {} {}\n",
                 r.phase, r.d_A, r.q_A, r.fixture_speed, r.fixture_position);
    }

    // And then analyze them.
    std::vector<std::string> errors;

    for (const auto& r : ramp_results) {
      if (r.mode != ServoStats::kVoltageFoc) {
        errors.push_back(
            fmt::format("Motor not in PWM mode at voltage: {} {}!={}",
                        r.voltage, r.mode, ServoStats::kVoltageFoc));
        break;
      }
    }

    for (const auto& r : slew_results) {
      if (r.mode != ServoStats::kVoltageFoc) {
        errors.push_back(
            fmt::format("Motor not in PWM mode at phase: {} {}!={}",
                        r.phase, r.mode, ServoStats::kVoltageFoc));
        break;
      }
    }

    for (const auto& r : ramp_results) {
      const double kMaxError = 1.5;
      if (std::abs(r.q_A) > kMaxError) {
        errors.push_back(
            fmt::format(
                "Too much Q phase current at voltage: {} ({} > {})",
                r.voltage, r.q_A, kMaxError));
        break;
      }
    }

    // This test runs with a motor that has a phase resistance of
    // roughly 0.051-0.065 ohms.
    const float kMotorResistance = 0.055;
    for (const auto& r : ramp_results) {
      const auto expected_current = r.voltage / kMotorResistance;
      if (std::abs(r.d_A - expected_current) > 2.5) {
        errors.push_back(
            fmt::format(
                "D phase current too far from purely resistive: "
                "I({}) = V({}) / R({}) != {}",
                expected_current, r.voltage, kMotorResistance, r.d_A));
        break;
      }
    }

    // The fixture should not be moving during the voltage ramp phase.
    const auto initial_fixture = ramp_results.at(0).fixture_position;
    for (const auto& r : ramp_results) {
      if (std::abs(r.fixture_position - initial_fixture) > 0.1) {
        errors.push_back(
            fmt::format("fixture moved position at voltage {}", r.voltage));
        break;
      }
      if (std::abs(r.fixture_speed) > 0.12) {
        errors.push_back(
            fmt::format("fixture had non-zero velocity at voltage {} |{}|>0.12",
                        r.voltage, r.fixture_speed));
        break;
      }
    }

    for (const auto& r : slew_results) {
      const double kMaxError = 1.5;
      if (std::abs(r.q_A) > kMaxError) {
        errors.push_back(
            fmt::format("Too much Q phase in slew at phase {} |{}| > {}",
                        r.phase, r.q_A, kMaxError));
        break;
      }
    }

    for (const auto& r : slew_results) {
      const double kExpectedCurrent = kSlewVoltage / kMotorResistance;
      const double kMaxError = 1.8;
      if (std::abs(r.d_A - kExpectedCurrent) > kMaxError) {
        errors.push_back(
            fmt::format("D phase is not correct |{} - {}| > {}",
                        r.d_A, kExpectedCurrent, kMaxError));
        break;
      }
    }

    for (const auto& r : slew_results) {
      if (std::abs(r.fixture_speed) > 0.12) {
        errors.push_back(
            fmt::format("fixture has non-zero speed at phase {}, {} != 0.0",
                        r.phase, r.fixture_speed));
        break;
      }
    }

    // The test motor requires 7 electrical cycles to result in 1 full
    // revolution.  We need to verify that each step is close to that
    // movement, and all in the same direction.
    double expected_position = slew_results.front().fixture_position;
    const double kMaxError = 0.015;
    for (const auto& r : slew_results) {
      if (std::abs(r.fixture_position - expected_position) > kMaxError) {
        errors.push_back(
            fmt::format("Fixture position off at phase {}, |{} - {}| > {}",
                        r.phase, r.fixture_position, expected_position, kMaxError));
        break;
      }
      expected_position -= (1.0 / (2.0 * M_PI)) / 7.0;
    }

    if (errors.size()) {
      fmt::print("\nERRORS\n");
      for (const auto& e : errors) {
        fmt::print("{}\n", e);
      }
      std::exit(1);
    } else {
      fmt::print("\nSUCCESS\n");
    }

    co_return;
  }

  boost::asio::awaitable<void> ValidateCurrentMode() {
    // Set the fixture to hold position.
    co_await CommandFixtureRigid();

    co_await fixture_->Command("d rezero");
    co_await dut_->Command("d rezero");

    auto test = [&](double d_A, double q_A,
                    double expected_torque,
                    double expected_speed,
                    const std::string& message) -> boost::asio::awaitable<void> {
      fmt::print("Testing d={} q={}\n", d_A, q_A);
      co_await dut_->Command(fmt::format("d dq {} {}", d_A, q_A));
      co_await Sleep(1.0);

      std::vector<double> data_q_A;
      std::vector<double> data_d_A;
      std::vector<double> data_torque;

      for (int i = 0; i < 4; i++) {
        data_d_A.push_back(dut_->servo_stats().d_A);
        data_q_A.push_back(dut_->servo_stats().q_A);
        data_torque.push_back(current_torque_Nm_);

        co_await Sleep(0.1);
      }

      const auto median_d_A = Median(data_d_A);
      const auto median_q_A = Median(data_q_A);
      const auto median_torque = Median(data_torque);

      try {
        if (dut_->servo_stats().mode != ServoStats::kCurrent) {
          throw mjlib::base::system_error::einval("DUT not in current mode");
        }
        if (std::abs(fixture_->servo_stats().velocity_filt - expected_speed) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "Fixture speed {} != {} (within {})",
                  fixture_->servo_stats().velocity_filt, expected_speed, 0.1));
        }
        if (std::abs(median_d_A - d_A) > 1.0) {
          throw mjlib::base::system_error::einval(
              fmt::format("D phase current {} != {} (within {})",
                          median_d_A, d_A, 1.0));
        }
        if (std::abs(median_q_A - q_A) > 1.0) {
          throw mjlib::base::system_error::einval(
              fmt::format("Q phase current {} != {} (within {})",
                          median_q_A, q_A, 1.0));
        }
        if (std::abs(median_torque - expected_torque) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format("Transducer torque {} != {} (within {})",
                          median_torque, expected_torque, 0.15));
        }
      } catch (mjlib::base::system_error& se) {
        se.code().Append(message);
        throw;
      }
    };

    // Verify a number of currents with the fixture stationary, and
    // moving in both directions.
    for (double fs : {0.0, -0.2, 0.2}) {
      // Start the fixture holding.
      co_await fixture_->Command(
          fmt::format("d pos nan {} 0.2", fs));
      co_await Sleep(1.0);

      try {
        co_await test(0, 0, 0, fs, "d=0 q=0");
        co_await test(2, 0, 0, fs, "d=2 q=0");
        co_await test(4, 0, 0, fs, "d=4 q=0");
        co_await test(0, 2, 0.1, fs, "d=0 q=2");
        co_await test(0, 4, 0.2, fs, "d=0 q=4");
        co_await test(0, -2, -0.1, fs, "d=0 q=-2");
        co_await test(0, -4, -0.2, fs, "d=0 q=-4");
      } catch (mjlib::base::system_error& se) {
        se.code().Append(fmt::format("fixture speed {}", fs));
        throw;
      }
    }

    fmt::print("SUCCESS\n");

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    co_return;
  }

  boost::asio::awaitable<void> RunBasicPositionTest(Controller::PidConstants pid) {
    // Move to a few different positions.
    for (const double position : {0.0, -0.2, 0.3}) {
      fmt::print("Moving to position {}\n", position);
      co_await dut_->Command(fmt::format("d pos {} 0 0.2", position));

      co_await Sleep(1.0);

      // The fixture should be close to this now.
      const double fixture_position =
          options_.transducer_scale * fixture_->servo_stats().position;
      if (std::abs(fixture_position - position) > 0.05) {
        throw mjlib::base::system_error::einval(
            fmt::format("Fixture position {} != {}",
                        fixture_position, position));
      }
    }

    co_await RunBasicPositionVelocityTest(pid, 1.0);
  }

  boost::asio::awaitable<void> RunBasicPositionVelocityTest(Controller::PidConstants pid, double tolerance_scale) {
    // Move at a few different velocities.
    for (const double velocity : {0.0, -1.5, 3.0}) {
      fmt::print("Moving at velocity {}\n", velocity);
      co_await dut_->Command(fmt::format("d pos nan {} 0.2", velocity));

      co_await Sleep(1.0);

      const double fixture_velocity =
          options_.transducer_scale * fixture_->servo_stats().velocity;
      if (std::abs(fixture_velocity - velocity) > 0.35 * tolerance_scale) {
        throw mjlib::base::system_error::einval(
            fmt::format("Fixture velocity {} != {}",
                        fixture_velocity, velocity));
      }
    }

    co_await dut_->Command("d stop");

    co_await Sleep(0.5);

    co_await dut_->Command("d index 0");
    co_await fixture_->Command("d index 0");

    // Use the stop position.
    for (const double stop_position : {2.0, 0.5}) {
      fmt::print("Moving using stop position to {}\n", stop_position);
      const double kFixedVelocity = 1.0;
      co_await dut_->Command(fmt::format("d pos nan {} 0.2 s{}",
                                         kFixedVelocity, stop_position));

      co_await Sleep(0.5);
      const double fixture_velocity =
          options_.transducer_scale * fixture_->servo_stats().velocity;
      if ((std::abs(fixture_velocity) - kFixedVelocity) > 0.38 * tolerance_scale) {
        throw mjlib::base::system_error::einval(
            fmt::format("Fixture velocity {} != {}",
                        fixture_velocity, kFixedVelocity));

      }

      co_await Sleep(2.5);
      const double fixture_position =
          options_.transducer_scale * fixture_->servo_stats().position;
      if (std::abs(fixture_position - stop_position) > 0.07 * tolerance_scale) {
        throw mjlib::base::system_error::einval(
            fmt::format("Fixture stop position {} != {}",
                        fixture_position, stop_position));
      }
    }

    // Configure with some position limits in place and verify we
    // don't move outside of them by much.
    for (const double position_limit : {0.1, 1.0, 2.0}) {
      co_await dut_->Command(
          fmt::format("d pos nan 1.5 {} s0", options_.max_torque_Nm));
      co_await Sleep(3.0);
      co_await fixture_->Command("d index 0");

      fmt::print("Testing position limit {}\n", position_limit);
      auto pid_limit = pid;
      pid_limit.position_min = -position_limit;
      pid_limit.position_max = 10.0;
      co_await dut_->ConfigurePid(pid_limit);

      co_await dut_->Command(
          fmt::format("d pos nan 1.5 {} s-10",
                      options_.max_torque_Nm));
      co_await Sleep(3.0);

      {
        const double fixture_position =
            options_.transducer_scale * fixture_->servo_stats().position;
        if (std::abs(fixture_position - (-position_limit)) > 0.07 * tolerance_scale) {
          throw mjlib::base::system_error::einval(
              fmt::format("Fixture stop position {} != {}",
                          fixture_position, -position_limit));
        }
      }

      pid_limit.position_min = -10.0;
      pid_limit.position_max = position_limit;
      co_await dut_->ConfigurePid(pid_limit);
      co_await dut_->Command(
          fmt::format("d pos nan 1.5 {} s10", options_.max_torque_Nm));
      co_await Sleep(3.0);

      {
        const double fixture_position =
            options_.transducer_scale * fixture_->servo_stats().position;
        if (std::abs(fixture_position - position_limit) > 0.07 * tolerance_scale) {
          throw mjlib::base::system_error::einval(
              fmt::format("Fixture stop position {} != {}",
                          fixture_position, position_limit));
        }
      }
    }

    co_await dut_->Command("d stop");

    // Get back to our default config.
    co_await dut_->ConfigurePid(pid);
  }

  boost::asio::awaitable<void> ValidatePositionBasic() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");
    co_await fixture_->Command("d index 0");
    co_await dut_->Command("d index 0");

    // Set some constants that should work for basic position control.
    Controller::PidConstants pid;
    pid.kp = 1.0;
    pid.ki = 0.0;
    pid.kd = 0.05;
    co_await dut_->ConfigurePid(pid);

    co_await RunBasicPositionTest(pid);

    // Now we'll do the basic tests with the fixture locked rigidly in
    // place.
    co_await fixture_->Command("d index 0");
    co_await dut_->Command("d index 0");

    co_await CommandFixtureRigid();
    co_await fixture_->Command(
        fmt::format("d pos 0 0 {}", options_.max_torque_Nm));

    for (const double max_torque : {0.15, 0.3}) {
      fmt::print("Testing max torque {}\n", max_torque);
      co_await dut_->Command(fmt::format("d pos 5 0 {}", max_torque));
      co_await Sleep(1.0);

      if (std::abs(current_torque_Nm_ - max_torque) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "kp torque not as expected {} != {} (within {})",
                  current_torque_Nm_, max_torque, 0.15));
      }

      co_await dut_->Command(fmt::format("d pos -5 0 {}", max_torque));
      co_await Sleep(1.0);
      if (std::abs(current_torque_Nm_ - (-max_torque)) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "kp torque not as expected {} != {} (within {})",
                  current_torque_Nm_, -max_torque, 0.15));
      }
    }

    for (const double feedforward_torque : {0.15, 0.3}) {
      fmt::print("Testing feedforward torque {}\n", feedforward_torque);
      co_await dut_->Command(
          fmt::format("d pos 0 0 {} f{}",
                      options_.max_torque_Nm, feedforward_torque));
      co_await Sleep(1.0);

      if (std::abs(current_torque_Nm_ - feedforward_torque) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "kp torque not as expected {} != {} (within {})",
                  current_torque_Nm_, feedforward_torque, 0.15));
      }

      co_await dut_->Command(
          fmt::format("d pos 0 0 {} f{}",
                      options_.max_torque_Nm, -feedforward_torque));
      co_await Sleep(1.0);
      if (std::abs(current_torque_Nm_ - (-feedforward_torque)) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "kp torque not as expected {} != {} (within {})",
                  current_torque_Nm_, -feedforward_torque, 0.15));
      }
    }

    fmt::print("SUCCESS\n");

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    co_return;
  }

  boost::asio::awaitable<void> ValidatePositionPid() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    // Set the fixture to hold position.
    co_await CommandFixtureRigid();

    auto verify_position_mode = [&]() {
      if (dut_->servo_stats().mode != ServoStats::kPosition) {
        throw mjlib::base::system_error::einval("DUT not in position mode");
      }
    };

    // Start the fixture holding where it is.
    co_await fixture_->Command("d index 0");
    co_await fixture_->Command(
        fmt::format("d pos 0 0 {}", options_.max_torque_Nm));

    co_await Sleep(1.0);

    // Verify position mode kp gain.
    for (const double kp : { 1.0, 0.5, 0.2}) {
      Controller::PidConstants pid;
      pid.kp = kp;
      co_await dut_->ConfigurePid(pid);

      // Set the DUT to be at exactly 0.
      co_await dut_->Command("d index 0");

      for (double position : {0.0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3}) {
        fmt::print("Verifying kp={} position={}\n", kp, position);

        // Enter position mode.
        co_await dut_->Command(fmt::format("d pos {} 0 0.4", position));
        co_await Sleep(1.5);

        // We should be in position mode, the actual fixture position
        // should not have materially changed, and the correct amount of
        // torque should be present at the physical torque sensor.
        verify_position_mode();

        if (std::abs(fixture_->servo_stats().position) > 0.02) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "fixture position no longer zero {}",
                  fixture_->servo_stats().position));
        }

        const double expected_torque = pid.kp * position;
        if (std::abs(current_torque_Nm_ - expected_torque) > 0.15) {
          throw mjlib::base::system_error::einval(
              fmt::format(
                  "kp torque not as expected {} != {} (within {})",
                  current_torque_Nm_, expected_torque, 0.15));
        }
      }
      dut_->Command("d stop");
    }

    // Now we'll test kd.
    for (const double kd : {0.15, 0.1, 0.05}) {
      fmt::print("Verifying kd {}\n", kd);
      Controller::PidConstants pid;
      pid.kp = 0.0;
      pid.kd = kd;
      co_await dut_->ConfigurePid(pid);

      co_await dut_->Command("d pos nan 0 0.4");

      for (double speed : {-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0}) {
        co_await fixture_->Command(fmt::format("d pos nan {} 0.4", speed));
        co_await Sleep(1.0);

        verify_position_mode();
        const double expected_torque = speed * pid.kd;

        if (std::abs(current_torque_Nm_ - expected_torque) > 0.18) {
          throw mjlib::base::system_error::einval(
              fmt::format("kd torque not as expected {} != {} (within {})",
                          current_torque_Nm_, expected_torque, 0.18));
        }
      }
      co_await fixture_->Command("d stop");
      co_await dut_->Command("d stop");
    }

    // Test ki.

    // Lock and rezero everything.
    co_await fixture_->Command("d index 0");
    co_await dut_->Command("d index 0");
    co_await fixture_->Command(
        fmt::format("d pos 0 0 {}", options_.max_torque_Nm));

    for (const double ki : {1.0, 0.5, 0.2}) {
      fmt::print("Verifying ki {}\n", ki);

      Controller::PidConstants pid;
      pid.kp = 0.0;
      pid.kd = 0.0;
      pid.ki = ki;
      pid.ilimit = options_.max_torque_Nm;
      co_await dut_->ConfigurePid(pid);

      for (const double position : {0.3, 0.15, 0.0, -0.15, -0.3}) {
        co_await dut_->Command(
            fmt::format("d pos {} 0 {}", position, options_.max_torque_Nm));

        const double kDelayS = position == 0.0 ? 2.0 :
            std::min(0.5 / (std::abs(position) * pid.ki), 5.0);
        co_await Sleep(kDelayS);

        const double kTorqueLagS = 0.3;
        const double expected_torque =
            (kDelayS - kTorqueLagS) * pid.ki * position;

        verify_position_mode();
        if (std::abs(current_torque_Nm_ - expected_torque) > 0.17) {
          throw mjlib::base::system_error::einval(
              fmt::format("ki torque not as expected {} != {} (within {})",
                          current_torque_Nm_, expected_torque, 0.17));
        }

        // Stop the DUT to clear out the I term.
        co_await dut_->Command("d stop");

        // And sleep a bit to let the fixture recuperate.
        co_await Sleep(0.2);
      }
    }

    fmt::print("SUCCESS\n");

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    co_return;
  }

  boost::asio::awaitable<void> ValidatePositionLowspeed() {
    co_await fixture_->Command("d stop");
    co_await dut_->Command("d stop");

    // We purposefully use no I term to see how good we are with just
    // PD.
    Controller::PidConstants pid;
    pid.kp = 10.0;
    pid.kd = 0.1;
    co_await dut_->ConfigurePid(pid);

    for (const double start_pos : {0.0, 16.0, 32700.0}) {
      // We start with a series of low speed maneuvers.  At each, we
      // verify that movement does take place, and that the velocity
      // profile as measured at the fixture is sufficiently "smooth".
      //
      // TODO: Test 0.001, although right now our performance isn't good
      // enough to distinguish it from zero velocity.
      for (const double speed : { 0.5, -0.5, 0.1, -0.1, 0.01, -0.01 }) {
        fmt::print("Testing speed {} from start {}\n", speed, start_pos);

        co_await dut_->Command(fmt::format("d index {}", start_pos));
        co_await fixture_->Command("d index 0");
        co_await dut_->Command(fmt::format("d pos nan {} {}",
                                           speed, options_.max_torque_Nm));

        struct Result {
          double position = 0;
          double velocity = 0;
        };
        std::vector<Result> results;
        constexpr auto kIterationCount = 50;
        constexpr auto kDelayS = 0.2;
        constexpr auto kTotalTime = kIterationCount * kDelayS;
        const double scale = options_.transducer_scale;

        for (int i = 0; i < kIterationCount; i++) {
          co_await Sleep(kDelayS);
          results.push_back({
              scale * fixture_->servo_stats().position,
                  scale * fixture_->servo_stats().velocity});
        }

        co_await dut_->Command("d stop");

        // Now look to see that the fixture moved the correct amount.
        const double expected_movement = kTotalTime * speed;
        const double actual_movement = results.back().position;

        // TODO: Lower this threshold once we get better performance.
        const double tolerance =
            (std::abs(speed) > 0.1) ? 0.07 : 0.021;
        if (std::abs(expected_movement - actual_movement) > tolerance) {
          throw mjlib::base::system_error::einval(
              fmt::format("total movement not as expected {} != {}",
                          actual_movement, expected_movement));
        }

        for (size_t i = 1; i < results.size(); i++) {
          const double estimated_velocity =
              (results[i].position - results[i - 1].position) / kDelayS;
          // TODO: Lower this threshold.
          if (std::abs(estimated_velocity - speed) > 0.205) {
            throw mjlib::base::system_error::einval(
                fmt::format("estimated speed at index {} too far off {} != {}",
                            i, estimated_velocity, speed));
          }
        }

        co_await Sleep(1.0);
      }
    }

    fmt::print("SUCCESS\n");

    co_return;
  }

  boost::asio::awaitable<void> ValidatePositionWraparound() {
    fmt::print("Testing for wraparound\n");
    co_await fixture_->Command("d stop");
    co_await fixture_->Command("d index 0");

    co_await dut_->Command("d stop");

    Controller::PidConstants pid;
    pid.kd = 0.05;
    co_await dut_->ConfigurePid(pid);

    co_await dut_->Command("d index 32760");
    co_await dut_->Command("d pos nan 4.0 0.1");

    co_await Sleep(0.5);

    // This should wrap around in 2 s.
    for (int tenth_seconds = 0; tenth_seconds < 40; tenth_seconds++) {
      co_await Sleep(0.1);

      const double velocity =
          options_.transducer_scale * fixture_->servo_stats().velocity;
      if (std::abs(velocity - 4.0) > 0.3) {
        throw mjlib::base::system_error::einval(
            fmt::format("velocity incorrect {} != {}",
                        velocity, 4.0));
      }
    }

    co_await dut_->Command("d stop");
    co_await Sleep(1.0);

    const double position = dut_->servo_stats().position;
    if (position > -30000.0) {
      throw mjlib::base::system_error::einval(
          fmt::format("position did not wrap {}", position));
    }

    // We used to check control_position here, but now the control
    // position is always relative, so it is hard to make it actually
    // wrap, aside from spinning at 10Hz for an hour.
    //
    // So we just assume it will be OK.

    co_await dut_->Command("d stop");

    fmt::print("SUCCESS\n");

    co_return;
  }

  boost::asio::awaitable<void> ValidateStayWithin() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    Controller::PidConstants pid;
    pid.kp = 2;
    co_await dut_->ConfigurePid(pid);
    co_await CommandFixtureRigid();
    constexpr double kSpeed = 2.0;

    for (const double feedforward : {0.0, -0.1, 0.1}) {
      for (const double max_torque : {0.35, 0.15}) {
        for (const double bounds_low : {kNaN, -0.5, -1.5}) {
          for (const double bounds_high : { kNaN, 0.5, 1.5}) {
            fmt::print("Testing bounds {}-{} max_torque={} feedforward={}\n",
                       bounds_low, bounds_high, max_torque, feedforward);

            co_await dut_->Command("d index 0");
            co_await fixture_->Command("d index 0");

            co_await dut_->Command(
                fmt::format("d within {} {} {} f{}",
                            bounds_low, bounds_high, max_torque, feedforward));

            co_await fixture_->Command(
                fmt::format("d pos nan {} 0.25",
                            -kSpeed * options_.transducer_scale));
            co_await Sleep(3.0 / kSpeed);
            const double low_position =
                options_.transducer_scale *
                fixture_->servo_stats().position;
            const double low_torque = current_torque_Nm_;

            co_await fixture_->Command(
                fmt::format("d pos nan {} 0.25", kSpeed * options_.transducer_scale));
            co_await Sleep(6.0 / kSpeed);
            const double high_position =
                options_.transducer_scale *
                fixture_->servo_stats().position;
            const double high_torque = current_torque_Nm_;


            co_await fixture_->Command("d stop");
            co_await dut_->Command("d stop");

            auto evaluate = [max_torque, feedforward](
                auto bounds, auto position, auto torque, auto name) {
              if (std::isnan(bounds) || max_torque < 0.2) {
                // We should not have stopped at any lower bound and our
                // torque should have been similarly low.
                if (std::isnan(bounds)) {
                  if (std::abs(torque - feedforward) > 0.07) {
                    throw mjlib::base::system_error::einval(
                        fmt::format("Unexpected {} torque present {} != {}",
                                    name, torque, feedforward));
                  }
                } else {
                  if (std::abs(std::abs(torque) - max_torque) > 0.08) {
                    throw mjlib::base::system_error::einval(
                        fmt::format(
                            "Should have pushed through bounds, torque |{}| != {}",
                            torque, max_torque));
                  }
                }
                if (std::abs(position) < 1.8) {
                  throw mjlib::base::system_error::einval(
                      fmt::format(
                          "{} bound unset, yet position didn't go far: |{}| < 1.8",
                          name, position));
                }
              } else {
                if (std::abs(torque) < 0.12) {
                  throw mjlib::base::system_error::einval(
                      fmt::format("Insufficient {} torque: |{}| < 0.10",
                                  name, torque));
                }
                if (std::abs(position - bounds) > 0.20) {
                  throw mjlib::base::system_error::einval(
                      fmt::format("{} bounds not constrained {} != {}",
                                  name, position, bounds));
                }
              }
            };

            evaluate(bounds_low, low_position, low_torque, "low");
            evaluate(bounds_high, high_position, high_torque, "high");

            co_await Sleep(1.0);
          }
        }
      }
    }

    fmt::print("SUCCESS\n");

    co_return;
  }

  boost::asio::awaitable<void> ValidateMaxSlip() {
    for (double max_slip : { std::numeric_limits<double>::quiet_NaN(), 0.05 }) {
      fmt::print("Testing max slip {}\n", max_slip);

      co_await dut_->Command("d stop");
      co_await fixture_->Command("d stop");

      // Start out with the fixture stopped and move the DUT at a fixed
      // velocity.
      auto pid = Controller::PidConstants();
      pid.kp = 2.0;
      pid.ki = 0.0;
      pid.kd = 0.05;
      pid.max_position_slip = max_slip;
      co_await dut_->ConfigurePid(pid);
      co_await CommandFixtureRigid();

      co_await dut_->Command("d rezero");

      co_await Sleep(0.5);

      const auto initial_dut = dut_->servo_stats();

      constexpr double kDesiredSpeed = 0.5;
      co_await dut_->Command(fmt::format("d pos nan {} 0.2", kDesiredSpeed));

      // Wait for a second, and verify that we're moving and that
      // position is tracking properly.
      co_await Sleep(1.0);
      const auto spinning_dut = dut_->servo_stats();
      const auto measured_speed =
          spinning_dut.position - initial_dut.position;
      if (std::abs(measured_speed - kDesiredSpeed) > 0.052) {
        throw mjlib::base::system_error::einval(
            fmt::format("Base speed not achieved |{} - {}| > 0.052",
                        measured_speed, kDesiredSpeed));
      }

      const auto position_error = spinning_dut.pid_position.error;
      if (std::abs(position_error) > 0.1f) {
        throw mjlib::base::system_error::einval(
            fmt::format("Base tracking not working |{}| > 4000",
                        position_error));
      }

      // Now we command the fixture to go slower with a higher maximum
      // torque.  That should cause the DUT to fall behind.
      co_await fixture_->Command(
          fmt::format("d pos nan {} 0.4",
                      options_.transducer_scale * 0.5 * kDesiredSpeed));

      co_await Sleep(2.0);

      const auto slow_dut = dut_->servo_stats();
      const auto slow_speed =
          (slow_dut.position -
           spinning_dut.position) / 2.0;
      if (std::abs(slow_speed - 0.5 * kDesiredSpeed) > 0.052) {
        throw mjlib::base::system_error::einval(
            fmt::format("DUT did not slow down |{} - {}| > 0.052",
                        slow_speed, 0.5 * kDesiredSpeed));
      }

      // Now we release the fixture.  In the absence of slip limiting,
      // the DUT will "catch up", making the overall velocity match what
      // was requested.
      co_await fixture_->Command("d stop");

      co_await Sleep(1.0);
      const auto final_dut = dut_->servo_stats();
      const auto final_speed =
          (final_dut.position -
           slow_dut.position);

      if (std::isfinite(max_slip)) {
        if (final_speed > 1.15 * kDesiredSpeed) {
          throw mjlib::base::system_error::einval(
              fmt::format("DUT inappropriately 'caught up' {} > {}",
                          final_speed, 1.05 * kDesiredSpeed));
        }
      } else {
        if (final_speed < 1.25 * kDesiredSpeed) {
          throw mjlib::base::system_error::einval(
              fmt::format("DUT did not 'catch up' {} < {}",
                          final_speed, 1.25 * kDesiredSpeed));
        }
      }
    }

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    co_return;
  }

  boost::asio::awaitable<void> ValidateSlipStopPosition() {
    co_await CommandFixtureRigid();

    for (const double slip : { std::numeric_limits<double>::quiet_NaN(), 0.04 }) {
      co_await dut_->Command("d stop");
      co_await fixture_->Command("d stop");

      co_await dut_->Command("d index 0");

      Controller::PidConstants pid;
      pid.kp = 5.0;
      pid.ki = 0.0;
      pid.kd = 0.2;
      pid.max_position_slip = slip;
      co_await dut_->ConfigurePid(pid);

      // Start out holding the DUT in place.
      co_await fixture_->Command(
          fmt::format("d pos nan 0 {}", options_.max_torque_Nm));

      // And command the DUT to move to a point far away.
      constexpr double kVelocity = 0.1;
      co_await dut_->Command(
          fmt::format("d pos nan {} {} s0.5", kVelocity,
                      0.5 * options_.max_torque_Nm));

      // It shouldn't be able to go anywhere because the fixture is
      // holding it in place.
      co_await Sleep(3.0);

      const auto before_letgo = dut_->servo_stats();

      // Now we stop the fixture, which should allow the DUT to move
      // once again.
      co_await fixture_->Command("d stop");

      co_await Sleep(1.0);
      const auto after_letgo = dut_->servo_stats();

      const double velocity =
          after_letgo.position - before_letgo.position;
      if (std::isfinite(slip)) {
        if (velocity > 1.5 * kVelocity) {
          throw mjlib::base::system_error::einval(
              fmt::format("DUT unexpectedly tried to catch up: {} > {}",
                          velocity, 1.5 * kVelocity));
        }
      } else {
        if (velocity < 2.0 * kVelocity) {
          throw mjlib::base::system_error::einval(
              fmt::format("DUT did not catch up: {} < {}",
                          velocity, 2.0 * kVelocity));
        }
      }

      // Now wait long enough to reach the stop position no matter
      // what.
      co_await Sleep(5.0);

      // Now try pulling in each direction.
      for (const double target : { 0.3, -0.3 }) {
        // Once we have reached the stop position, then the fixture can
        // drag it away and the control position will "stick" if slip is
        // enabled.
        co_await fixture_->Command("d index 0");
        co_await fixture_->Command(
            fmt::format("d pos nan 0.5 {} s{}",
                        options_.max_torque_Nm, target));
        co_await Sleep(1.0);

        co_await fixture_->Command("d stop");
        co_await Sleep(1.0);

        // Let's see if the fixture got pulled back to 0.0 or not.
        const auto final_fixture =
            std::abs(fixture_->servo_stats().position);
        if (std::isfinite(slip)) {
          if (final_fixture < 0.2) {
            throw mjlib::base::system_error::einval(
                fmt::format("Dragging DUT did not 'stick' {} < {}",
                            final_fixture, 0.2));
          }
        } else {
          if (final_fixture > 0.05) {
            throw mjlib::base::system_error::einval(
                fmt::format("DUT did not fully restore to target {} > {}",
                            final_fixture, 0.05));
          }
        }
      }
    }

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");
  }

  boost::asio::awaitable<void> ValidateSlipBounds() {
    Controller::PidConstants pid;
    pid.kp = 5.0;
    pid.ki = 0.0;
    pid.kd = 0.2;
    pid.position_min = -0.3;
    pid.position_max = 0.5;

    for (const double slip : { std::numeric_limits<double>::quiet_NaN(), 0.03}) {
      pid.max_position_slip = slip;

      for (const double direction : { -0.6, 0.6}) {

        co_await dut_->Command("d stop");
        co_await fixture_->Command("d stop");

        co_await dut_->Command("d index 0");
        co_await fixture_->Command("d index 0");

        co_await dut_->ConfigurePid(pid);
        co_await CommandFixtureRigid();

        // Tell the DUT to stay at 0.0
        co_await dut_->Command(
            fmt::format("d pos 0 0 {}", 0.5 * options_.max_torque_Nm));

        // Now drag it away in one direction or the other.
        co_await fixture_->Command(
            fmt::format("d pos nan 0.5 {} s{}",
                        options_.max_torque_Nm,
                        options_.transducer_scale * direction));

        co_await Sleep(2.0);

        // Now release the fixture.  If slip is turned on, we should
        // stay close to the bound, otherwise we should revert back to
        // 0.
        co_await fixture_->Command("d stop");
        co_await Sleep(1.0);

        const double desired =
            (direction > 0.0) ? pid.position_max : pid.position_min;
        const double last = dut_->servo_stats().position;
        const double last_desired = std::isfinite(slip) ? desired : 0.0;

        if (std::abs(last - last_desired) > 0.05) {
          throw mjlib::base::system_error::einval(
              fmt::format("DUT in unexpected position {} != {}",
                          last, last_desired));
        }
      }
    }

    co_return;
  }

  boost::asio::awaitable<void> ValidateDqIlimit() {
    Controller::PidConstants pid;
    pid.position_min = kNaN;
    pid.position_max = kNaN;

    pid.kp = 1.0;
    pid.ki = 0.0;
    pid.kd = 0.05;

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    // The dq ilimit shouldn't be able to go larger than the input
    // voltage permits.  Otherwise, it can wind-up, resulting in a
    // long time before torque stops being applied after time spent at
    // maximum speed.

    // It would be nice to validate this in the normal run, but to do
    // so we'd need to spin at maximum speed for some time, which
    // isn't probably super advisable at the normal 24V run of these
    // tests.  When we have control over the voltage input, then we
    // could consider making an actual automated test for this.

    co_return;
  }

  boost::asio::awaitable<void> ValidatePowerLimit() {
    // For this test, we'll configure the fixture a fixed resistance,
    // ask the DUT to drive things at some speed, and then vary the
    // configured power limit.

    {
      Controller::PidConstants pid;
      pid.ki = 0.0;
      pid.kp = 0.0;
      pid.kd = 0.15;
      co_await fixture_->ConfigurePid(pid);
    }

    Controller::PidConstants dut_pid;
    dut_pid.ki = 0.0;
    dut_pid.kp = 5.0;
    dut_pid.kd = 0.2;
    dut_pid.max_position_slip = 0.2;

    struct Test {
      double power_W;
      double expected_speed_Hz;
    } tests[] = {
      { 100.0, 4.04 },
      { 20.0, 3.12 },
      { 10.0, 1.90 },
      { 5.0, 1.30 },
    };

    std::string errors;

    for (const auto test : tests) {
      dut_pid.max_power_W = test.power_W;
      co_await dut_->ConfigurePid(dut_pid);

      co_await fixture_->Command(
          fmt::format("d pos nan 0 {}", options_.max_torque_Nm));

      co_await dut_->Command("d pos nan 4.0 1.0");
      co_await Sleep(0.5);
      const double start_pos = dut_->servo_stats().position;
      co_await Sleep(0.5);
      const double end_pos = dut_->servo_stats().position;
      const double average_speed = (end_pos - start_pos) / 0.5;

      fmt::print("Power {} / Speed {}\n", test.power_W, average_speed);

      if (RelativeError(average_speed, test.expected_speed_Hz) > 0.15) {
        if (!errors.empty()) { errors += "\n"; }
        errors +=
            fmt::format(
                "Speed {} != {} (within {}%)",
                average_speed, test.expected_speed_Hz, 0.15 * 100);
      }

      co_await dut_->Command("d stop");
      co_await fixture_->Command("d stop");
      co_await Sleep(0.5);
    }

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    if (!errors.empty()) {
      throw mjlib::base::system_error::einval(errors);
    }

    co_return;
  }

  boost::asio::awaitable<void> ValidateMaxVelocity() {
    co_await fixture_->Command("d stop");
    co_await dut_->Command("d stop");
    co_await dut_->Command("d rezero");

    Controller::PidConstants pid;

    const double kQCurrent = 1.5;

    // First, make sure it works in current mode.
    for (double velocity : { 1.0, 3.0, 7.0, 10.0 }) {
      for (double direction : { -1.0, 1.0 }) {
        for (bool current_mode : { false, true } ) {
          pid.max_velocity = velocity;
          co_await dut_->ConfigurePid(pid);

          if (current_mode) {
            co_await dut_->Command(
                fmt::format("d dq 0.0 {}", kQCurrent * direction));
          } else {
            co_await dut_->Command(
                fmt::format("d pos nan {} 1.0", 25 * direction));
          }
          co_await Sleep(0.5);
          const double start = dut_->servo_stats().position;
          co_await Sleep(0.5);
          const double end = dut_->servo_stats().position;
          co_await dut_->Command("d stop");
          co_await Sleep(2.0);

          const double average_speed = std::abs((end - start) / 0.5);

          fmt::print("velocity {} dir {}  current_mode {}  average_speed {}\n",
                     velocity, direction, current_mode, average_speed);

          constexpr double kVelocityDerate = 2.0;

          if (RelativeError(average_speed, velocity +
                            kVelocityDerate) > 0.2) {
            throw mjlib::base::system_error::einval(
                fmt::format(
                    "Speed {} != {} (within {}%)",
                    average_speed, velocity, 0.2 * 100));
          }
        }
      }
    }

    co_return;
  }

  boost::asio::awaitable<void> DoRezeroTest(double value) {
    co_await dut_->Command(fmt::format("d rezero {}", value));
    co_await Sleep(0.5);

    {
      // We should be within 0.5 of the desired.
      const auto pos = dut_->servo_stats().position;
      if (std::abs(pos - value) > 0.5) {
        throw mjlib::base::system_error::einval(
            fmt::format(
                "DUT rezero != {} ({})",
                value, pos));
      }
    }
  }

  boost::asio::awaitable<void> ValidateRezero() {
    co_await fixture_->Command("d stop");
    co_await dut_->Command("d stop");

    co_await DoRezeroTest(0.0);
    co_await DoRezeroTest(4.2);
    co_await DoRezeroTest(-7.9);
    co_await DoRezeroTest(-32764.0);
    co_await DoRezeroTest(-32767.0);
    co_await DoRezeroTest(32763.0);
    co_await DoRezeroTest(32767.0);

    co_return;
  }

  boost::asio::awaitable<void> ValidateVoltageModeControl() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");
    co_await fixture_->Command("d index 0");
    co_await dut_->Command("d index 0");

    Controller::PidConstants pid;
    pid.voltage_mode_control = true;
    pid.kp = 1.0;
    pid.ki = 0.0;
    pid.kd = 0.01;

    co_await dut_->ConfigurePid(pid);

    // With voltage mode control turned on, basic operation should
    // work as before modulo different position PID values.
    co_await RunBasicPositionTest(pid);

    co_return;
  }

  boost::asio::awaitable<void> ValidateFixedVoltageMode() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");
    co_await dut_->Command("d index 0");

    Controller::PidConstants pid;
    pid.voltage_mode_control = true;
    pid.kp = 1.0;
    pid.ki = 0.0;
    pid.kd = 0.01;
    pid.fixed_voltage_mode = true;
    pid.fixed_voltage_control_V = 0.45;

    co_await dut_->ConfigurePid(pid);

    // In this mode, the DUT ignores the encoder, so when we turn it
    // on to begin with, it will center on a random position.  So turn
    // it on first, then zero the fixture.
    co_await dut_->Command("d pos 0 0 0.2");
    co_await Sleep(0.5);
    co_await fixture_->Command("d index 0");

    // Despite burning power, all the basic position mode things that
    // don't involve jumps should work as is with fixed voltage mode.
    co_await RunBasicPositionVelocityTest(pid, 2.3);

    // However, we can use the fixture to drive the motor to the next
    // electrical phase and then it will stay there.

    co_return;
  }

  boost::asio::awaitable<void> ValidateBrakeMode() {
    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");
    co_await fixture_->Command("d index 0");

    co_await dut_->Command("d brake");
    co_await CommandFixtureRigid();

    struct BrakeTest {
      double velocity;
      double expected_torque;
    } brake_tests[] = {
      // These torques were just experimentally determined on the dyno
      // fixture's MiToot motor.
      { 0.0, 0.0 },
      { 2.0, 0.1 },
      { 5.0, 0.30 },
      { -2.0, -0.1 },
      { -5.0, -0.30 },
    };

    for (const auto& test : brake_tests) {
      co_await fixture_->Command(
          fmt::format("d pos nan {} {}",
                      test.velocity,
                      options_.max_torque_Nm));
      co_await Sleep(1.0);

      if (std::abs(current_torque_Nm_ - test.expected_torque) > 0.06) {
        throw mjlib::base::system_error::einval(
            fmt::format("brake torque not as expected {} != {} (within {})",
                        current_torque_Nm_, test.expected_torque, 0.06));
      }
    }

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    co_return;
  }

  boost::asio::awaitable<void> ValidateVelocityAccelLimits() {
    co_await fixture_->Command("d stop");
    co_await fixture_->Command("d index 0");

    co_await dut_->Command("d stop");
    co_await dut_->Command("d index 0");

    Controller::PidConstants pid;
    pid.kp = 2.0;
    pid.kd = 0.1;
    co_await dut_->ConfigurePid(pid);

    // We use a trajectory with a non-zero final velocity since we
    // have out of band knowledge that this results in the worst
    // possible loop timing.
    co_await dut_->Command(
        fmt::format("d pos 3.0 -0.5 {} v0.7 a0.2", options_.max_torque_Nm));

    const double step_s = 0.1;
    const int loops = 14 / step_s;
    double done_time = 0.0;
    std::map<double, double> fixture_position_history;
    std::map<double, double> fixture_velocity_history;

    for (int i = 0; i < loops; i++) {
      const double time_s = i * step_s;
      co_await Sleep(step_s);

      const double fixture_position =
          options_.transducer_scale *
          fixture_->servo_stats().position;
      const double fixture_velocity =
          options_.transducer_scale *
          fixture_->servo_stats().velocity;
      const double target_pos = 3.0 - time_s * 0.5;

      fixture_position_history.insert(
          std::make_pair(time_s, fixture_position));
      fixture_velocity_history.insert(
          std::make_pair(time_s, fixture_velocity));

      if (std::abs(fixture_velocity + 0.5) < 0.2 &&
          std::abs(fixture_position - target_pos) < 0.2 &&
          done_time == 0.0) {
        done_time = time_s;
      }

      const double kVelocityWindow = 0.5;
      if (time_s < kVelocityWindow) {
        continue;
      }

      const double old_position = fixture_position_history.lower_bound(
          time_s - kVelocityWindow)->second;
      const double average_velocity =
          (fixture_position - old_position) / kVelocityWindow;

      if (time_s > 12) {
        // We should be in the final parts of the trajectory moving at
        // -0.5 units per second.
        if (std::abs(average_velocity + 0.5) > 0.35) {
          throw mjlib::base::system_error::einval(
              fmt::format("Not stopped at end {} != 0",
                          average_velocity));
        }
      } else {
        // We are in-motion.
        if (std::abs(average_velocity) > 0.60) {
          throw mjlib::base::system_error::einval(
              fmt::format("Velocity while moving exceeded limit |{}| > 0.60",
                          fixture_velocity));
        }
        const double kAccelWindow = 1.0;
        if (i * step_s > kAccelWindow) {
          const double old_vel =
              fixture_velocity_history.lower_bound(
                  i * step_s - kAccelWindow)->second;
          const double measured_accel =
              (fixture_velocity - old_vel) / kAccelWindow;
          if (std::abs(measured_accel) > 0.60) {
            throw mjlib::base::system_error::einval(
                fmt::format("Measured acceleration exceeds limit |{}| > 0.60",
                            measured_accel));
          }
        }
      }
    }

    if (std::abs(done_time - 4.8) > 1.0) {
      throw mjlib::base::system_error::einval(
          fmt::format("Took wrong amount of time {} != 4.8", done_time));
    }

    co_await dut_->Command("d stop");
    co_await fixture_->Command("d stop");

    co_return;
  }

  boost::asio::awaitable<void> Sleep(double seconds) {
    boost::asio::deadline_timer timer(executor_);
    timer.expires_from_now(mjlib::base::ConvertSecondsToDuration(seconds));
    co_await timer.async_wait(boost::asio::use_awaitable);
    co_return;
  }

  boost::asio::awaitable<void> Init() {
    // First, connect to our two devices.
    fixture_.emplace(
        "fixture_",
        &file_writer_,
        client_->MakeTunnel(options_.fixture_id, kDebugTunnel, {}),
        options_);
    dut_.emplace(
        "dut_",
        &file_writer_,
        client_->MakeTunnel(options_.dut_id, kDebugTunnel, {}),
        options_);

    const auto expiration =
        mjlib::io::Now(executor_.context()) +
        boost::posix_time::seconds(3);

    co_await RunFor(
        executor_,
        fixture_->stream(),
        [&]() -> boost::asio::awaitable<bool> {
          co_await fixture_->Start();
          co_await dut_->Start();
          co_return false;
        },
        expiration);

    // Then the torque transducer.
    log_registrar_.Register("torque", &torque_signal_);
    boost::asio::co_spawn(
        executor_,
        std::bind(&Application::ReadTorque, this),
        ExceptionRethrower);

    // TODO: Then the current sense.

    // Give things a little bit to wait.
    while (torque_tare_count_ < kNumTareSamples) {
      co_await Sleep(0.5);
    }

    // Ensure that both devices start with a known configuration.
    co_await dut_->ConfigurePid(Controller::PidConstants());
    co_await fixture_->ConfigurePid(Controller::PidConstants());

    fmt::print("Initialization complete\n");

    co_return;
  }

  boost::asio::awaitable<void> ReadTorque() {
    boost::asio::serial_port port(executor_, options_.torque_transducer);

    boost::asio::streambuf streambuf;
    std::istream istr(&streambuf);
    while (true) {
      co_await boost::asio::async_read_until(
          port,
          streambuf,
          "\n",
          boost::asio::use_awaitable);

      std::string line;
      std::getline(istr, line);
      EmitTorque(line);
    }
  }

  void EmitTorque(const std::string& line) {
    std::vector<std::string> fields;
    boost::split(fields, line, boost::is_any_of(","));
    if (fields.size() < 4) { return; }
    TorqueTransducer data;
    try {
      data.timestamp = mjlib::io::Now(executor_.context());
      data.time_code = std::stol(fields.at(0));
      data.raw_value = std::stod(fields.at(1));
      data.torque_Nm =
          options_.transducer_scale * data.raw_value - torque_tare_;
      current_torque_Nm_ = data.torque_Nm;
      data.temperature_C = std::stod(fields.at(3));

      // We skip the first N samples to tare on startup.
      if (torque_tare_count_ < kNumTareSamples) {
        torque_tare_total_ += data.torque_Nm;
        torque_tare_count_++;
        if (torque_tare_count_ >= kNumTareSamples) {
          torque_tare_ = torque_tare_total_ / torque_tare_count_;
        }
      }
    } catch (std::invalid_argument& e) {
      fmt::print("Ignoring torque data: '{}': {}\n", line, e.what());
      // Ignore.
      return;
    }
    torque_signal_(&data);
  }

  class StatusPrinter {
   public:
    StatusPrinter(Application* parent, const std::string& header) {
      boost::asio::co_spawn(
          parent->executor_,
          std::bind(&StatusPrinter::Run, parent, header, done_),
          ExceptionRethrower);
    }

    ~StatusPrinter() {
      *done_ = true;
      fmt::print("\n");
    }

    static boost::asio::awaitable<void> Run(
        Application* app, const std::string& header,
        std::shared_ptr<bool> done) {
      const auto start = mjlib::io::Now(app->executor_.context());
      const auto elapsed = [&]() {
        const auto now = mjlib::io::Now(app->executor_.context());
        const auto delta = now - start;
        return fmt::format("{:02d}:{:02d}:{:02d}",
                           delta.hours(), delta.minutes(), delta.seconds());
      };

      while (!*done) {
        fmt::print("{} {} fixture=({})  dut=({})\r",
                   elapsed(), header, app->fixture_->stats(), app->dut_->stats());
        ::fflush(stdout);

        boost::asio::deadline_timer timer(app->executor_);
        timer.expires_from_now(boost::posix_time::seconds(1));
        co_await timer.async_wait(boost::asio::use_awaitable);
      }
    }

    std::shared_ptr<bool> done_ = std::make_shared<bool>(false);
  };

  boost::asio::any_io_executor executor_;
  boost::asio::executor_work_guard<
    boost::asio::any_io_executor> guard_{executor_};
  io::Selector<mp::AsioClient>* const client_selector_;
  const Options& options_;

  mp::AsioClient* client_ = nullptr;

  mjlib::telemetry::FileWriter file_writer_;
  LogRegistrar log_registrar_{&file_writer_};

  std::optional<Controller> fixture_;
  std::optional<Controller> dut_;

  boost::signals2::signal<void (const TorqueTransducer*)> torque_signal_;

  double torque_tare_total_ = 0.0;
  double torque_tare_count_ = 0;
  double torque_tare_ = 0.0;

  double current_torque_Nm_ = 0.0;
};

struct Context {
  boost::asio::io_context context;

  io::Selector<mp::AsioClient> default_client_selector{
    context.get_executor(), "client_type"};

  Options options;

  Context(int argc, char** argv) {
    mp::StreamAsioClientBuilder::Options default_stream_options;
    default_stream_options.stream.type = io::StreamFactory::Type::kSerial;
    default_stream_options.stream.serial_port = "/dev/fdcanusb";
    // If the baud rate does matter, 3mbit is a good one.
    default_stream_options.stream.serial_baud = 3000000;

    default_client_selector.Register<mp::StreamAsioClientBuilder>(
        "stream", default_stream_options);
    default_client_selector.set_default("stream");

    auto group = mjlib::base::ClippArchive().Accept(&options).group();
    group.merge(clipp::with_prefix(
                    "client.", default_client_selector.program_options()));

    mjlib::base::ClippParse(argc, argv, group);
  }
};

int do_main(int argc, char** argv) {
  {
    Context ctx(argc, argv);

    Application application{ctx.context, &ctx.default_client_selector, ctx.options};
    application.Start();
    try {
      ctx.context.run();
      return 0;
    } catch (std::runtime_error& e) {
      fmt::print(stderr, "Error: {}\n", e.what());
    }
  }

  // Clean up after an error by sending stop commands to everything.

  Context ctx(argc, argv);
  Application application{ctx.context, &ctx.default_client_selector, ctx.options};
  application.ErrorStop();
  ctx.context.run();

  return 1;
}

}
}
}

int main(int argc, char** argv) {
  return ::moteus::tool::do_main(argc, argv);
}
