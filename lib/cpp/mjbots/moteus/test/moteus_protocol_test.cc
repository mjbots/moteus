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

#include "mjbots/moteus/moteus_protocol.h"

#include <boost/test/auto_unit_test.hpp>

#include <string>

using namespace mjbots;

namespace tt = boost::test_tools;

namespace {
std::string Hexify(const moteus::CanData& frame) {
  char buf[10] = {};
  std::string result;
  for (size_t i = 0; i < frame.size; i++) {
    ::snprintf(buf, sizeof(buf) - 1, "%02x", static_cast<int>(frame.data[i]));
    result += buf;
  }
  return result;
}
}

BOOST_AUTO_TEST_CASE(QueryMake) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  const auto reply_size = moteus::Query::Make(&write_frame, {});

  BOOST_TEST(Hexify(frame) == "11001f01130d");
  BOOST_TEST(reply_size == 22);
}

BOOST_AUTO_TEST_CASE(QueryMakeEverything) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  moteus::Query::Format fmt;
  fmt.q_current = moteus::kFloat;
  fmt.d_current = moteus::kFloat;
  fmt.abs_position = moteus::kFloat;

  fmt.motor_temperature = moteus::kInt8;
  fmt.trajectory_complete = moteus::kInt8;
  fmt.home_state = moteus::kInt8;

  fmt.aux1_gpio = moteus::kInt8;
  fmt.aux2_gpio = moteus::kInt8;

  fmt.extra[0].register_number = moteus::Register::kEncoder0Position;
  fmt.extra[0].resolution = moteus::kFloat;
  fmt.extra[1].register_number = moteus::Register::kEncoder0Velocity;
  fmt.extra[1].resolution = moteus::kFloat;

  const auto reply_size = moteus::Query::Make(&write_frame, fmt);
  BOOST_TEST(Hexify(frame) == "11001c060110060a125e1e50");
  BOOST_TEST(reply_size == 53);
}


BOOST_AUTO_TEST_CASE(QueryMinimal) {
  moteus::CanData query_data{
    {
      0x24, 0x04, 0x00,
      0x0a, 0x00,  // mode
      0x10, 0x02,  // position
      0x00, 0xfe,  // velocity
      0x20, 0x00,  // torque
      0x23, 0x0d,
      0x20,  // voltage
      0x30,  // temperature
      0x40,  // fault
    },
    16,
  };

  const auto result = moteus::Query::Parse(&query_data);
  BOOST_TEST((result.mode == moteus::Mode::kPosition));
  BOOST_TEST(result.position == 0.0528);
  BOOST_TEST(result.velocity == -0.128);
  BOOST_TEST(result.torque == 0.320);
  BOOST_TEST(result.voltage == 16.0);
  BOOST_TEST(result.temperature == 48.0);
  BOOST_TEST(result.fault == 64);
}

BOOST_AUTO_TEST_CASE(QueryMaximal) {
  moteus::CanData query_data{
    {
      0x24, 0x07, 0x00,
      0x0a, 0x00,  // mode
      0x10, 0x02,  // position
      0x00, 0xfe,  // velocity
      0x20, 0x00,  // torque
      0x10, 0x00,  // q current
      0x40, 0x00,  // d current
      0x70, 0x00,  // abs_position

      0x20, 0x06, 0x0a,
      0x10,  // motor temp
      0x01,  // trajectory complete
      0x02,  // home state
      0x20,  // voltage
      0x30,  // temperature
      0x40,  // fault

      0x24, 0x02, 0x50,
      0x20, 0x00,  // encoder 0 position
      0x30, 0x00,  // encoder 0 velocity
    },
    33,
  };

  const auto result = moteus::Query::Parse(&query_data);
  BOOST_TEST((result.mode == moteus::Mode::kPosition));
  BOOST_TEST(result.position == 0.0528);
  BOOST_TEST(result.velocity == -0.128);
  BOOST_TEST(result.torque == 0.320);
  BOOST_TEST(result.q_current == 1.6);
  BOOST_TEST(result.d_current == 6.4);
  BOOST_TEST(result.abs_position == 0.0112);
  BOOST_TEST(result.motor_temperature == 16.0);
  BOOST_TEST(result.trajectory_complete == true);
  BOOST_TEST((result.home_state == moteus::HomeState::kOutput));
  BOOST_TEST(result.voltage == 16.0);
  BOOST_TEST(result.temperature == 48.0);
  BOOST_TEST(result.fault == 64);

  BOOST_TEST(result.extra[0].register_number == moteus::Register::kEncoder0Position);
  BOOST_TEST(result.extra[0].value == 0.0032);
  BOOST_TEST(result.extra[1].register_number == moteus::Register::kEncoder0Velocity);
  BOOST_TEST(result.extra[1].value == 0.012);
  BOOST_TEST(result.extra[2].register_number == std::numeric_limits<int16_t>::max());
}

BOOST_AUTO_TEST_CASE(GenericQueryMake) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  moteus::GenericQuery::Format fmt;

  fmt.values[0].register_number = moteus::Register::kEncoder0Position;
  fmt.values[0].resolution = moteus::kFloat;
  fmt.values[1].register_number = moteus::Register::kEncoder0Velocity;
  fmt.values[1].resolution = moteus::kFloat;

  fmt.values[2].register_number = moteus::Register::kAux1AnalogIn1;
  fmt.values[2].resolution = moteus::kInt16;
  fmt.values[3].register_number = moteus::Register::kAux1AnalogIn2;
  fmt.values[3].resolution = moteus::kInt16;

  const auto reply_size = moteus::GenericQuery::Make(&write_frame, {}, fmt);
  BOOST_TEST(Hexify(frame) == "1e501660");
  BOOST_TEST(reply_size == 16);
}

BOOST_AUTO_TEST_CASE(GenericQueryParse) {
  moteus::CanData query_data {
    {
      0x24, 0x04, 0x00,
      0x0a, 0x00,  // mode
      0x10, 0x02,  // position
      0x00, 0xfe,  // velocity
      0x20, 0x00,  // torque
      0x23, 0x0d,
      0x20,  // voltage
      0x30,  // temperature
      0x40,  // fault
    },
    16,
  };

  const auto result = moteus::GenericQuery::Parse(&query_data);
  BOOST_TEST(result.values[0].register_number == 0);
  BOOST_TEST(result.values[0].value == 10.0);

  BOOST_TEST(result.values[1].register_number == 1);
  BOOST_TEST(result.values[1].value == 0.0528);

  BOOST_TEST(result.values[2].register_number == 2);
  BOOST_TEST(result.values[2].value == -0.128);

  BOOST_TEST(result.values[3].register_number == 3);
  BOOST_TEST(result.values[3].value == 0.320);

  BOOST_TEST(result.values[4].register_number == 13);
  BOOST_TEST(result.values[4].value == 16.0);

  BOOST_TEST(result.values[5].register_number == 14);
  BOOST_TEST(result.values[5].value == 48.0);

  BOOST_TEST(result.values[6].register_number == 15);
  BOOST_TEST(result.values[6].value == 64.0);
}

BOOST_AUTO_TEST_CASE(PositionDefaults) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  const auto reply_size = moteus::PositionMode::Make(&write_frame, {}, {});

  BOOST_TEST(Hexify(frame) == "01000a0e200000000000000000");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(PositionMaximal) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  moteus::PositionMode::Command cmd;
  cmd.position = 0.25;
  cmd.velocity = 0.50;
  cmd.feedforward_torque = 1.0;
  cmd.kp_scale = 0.25;
  cmd.kd_scale = 0.125;
  cmd.maximum_torque = 8.0;
  cmd.stop_position = 2.0;
  cmd.watchdog_timeout = 0.2;
  cmd.velocity_limit = 5.0;
  cmd.accel_limit = 2.0;
  cmd.fixed_voltage_override = 4.0;

  moteus::PositionMode::Format fmt;
  fmt.position = moteus::kInt16;
  fmt.velocity = moteus::kInt16;
  fmt.feedforward_torque = moteus::kInt16;
  fmt.kp_scale = moteus::kInt16;
  fmt.kd_scale = moteus::kInt16;
  fmt.maximum_torque = moteus::kInt16;
  fmt.stop_position = moteus::kInt16;
  fmt.watchdog_timeout = moteus::kInt16;
  fmt.velocity_limit = moteus::kInt16;
  fmt.accel_limit = moteus::kInt16;
  fmt.fixed_voltage_override = moteus::kInt16;

  const auto reply_size = moteus::PositionMode::Make(&write_frame, cmd, fmt);

  BOOST_TEST(Hexify(frame) == "01000a040b20c409d0076400ff1fff0f2003204ec800204ed0072800");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(VFOCMake) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  moteus::VFOCMode::Command cmd;
  cmd.theta_rad = 0.2;
  cmd.voltage = 0.5;
  cmd.theta_rad_rate = 1.0;

  const auto reply_size = moteus::VFOCMode::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "0100070e183661823d0000003f0d1e83f9a23e");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(CurrentMake) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  moteus::CurrentMode::Command cmd;
  cmd.d_A = 1.0;
  cmd.q_A = 2.0;

  const auto reply_size = moteus::CurrentMode::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "0100090e1c000000400000803f");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(StayWithinDefaults) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  moteus::StayWithinMode::Command cmd;
  cmd.lower_bound = 1.0;
  cmd.upper_bound = 2.0;

  const auto reply_size = moteus::StayWithinMode::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "01000d0e400000803f00000040");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(StayWithinMaximal) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  moteus::StayWithinMode::Command cmd;
  cmd.lower_bound = 0.5;
  cmd.upper_bound = 1.0;
  cmd.feedforward_torque = 2.0;
  cmd.kp_scale = 0.25;
  cmd.kd_scale = 0.5;
  cmd.maximum_torque = 2.0;
  cmd.watchdog_timeout = 0.05;

  moteus::StayWithinMode::Format fmt;
  fmt.feedforward_torque = moteus::kFloat;
  fmt.kp_scale = moteus::kFloat;
  fmt.kd_scale = moteus::kFloat;
  fmt.maximum_torque = moteus::kFloat;
  fmt.watchdog_timeout = moteus::kFloat;

  const auto reply_size = moteus::StayWithinMode::Make(&write_frame, cmd, fmt);

  BOOST_TEST(Hexify(frame) == "01000d0c07400000003f0000803f000000400000803e0000003f00000040cdcc4c3d");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(BrakeMode) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  const auto reply_size = moteus::BrakeMode::Make(&write_frame, {}, {});

  BOOST_TEST(Hexify(frame) == "01000f");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(MakeStop) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  const auto reply_size = moteus::StopMode::Make(&write_frame, {}, {});

  BOOST_TEST(Hexify(frame) == "010000");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(GpioWrite) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  moteus::GpioWrite::Command cmd;
  cmd.aux1 = 2;
  cmd.aux2 = 4;
  const auto reply_size = moteus::GpioWrite::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "025c0204");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(OutputNearest) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  moteus::OutputNearest::Command cmd;
  cmd.position = 2.0;
  const auto reply_size = moteus::OutputNearest::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "0db00200000040");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(OutputExact) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);
  moteus::OutputExact::Command cmd;
  cmd.position = 3.0;
  const auto reply_size = moteus::OutputExact::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "0db10200004040");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(RequireReindex) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  const auto reply_size = moteus::RequireReindex::Make(&write_frame, {}, {});

  BOOST_TEST(Hexify(frame) == "01b20201");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(DiagnosticWrite) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  std::string msg = "tel stop\n";

  moteus::DiagnosticWrite::Command cmd;
  cmd.channel = 1;
  cmd.data = msg.data();
  cmd.size = msg.size();

  const auto reply_size = moteus::DiagnosticWrite::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "40010974656c2073746f700a");
  BOOST_TEST(reply_size == 0);
}

BOOST_AUTO_TEST_CASE(DiagnosticRead) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  const auto reply_size = moteus::DiagnosticRead::Make(&write_frame, {}, {});

  BOOST_TEST(Hexify(frame) == "420130");
  BOOST_TEST(reply_size == 51);
}

BOOST_AUTO_TEST_CASE(ClockTrim) {
  moteus::CanData frame;
  moteus::WriteCanData write_frame(&frame);

  moteus::ClockTrim::Command cmd;
  cmd.trim = 5;

  const auto reply_size = moteus::ClockTrim::Make(&write_frame, cmd, {});

  BOOST_TEST(Hexify(frame) == "097105000000");
  BOOST_TEST(reply_size == 0);
}
