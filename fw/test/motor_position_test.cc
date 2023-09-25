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

#include "fw/motor_position.h"

#include <boost/test/auto_unit_test.hpp>
#include <boost/random.hpp>

#include "mjlib/micro/test/persistent_config_fixture.h"

using namespace moteus;

namespace {
constexpr float kDt = 0.0001f;

struct Context {
  mjlib::micro::test::PersistentConfigFixture pcf;
  mjlib::micro::TelemetryManager telemetry_manager{
    &pcf.pool, &pcf.command_manager, &pcf.write_stream, pcf.output_buffer};
  aux::AuxStatus aux1_status;
  aux::AuxStatus aux2_status;
  aux::AuxConfig aux1_config;
  aux::AuxConfig aux2_config;

  MotorPosition dut{&pcf.persistent_config, &telemetry_manager,
                    &aux1_status, &aux2_status,
                    &aux1_config, &aux2_config};

  Context() {
    dut.motor()->poles = 4;

    // Force everything to load from defaults and all callbacks to be
    // invoked.
    pcf.persistent_config.Load();
  }

  void Update() {
    dut.ISR_Update(kDt);
  }
};
}

BOOST_AUTO_TEST_CASE(MotorPositionBasicOperation) {
  Context ctx;

  // Assume the default config of a single absolute SPI encoder
  // attached to the rotor with a scaling factor of 1.0.
  ctx.aux1_status.spi.active = false;
  ctx.aux1_status.spi.value = 4096;
  ctx.aux1_status.spi.nonce = 1;

  ctx.dut.ISR_Update(kDt);

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.position_relative_valid == false);
    BOOST_TEST(status.theta_valid == false);
  }

  ctx.aux1_status.spi.active = true;
  ctx.aux1_status.spi.nonce++;

  ctx.dut.ISR_Update(kDt);

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.position_relative_raw == 0ll);
    BOOST_TEST(status.position_relative == 0.0f);

    BOOST_TEST(status.position_raw == 70368744177664ll);
    BOOST_TEST(status.position == 0.25f);

    BOOST_TEST(status.velocity == 0.0f);

    BOOST_TEST(status.theta_valid == true);
    BOOST_TEST(status.electrical_theta == 3.14159274f);
  }

  // Now make an update and verify that things change.
  ctx.aux1_status.spi.value = 4100;
  ctx.aux1_status.spi.nonce++;

  ctx.dut.ISR_Update(kDt);

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(std::abs(status.position_relative_raw - 34561064960ll) <= (2ll << 24));
    BOOST_TEST(status.position_relative == 0.000122070312f);

    BOOST_TEST(std::abs(status.position_raw - 70403305242624ll) <= (2ll << 24));
    BOOST_TEST(status.position == 0.25012207f);

    BOOST_TEST(status.velocity == 0.154212564f);

    BOOST_TEST(status.theta_valid == true);
    BOOST_TEST(status.electrical_theta == 3.14313507f);
  }

  ctx.dut.ISR_SetOutputPosition(1.5f);
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.position == 1.5f);
    BOOST_TEST(status.position_raw == 422212465065984ll);
  }

  ctx.aux1_status.spi.nonce++;

  ctx.dut.ISR_Update(kDt);

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.position_raw == 422231792418816ll);
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionStartupCapture,
                     * boost::unit_test::tolerance(1e-3)) {
  struct TestCase {
    uint32_t initial;
    float expected;
  };
  TestCase test_cases[] = {
    { 0,      0.0f },
    { 4096,   0.25f },
    { 12288, -0.25f },
  };

  for (const auto& test : test_cases) {
    Context ctx;

    ctx.aux1_status.spi.active = true;
    ctx.aux1_status.spi.value = test.initial;
    ctx.aux1_status.spi.nonce = 1;

    ctx.dut.ISR_Update(kDt);

    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.position == test.expected);
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionSetOutput,
                     *boost::unit_test::tolerance(1e-3f)) {
  const float test_cases[] = {
    0.0,
    -1.0,
    1.0,
    0.1,
    -0.1,
    200.0,
    -200.0,
    32767.9,
    -32767.9,
  };
  for (const auto test : test_cases) {
    Context ctx;
    ctx.pcf.persistent_config.Load();

    ctx.aux1_status.spi.active = true;
    ctx.aux1_status.spi.value = 0;
    ctx.aux1_status.spi.nonce = 1;

    ctx.dut.ISR_Update(kDt);
    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.position == 0.0f);
      BOOST_TEST(status.homed == MotorPosition::Status::kRotor);
    }

    ctx.dut.ISR_SetOutputPosition(test);
    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.position == test);
      BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    }

    ctx.aux1_status.spi.nonce = 2;
    ctx.dut.ISR_Update(kDt);
    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.position == test);
      BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionSetOutputNearest,
                     * boost::unit_test::tolerance(1e-3f)) {
  struct TestCase {
    float source;
    float rotor_to_output;
    float command;

    float expected;
  };
  TestCase test_cases[] = {
    { 0.25f, 1.0f, 0.25f,    0.25f },
    { 0.25f, 1.0f, 1.25f,    1.25f },
    { 0.25f, 1.0f, 2.50f,    2.25f },
    { 0.25f, 1.0f, 0.0f,     0.25f },
    { 0.25f, 1.0f, 0.50f,    0.25f },
    { 0.25f, 1.0f, -0.50f,  -0.75f },
    { 0.75f, 1.0f, -0.70,   -0.25f },
    { 0.75f, 1.0f, -0.80f,  -1.25f },
    { 0.75f, 1.0f, -0.85f,  -1.25f },
    { 0.00f, 1.0f, 0.00f,    0.00f },
    { 0.00f, 1.0f, 0.45f,    0.00f },
    { 0.00f, 1.0f, 0.55f,    1.00f },
    { 0.00f, 1.0f, -0.45f,   0.00f },
    { 0.00f, 1.0f, -0.55f,  -1.00f },

    { 0.00f, 0.5f,  0.00f,   0.00f },
    { 0.00f, 0.5f, -0.55f,  -0.50f },
    { 0.00f, 0.5f, -0.70f,  -0.50f },
    { 0.00f, 0.5f, -0.77f,  -1.00f },

    { 0.10f, 0.5f,  0.00f,   0.05f },
    { 0.10f, 0.5f, -0.50f,  -0.45f },
    { 0.10f, 0.5f, -0.55f,  -0.45f },
    { 0.10f, 0.5f, -0.70f,  -0.45f },
    { 0.10f, 0.5f, -0.77f,  -0.95f },

    { 0.25f, 1.0f,  32764.0f,  32764.25f },
    { 0.25f, 1.0f, -32764.0f, -32763.75f },
  };

  for (const auto& test : test_cases) {
    BOOST_TEST_CONTEXT("source " << test.source <<
                       "  ratio " << test.rotor_to_output <<
                       "  command " << test.command) {

      Context ctx;

      ctx.dut.config()->rotor_to_output_ratio = test.rotor_to_output;
      ctx.pcf.persistent_config.Load();

      ctx.dut.ISR_Update(kDt);
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.homed == MotorPosition::Status::kRelative);
      }

      ctx.aux1_status.spi.active = true;
      ctx.aux1_status.spi.value = test.source * 16384.0f;
      ctx.aux1_status.spi.nonce = 1;

      ctx.dut.ISR_Update(kDt);
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.homed == MotorPosition::Status::kRotor);
      }

      ctx.dut.ISR_SetOutputPositionNearest(test.command);
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.position == test.expected);
        BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
      }

      ctx.aux1_status.spi.nonce = 2;
      ctx.dut.ISR_Update(kDt);
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.position == test.expected);
        BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionNearestReferenceSource,
                     * boost::unit_test::tolerance(1e-3f)) {
  // A 5:1 reducer with a separate reference source.
  Context ctx;
  auto& config = *ctx.dut.config();
  config.sources[1].aux_number = 2;
  config.sources[1].type = MotorPosition::SourceConfig::kUart;
  config.sources[1].offset = 0;
  config.sources[1].cpr = 16384;
  config.sources[1].reference = MotorPosition::SourceConfig::kOutput;
  config.output.reference_source = 1;
  config.rotor_to_output_ratio = 0.2f;

  ctx.pcf.persistent_config.Load();

  ctx.aux1_status.spi.active = true;
  ctx.aux1_status.spi.value = 4096;
  ctx.aux1_status.spi.nonce++;

  ctx.aux2_status.uart.active = true;
  ctx.aux2_status.uart.value = 7372;
  ctx.aux2_status.uart.nonce++;

  // This should work out to a position of around 0.9 at the

  ctx.Update();
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.45f);
    BOOST_TEST(status.position_relative == 0.0f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // At this point, moving the reference source shouldn't have any
  // effect on the output.
  ctx.aux2_status.uart.value = 4000;

  for (int i = 0; i < 50; i++) {
    ctx.aux2_status.uart.nonce++;
    ctx.Update();
  }

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.45f);
    BOOST_TEST(status.position_relative == 0.0f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // But updating the primary source will.
  ctx.aux1_status.spi.value = 4200;
  for (int i = 0; i < 50; i++) {
    ctx.aux1_status.spi.nonce++;
    ctx.Update();
  }

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.45126f);
    BOOST_TEST(status.position_relative == 0.001266f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // If we command a "set output nearest", it will once again take
  // into account the reference source.
  ctx.dut.ISR_SetOutputPositionNearest(0.0f);
  ctx.Update();

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.25126f);
    BOOST_TEST(status.position_relative == 0.001266f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionReferenceSourceStartup,
                     * boost::unit_test::tolerance(1e-2f)) {
  struct TestCase {
    float rotor_offset = 0.0f;
    float aux_offset = 0.0f;
    float output_offset = 0.0f;

    float expected_value = 0.0f;
  };

  TestCase test_cases[] = {
    { 0.0f,  0.0f,  0.0f,     0.0f },
    { 0.0f,  0.0f,  0.1f,     0.1f },
    { 0.0f,  0.0f,  0.6f,    -0.4f },
    { 0.0f,  0.0f, -0.1f,    -0.1f },
    { 0.0f,  0.0f, -0.7f,     0.3f },
    { 0.3f,  0.0f,  0.0f,     0.0f },
    { 0.0f,  0.3f,  0.0f,     0.0f },
    { 0.0f, -0.3f,  0.0f,     0.0f },
    { 0.2f, -0.3f,  0.0f,     0.0f },
  };

  for (const auto& test : test_cases) {
    for (float output_position : { 0.0f,
                                  -0.15f, 0.15f,
                                  -0.35f, 0.35f,
                                   0.45f, -0.45f }) {
      for (float ratio : { 1.0f, 0.5f, 0.2f }) {
        BOOST_TEST_CONTEXT("rotor_offset " << test.rotor_offset <<
                           "  aux_offset " << test.aux_offset <<
                           "  output_offset " << test.output_offset <<
                           "  output_position " << output_position <<
                           "  ratio " << ratio) {
          // A N:1 reducer with a separate reference source.
          Context ctx;
          auto& config = *ctx.dut.config();
          config.sources[0].offset = test.rotor_offset * 16384;

          config.sources[1].aux_number = 2;
          config.sources[1].type = MotorPosition::SourceConfig::kUart;
          config.sources[1].offset = test.aux_offset * 16384;
          config.sources[1].cpr = 16384;
          config.sources[1].reference = MotorPosition::SourceConfig::kOutput;

          config.output.reference_source = 1;
          config.output.offset = test.output_offset;
          config.rotor_to_output_ratio = ratio;

          ctx.pcf.persistent_config.Load();

          ctx.aux1_status.spi.active = true;
          ctx.aux1_status.spi.value =
              static_cast<int>(
                  output_position / config.rotor_to_output_ratio * 16384 +
                  16384 * 10 -
                  test.rotor_offset * 16384) % 16384;
          ctx.aux1_status.spi.nonce++;

          ctx.Update();

          // We should be only in rotor sync.
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.homed == MotorPosition::Status::kRotor);
          }

          ctx.aux2_status.uart.active = true;
          ctx.aux2_status.uart.value =
              static_cast<int>(
                  output_position * 16384 +
                  16384 * 10 -
                  test.aux_offset * 16384) % 16384;
          ctx.aux2_status.uart.nonce++;

          ctx.Update();

          const float expected_value =
              MotorPosition::WrapBalancedCpr(
                  test.expected_value + output_position, 1.0f);
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
            BOOST_TEST(status.position == expected_value);
          }

          ctx.aux1_status.spi.nonce++;
          ctx.aux2_status.uart.nonce++;

          ctx.Update();

          // Nothing should have changed.
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
            BOOST_TEST(status.position == expected_value);
          }

          for (float nearest_offset : { 0.0f, 0.3f, 1.2f, 1.7f, 2.0f }) {
            for (float nearest_sign : { 1.0f, -1.0f }) {
              const float delta = nearest_offset * nearest_sign;
              BOOST_TEST_CONTEXT("delta " << delta) {
                ctx.dut.ISR_SetOutputPositionNearest(expected_value + delta);

                const auto status = ctx.dut.status();
                BOOST_TEST(status.homed == MotorPosition::Status::kOutput);

                const float offset_expected_value =
                    expected_value + std::round(delta);

                BOOST_TEST(status.position == offset_expected_value);
              }
            }
          }
        }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionIncrementalReferenceSource,
                     * boost::unit_test::tolerance(1e-3f)) {
  // An incremental output encoder, with an absolute reference source.
  Context ctx;
  auto& config = *ctx.dut.config();
  config.sources[1].aux_number = 2;
  config.sources[1].type = MotorPosition::SourceConfig::kQuadrature;
  config.sources[1].offset = 0;
  config.sources[1].cpr = 20000;
  config.sources[1].reference = MotorPosition::SourceConfig::kOutput;
  config.output.reference_source = 0;
  config.output.source = 1;
  config.commutation_source = 0;

  config.rotor_to_output_ratio = 1.0f;

  ctx.pcf.persistent_config.Load();

  ctx.aux1_status.spi.active = true;
  ctx.aux1_status.spi.value = 4096;
  ctx.aux1_status.spi.nonce++;

  ctx.aux2_status.quadrature.active = true;
  ctx.aux2_status.quadrature.value = 8999;

  // The "reference source" is at 0.25, so we should start out exactly
  // there.

  ctx.Update();
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.25f);
    BOOST_TEST(status.position_relative == 0.0f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // At this point, moving the reference source shouldn't have any
  // effect on the output.
  ctx.aux1_status.spi.value = 4000;

  for (int i = 0; i < 50000; i++) {
    ctx.aux1_status.spi.nonce++;
    ctx.Update();
  }

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.25f);
    BOOST_TEST(status.position_relative == 0.0f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // But updating the incremental output will.
  ctx.aux2_status.quadrature.value = 9155;
  for (int i = 0; i < 50; i++) {
    ctx.Update();
  }

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.2578f);
    BOOST_TEST(status.position_relative == 0.007797f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // If we command a "set output nearest", it will once again take
  // into account the reference source.
  ctx.dut.ISR_SetOutputPositionNearest(0.0f);
  ctx.Update();

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.2441f);
    BOOST_TEST(status.position_relative == 0.007797f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  // Advance the incremental by a full revolution.
  for (int i = 0; i < 20000; i++) {
    ctx.aux2_status.quadrature.value =
        (ctx.aux2_status.quadrature.value + 1) % 20000;
    for (int j = 0; j < 5; j++) {
      ctx.Update();
    }
  }

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(std::abs(status.position - 1.2409f) < 0.004f);
    BOOST_TEST(std::abs(status.position_relative - 1.00456f) < 0.004f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  ctx.dut.ISR_SetOutputPositionNearest(0.0f);
  ctx.Update();

  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.2441f);
    BOOST_TEST(std::abs(status.position_relative - 1.00456f) < 0.004f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionCompensation,
                     * boost::unit_test::tolerance(1e-2f)) {
  Context ctx;
  ctx.dut.config()->sources[0].compensation_table[0] = 0.1;
  ctx.dut.config()->sources[0].compensation_table[1] = 0.0;
  ctx.dut.config()->sources[0].compensation_table[2] = -0.05;
  ctx.dut.config()->sources[0].compensation_table[3] = -0.2;
  ctx.dut.config()->sources[0].pll_filter_hz = 0.1;
  ctx.pcf.persistent_config.Load();

  struct TestCase {
    uint32_t raw;
    float expected;
  };

  TestCase test_cases[] = {
    { 0, 819.2f },
    { 256, 1894.4f }, // exactly mid-bucket
    { 511, 1333.4f },
    { 512, 1331.2f },
    { 768, 768.0f }, // exactly mid-bucket
    { 1023, 615.0f },
    { 1024, 614.4f },
    { 1280, 460.8f }, // exactly mid-bucket
    { 1536, 15872.0f },
  };

  for (const auto& test : test_cases) {
    BOOST_TEST_CONTEXT("raw=" << test.raw) {
      ctx.aux1_status.spi.active = true;
      ctx.aux1_status.spi.value = test.raw;
      ctx.aux1_status.spi.nonce += 1;

      for (int i = 0; i < 10000; i++) {
        ctx.dut.ISR_Update(kDt);
      }

      const auto status = ctx.dut.status();
      BOOST_TEST(status.sources[0].compensated_value == test.expected);
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionSpiTransform,
                     * boost::unit_test::tolerance(5e-3f)) {
  struct TestCase {
    float start;
    float offset;
    float sign;
    float output_offset;
    int8_t output_sign;

    float expected;
  };

  TestCase test_cases[] = {
    { 0.0f, 0.0f,  1.0f,  0.0f,  1,   0.0f },
    { 0.0f, 0.1f,  1.0f,  0.0f,  1,   0.1f },
    { 0.0f, 0.7f,  1.0f,  0.0f,  1,  -0.3f },
    { 0.0f, 0.7f, -1.0f,  0.0f,  1,   0.3f },
    { 0.0f, -0.1f, 1.0f,  0.0f,  1,  -0.1f },
    { 0.0f, -0.2f, 1.0f,  0.0f,  1,  -0.2f },
    { 0.0f, -0.6f, 1.0f,  0.0f,  1,   0.4f },
    { 0.0f, 0.0f,  1.0f,  0.1f,  1,   0.1f },
    { 0.0f, 0.0f,  1.0f, -0.1f,  1,  -0.1f },
    { 0.0f, 0.0f,  1.0f,  0.3f,  1,   0.3f },
    { 0.3f, 0.0f,  1.0f,  0.3f,  1,  -0.4f },
    { 0.3f, 0.3f,  1.0f,  0.3f,  1,  -0.1f },

    { 0.0f, 0.0f,  1.0f,  0.0f, -1,   0.0f },
    { 0.0f, 0.1f,  1.0f,  0.0f, -1,  -0.1f },
    { 0.0f, 0.7f,  1.0f,  0.0f, -1,   0.3f },
    { 0.0f, 0.7f, -1.0f,  0.0f, -1,  -0.3f },
    { 0.0f, 0.7f, -1.0f,  0.0f, -1,  -0.3f },
    { 0.3f, 0.0f,  1.0f,  0.3f, -1,   0.4f },
    { 0.3f, 0.2f,  1.0f,  0.3f, -1,   0.2f },
  };

  for (const auto& test : test_cases) {
    BOOST_TEST_CONTEXT("start " << test.start <<
                       "  offset " << test.offset <<
                       "  sign " << test.sign <<
                       "  output_offset " << test.output_offset <<
                       "  output_sign " << static_cast<int>(test.output_sign)) {
      Context ctx;
      ctx.dut.config()->sources[0].offset = test.offset * 16384.0f;
      ctx.dut.config()->sources[0].sign = test.sign;
      ctx.dut.config()->output.offset = test.output_offset;
      ctx.dut.config()->output.sign = test.output_sign;

      ctx.aux1_status.spi.active = false;
      ctx.aux1_status.spi.value = test.start * 16384.0f;
      ctx.aux1_status.spi.nonce = 1;

      ctx.pcf.persistent_config.Load();

      ctx.dut.ISR_Update(kDt);
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.position_relative_valid == false);
      }

      ctx.aux1_status.spi.active = true;
      ctx.dut.ISR_Update(kDt);

      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.position == test.expected);
        BOOST_TEST(status.position_relative_valid == true);
        BOOST_TEST((status.homed == MotorPosition::Status::kRotor));
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionInfrequentUpdates) {
  Context ctx;

  ctx.aux1_status.spi.active = true;
  ctx.aux1_status.spi.value = 4096;
  ctx.aux1_status.spi.nonce = 1;

  ctx.dut.ISR_Update(kDt);
  BOOST_TEST(ctx.dut.status().sources[0].time_since_update == 0.0f);
  ctx.dut.ISR_Update(kDt);
  BOOST_TEST(ctx.dut.status().sources[0].time_since_update == (1 * kDt));
  ctx.dut.ISR_Update(kDt);
  BOOST_TEST(ctx.dut.status().sources[0].time_since_update == (2 * kDt));

  ctx.aux1_status.spi.nonce = 2;
  ctx.dut.ISR_Update(kDt);
  BOOST_TEST(ctx.dut.status().sources[0].time_since_update == 0.0f);
}

BOOST_AUTO_TEST_CASE(WrapBalancedCpr) {
  BOOST_TEST(MotorPosition::WrapBalancedCpr(40.0f, 100.0f) == 40.0f);
  BOOST_TEST(MotorPosition::WrapBalancedCpr(-40.0f, 100.0f) == -40.0f);
  BOOST_TEST(MotorPosition::WrapBalancedCpr(-55.0f, 100.0f) == 45.0f);
  BOOST_TEST(MotorPosition::WrapBalancedCpr(55.0f, 100.0f) == -45.0f);
}

BOOST_AUTO_TEST_CASE(MotorPositionBasicI2C) {
  Context ctx;
  auto& config = *ctx.dut.config();
  config.sources[1].aux_number = 2;
  config.sources[1].type = MotorPosition::SourceConfig::kI2C;

  config.sources[1].pll_filter_hz = 20;

  ctx.pcf.persistent_config.Load();

  ctx.aux2_status.i2c.devices[0].active = true;
  ctx.aux2_status.i2c.devices[0].value = 4096;
  ctx.aux2_status.i2c.devices[0].nonce = 1;

  ctx.dut.ISR_Update(kDt);
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.sources[1].active_theta == true);
    BOOST_TEST(status.sources[1].active_velocity == true);
    BOOST_TEST(status.sources[1].nonce == 1);
    BOOST_TEST(status.sources[1].filtered_value == 4096.0f);
  }
  // We'll update the sensor at 1000Hz.
  for (int i = 0; i < 10000; i++) {
    if ((i % 10) == 0) {
      ctx.aux2_status.i2c.devices[0].value = 4096 + (i / 10) % 2;
      ctx.aux2_status.i2c.devices[0].nonce += 1;
    }
    ctx.dut.ISR_Update(kDt);
  }
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(std::abs(status.sources[1].filtered_value - 4096.0f) < 2.0f);
    BOOST_TEST(std::abs(status.sources[1].velocity) < 30.0f);
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionHallSource) {
  // Hall sources impose constraints on CPR, and perform some
  // transformations between the source and the final state.  The
  // source is from 0-5, whereas the final state is transformed to
  // such that the CPR covers one full revolution of the rotor.

  constexpr float dt = kDt;

  struct TestValues {
    uint32_t count;

    uint32_t expected;
  };

  auto run = [&](auto& ctx, const auto& test_values) {
    ctx.pcf.persistent_config.Load();

    for (const auto& test : test_values) {
      ctx.aux1_status.hall.count = test.count;

      ctx.dut.ISR_Update(dt);

      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.sources[0].offset_value == test.expected);
      }
    }
  };

  {
    Context ctx;
    ctx.dut.config()->sources[0].type = MotorPosition::SourceConfig::kHall;
    ctx.dut.config()->sources[0].pll_filter_hz = 1.0;
    ctx.pcf.persistent_config.Load();
    ctx.aux1_status.hall.active = true;

    TestValues test_values[] = {
      { 0, 0 },
      { 1, 1 },
      { 2, 2 },
      { 3, 3 },
      { 4, 4 },
      { 5, 5 },
      { 0, 6 },
      { 1, 7 },
      { 2, 8 },
      { 3, 9 },
      { 4, 10 },
      { 5, 11 },
      { 0, 0 },
      { 1, 1 },
      { 0, 0 },
      { 5, 11 },
    };

    run(ctx, test_values);
  }
  {
    Context ctx;
    ctx.dut.config()->sources[0].type = MotorPosition::SourceConfig::kHall;
    ctx.dut.config()->sources[0].sign = -1;
    ctx.aux1_status.hall.active = true;

    TestValues test_values[] = {
      { 0, 0 },
      { 1, 11 },
      { 2, 10 },
      { 3, 9 },
      { 4, 8 },
      { 5, 7 },
      { 0, 6 },
      { 1, 5 },
      { 2, 4 },
      { 3, 3 },
      { 4, 2 },
      { 5, 1 },
      { 0, 0 },
      { 1, 11 },
      { 0, 0 },
      { 5, 1 },
    };

    run(ctx, test_values);
  }

  {
    Context ctx;
    ctx.dut.config()->sources[0].type = MotorPosition::SourceConfig::kHall;
    ctx.dut.config()->sources[0].sign = -1;
    ctx.dut.config()->sources[0].offset = 1;
    ctx.aux1_status.hall.active = true;

    TestValues test_values[] = {
      { 0, 11 },
      { 1, 10 },
      { 2, 9 },
      { 3, 8 },
      { 4, 7 },
      { 5, 6 },
      { 0, 5 },
      { 1, 4 },
      { 2, 3 },
      { 3, 2 },
      { 4, 1 },
      { 5, 0 },
      { 0, 11 },
      { 1, 10 },
      { 0, 11 },
      { 5, 0 },
    };

    run(ctx, test_values);
  }
  {
    Context ctx;
    ctx.dut.config()->sources[0].type = MotorPosition::SourceConfig::kHall;
    ctx.dut.config()->sources[0].sign = -1;
    ctx.dut.config()->sources[0].offset = -2;
    ctx.aux1_status.hall.active = true;

    TestValues test_values[] = {
      { 0, 2 },
      { 1, 1 },
      { 2, 0 },
      { 3, 11 },
      { 4, 10 },
      { 5, 9 },
      { 0, 8 },
      { 1, 7 },
      { 2, 6 },
      { 3, 5 },
      { 4, 4 },
      { 5, 3 },
      { 0, 2 },
      { 1, 1 },
      { 0, 2 },
      { 5, 3 },
    };

    run(ctx, test_values);
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionQuadratureTest) {
  for (int index = 0; index < 2; index++) {
    Context ctx;
    auto& config = *ctx.dut.config();
    config.sources[0].aux_number = 1;
    config.sources[0].type = MotorPosition::SourceConfig::kQuadrature;
    config.sources[0].pll_filter_hz = 100.0;

    if (index) {
      config.sources[0].incremental_index = 1;
      ctx.aux1_status.index.active = true;
    }
    ctx.pcf.persistent_config.Load();

    ctx.dut.ISR_Update(kDt);
    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.sources[0].active_velocity == false);
      BOOST_TEST(status.sources[0].active_theta == false);
      BOOST_TEST(status.position_relative_valid == false);
      BOOST_TEST(status.theta_valid == false);
      BOOST_TEST(status.homed == MotorPosition::Status::kRelative);
    }

    ctx.aux1_status.quadrature.active = true;
    ctx.aux1_status.quadrature.value = 100;

    ctx.Update();
    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.sources[0].active_velocity == true);
      BOOST_TEST(status.sources[0].active_theta == false);
      BOOST_TEST(status.sources[0].nonce == 1);
      BOOST_TEST(status.sources[0].offset_value == 100);
      BOOST_TEST(status.sources[0].filtered_value == 100.0f);
      BOOST_TEST(status.sources[0].velocity == 0.0f);

      BOOST_TEST(status.position_relative_valid == true);
      BOOST_TEST(status.position_relative == 0.0f);
      BOOST_TEST(status.position == 0.0f);
      BOOST_TEST(status.theta_valid == false);
      BOOST_TEST(status.homed == MotorPosition::Status::kRelative);
    }

    ctx.aux1_status.quadrature.value = 101;
    ctx.Update();
    {
      const auto status = ctx.dut.status();
      BOOST_TEST(status.sources[0].active_velocity == true);
      BOOST_TEST(status.sources[0].active_theta == false);
      BOOST_TEST(status.sources[0].nonce == 2);
      BOOST_TEST(status.sources[0].offset_value == 101);
      BOOST_TEST(status.sources[0].velocity == 39.4784164f);
      BOOST_TEST(status.position_relative_raw == 2147483648);
      BOOST_TEST(status.position_raw == 2147483648);

      BOOST_TEST(status.position_relative_valid == true);
      BOOST_TEST(status.theta_valid == false);
      BOOST_TEST(status.homed == MotorPosition::Status::kRelative);
    }

    if (index) {
      ctx.aux1_status.index.value = true;
      ctx.aux1_status.quadrature.value = 101;
      ctx.Update();
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.sources[0].active_velocity == true);
        BOOST_TEST(status.sources[0].active_theta == true);
        BOOST_TEST(status.sources[0].nonce == 3);
        BOOST_TEST(status.sources[0].offset_value == 0);

        BOOST_TEST(status.sources[0].filtered_value == 0.0f);
        BOOST_TEST(status.sources[0].velocity == 39.4784164f);

        BOOST_TEST(std::abs(status.position_relative_raw - 2147483648ll) <= (2ll << 24));
        BOOST_TEST(status.position_raw == 0);

        BOOST_TEST(status.position_relative_valid == true);
        BOOST_TEST(status.theta_valid == true);
        BOOST_TEST(status.homed == MotorPosition::Status::kRotor);
      }

      ctx.aux1_status.index.value = false;
      ctx.aux1_status.quadrature.value = 102;
      ctx.Update();
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.sources[0].active_velocity == true);
        BOOST_TEST(status.sources[0].active_theta == true);
        BOOST_TEST(status.sources[0].nonce == 4);
        BOOST_TEST(status.sources[0].offset_value == 1);
        BOOST_TEST(status.sources[0].filtered_value == 0.129120678f);
        BOOST_TEST(status.sources[0].velocity == 78.8026199f);

        BOOST_TEST(std::abs(status.position_relative_raw - 4362076160) <= (2ll << 24));
        BOOST_TEST(status.position_raw == 2214592512);
      }

      // And further index pulses don't do anything.
      ctx.aux1_status.index.value = true;
      ctx.aux1_status.quadrature.value = 103;
      ctx.Update();
      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.sources[0].active_velocity == true);
        BOOST_TEST(status.sources[0].active_theta == true);
        BOOST_TEST(status.sources[0].nonce == 5);
        BOOST_TEST(status.sources[0].offset_value == 2);
        BOOST_TEST(status.sources[0].filtered_value == 0.371147752f);
        BOOST_TEST(status.sources[0].velocity == 152.362015f);
        BOOST_TEST(std::abs(status.position_relative_raw - 8522825728ll) <= (2ll << 24));
        BOOST_TEST(status.position_raw == 6375342080);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionExternalIndex) {
  Context ctx;
  auto& config = *ctx.dut.config();
  config.sources[1].aux_number = 1;
  config.sources[1].type = MotorPosition::SourceConfig::kIndex;
  config.sources[1].offset = 4096;
  config.sources[1].cpr = 16384;
  config.sources[1].reference = MotorPosition::SourceConfig::kOutput;
  config.output.reference_source = 1;
  config.rotor_to_output_ratio = 0.1f;

  ctx.pcf.persistent_config.Load();

  ctx.aux1_status.index.active = true;
  ctx.aux1_status.spi.active = true;
  ctx.aux1_status.spi.value = 128;
  ctx.aux1_status.spi.nonce++;

  ctx.Update();
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kRotor);
    BOOST_TEST(status.position == 0.000778198242f);
    BOOST_TEST(status.position_relative == 0.0f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  ctx.aux1_status.spi.value = 256;
  ctx.aux1_status.spi.nonce++;

  ctx.Update();
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kRotor);
    BOOST_TEST(status.position == 0.00115966797f);
    BOOST_TEST(status.position_relative == 0.000381469727f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  ctx.aux1_status.index.value = true;
  ctx.aux1_status.spi.value = 256;
  ctx.aux1_status.spi.nonce++;

  ctx.Update();
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.201385498f);
    BOOST_TEST(status.position_relative == 0.000610351562f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }

  ctx.aux1_status.index.value = false;
  ctx.aux1_status.spi.value = 384;
  ctx.aux1_status.spi.nonce++;

  ctx.Update();
  {
    const auto status = ctx.dut.status();
    BOOST_TEST(status.error == MotorPosition::Status::kNone);
    BOOST_TEST(status.homed == MotorPosition::Status::kOutput);
    BOOST_TEST(status.position == 0.201889038f);
    BOOST_TEST(status.position_relative == 0.0011138916f);
    BOOST_TEST(status.position_relative_valid == true);
    BOOST_TEST(status.theta_valid == true);
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionSlew,
                     * boost::unit_test::tolerance(1e-3f)) {
  for (const float sign : {1.0f, -1.0f}) {
    for (const float offset : {0.0f, 0.2f}) {
      BOOST_TEST_CONTEXT("sign:" << sign <<
                         "  offset:" << offset) {
        Context ctx;
        auto& config = *ctx.dut.config();

        config.output.sign = sign;
        config.output.offset = offset;
        ctx.pcf.persistent_config.Load();

        ctx.aux1_status.spi.active = true;
        ctx.aux1_status.spi.value = 0;

        int64_t old_position_relative = 0;

        // Move in a triangle pattern through zero a few times in
        // single steps.
        for (int i = 0; i < 8192 * 8; i++) {
          const int sign = [&]() {
            if (i < 8192 * 2) {
              return 2;
            } else if (i < 8192 * 4) {
              return 16382;
            } else {
              return 2;
            }
          }();

          ctx.aux1_status.spi.value = (i + sign) % 16384;
          ctx.aux1_status.spi.nonce++;

          ctx.dut.ISR_Update(kDt);

          // We should verify that the reported position never changes
          // by more than a certain amount.

          const auto status = ctx.dut.status();
          const auto delta = status.position_relative_raw - old_position_relative;
          BOOST_TEST(std::abs(delta) < (4ll << 34));
          if (i > 1) {
            BOOST_TEST(std::abs(delta) > (1ll << 31));
          }
          old_position_relative = status.position_relative_raw;
          BOOST_TEST(status.position_relative_valid == true);
        }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionOutputSign,
                     * boost::unit_test::tolerance(1e-3f)) {
  for (const int spi_value : { 2048, 4096, 15360 }) {
    for (const int sign : {1, -1}) {
      for (const float offset : {0.0f, 0.2f, -0.3f}) {
        BOOST_TEST_CONTEXT("spi_value: " << spi_value <<
                           "  sign:" << sign <<
                           "  offset:" << offset) {
          Context ctx;
          auto& config = *ctx.dut.config();

          config.output.sign = sign;
          config.output.offset = offset;
          ctx.pcf.persistent_config.Load();

          ctx.aux1_status.spi.active = true;
          ctx.aux1_status.spi.value = spi_value;
          ctx.aux1_status.spi.nonce = 1;

          ctx.dut.ISR_Update(kDt);

          const float expected_position =
              ((spi_value >= 8192) ? (spi_value-16384) : spi_value) / 16384.0f;

          const float expected_theta =
              spi_value < 4096 ?
              1.57079637f : spi_value < 8192 ?
              3.14159274f :
              5.49932909f;

          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.position == (sign * expected_position + sign * offset));

            BOOST_TEST(status.velocity == 0.0f);

            BOOST_TEST(status.theta_valid == true);
            BOOST_TEST(status.electrical_theta == expected_theta);
          }

          // Now increase the source value.  This should result in position
          // going down, velocity going down, but etheta still going up, since
          // it isn't affected by output.sign.
          ctx.aux1_status.spi.value += 4;
          ctx.aux1_status.spi.nonce++;

          ctx.dut.ISR_Update(kDt);

          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.position_relative_valid == true);
            BOOST_TEST(std::abs(status.position_relative_raw - (sign * 34561064960ll)) <= (2ll << 24));
            BOOST_TEST((std::abs(status.position_relative -
                                 (sign * 0.000137329102f)) < 1e-4f));

            BOOST_TEST(status.position == (sign * (expected_position + 0.000137329f) + sign * offset));

            BOOST_TEST(status.velocity == sign * 0.154212564f);

            BOOST_TEST(status.theta_valid == true);
            BOOST_TEST(status.electrical_theta == (expected_theta + .00154239f));
          }

          // We need to verify that the different types of homing do the right
          // thing, and that offsets work in a usable way.  Preferably also
          // that offsets work the same way as they do for sources.

          ctx.dut.ISR_SetOutputPositionNearest(sign * 2.0f + sign * offset);
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.position == (sign * (2.0f + expected_position + 0.000137329f) + sign * offset));
          }

          ctx.dut.ISR_Update(kDt);
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.position == (sign * (2.0f + expected_position + 0.00015259f) + sign * offset));
          }

          ctx.dut.ISR_SetOutputPositionNearest(-sign * 2.0f);
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.position == (-sign * 2.0f + sign * expected_position + sign * offset));
          }

          ctx.dut.ISR_Update(kDt);
          {
            const auto status = ctx.dut.status();
            BOOST_TEST(status.position == (-sign * 2.0f + sign * expected_position + sign * offset));
          }
        }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(MotorPositionDrift) {
  // Floating point rounding can cause significant drift when the gear
  // ratio is set to certain values.

  for (double ratio : { 1.0, 0.5, 0.252525 }) {
    BOOST_TEST_CONTEXT("ratio " << ratio) {
      Context ctx;

      ctx.dut.config()->rotor_to_output_ratio = ratio;
      ctx.pcf.persistent_config.Load();

      // Assume the default config of a single absolute SPI encoder
      // attached to the rotor with a scaling factor of 1.0.
      ctx.aux1_status.spi.active = true;
      ctx.aux1_status.spi.value = 4096;
      ctx.aux1_status.spi.nonce = 1;

      ctx.dut.ISR_Update(kDt);

      boost::random::mt19937 rng;
      boost::random::normal_distribution dist(0.0, 2.0);
      for (int i = 0; i < 30000 * 200; i++) {
        ctx.aux1_status.spi.value = 4096 + static_cast<int>(dist(rng));
        ctx.aux1_status.spi.nonce++;

        ctx.dut.ISR_Update(kDt);
      }

      {
        const auto status = ctx.dut.status();
        BOOST_TEST(status.position_relative_valid == true);
        // Check for within so many counts of the 16384 count encoder.
        BOOST_TEST(std::abs(status.position_relative) < ((0.6 / 16384) * ratio));
      }
    }
  }
}
