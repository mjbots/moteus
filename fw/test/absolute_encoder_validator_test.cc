// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "fw/absolute_encoder_validator.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {
constexpr uint32_t kTimeoutMs = AbsoluteEncoderValidator::kZeroTimeoutMs;
}

// A non-zero reading activates immediately -- ideally, the encoder
// would not synthesise a non-zero default during its set-up window.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorActivatesOnFirstNonZero) {
  AbsoluteEncoderValidator dut;
  BOOST_TEST(dut.Update(false, true, 1234, 0) == true);
}

// An error reading never activates and clears any in-progress wait.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorErrorGate) {
  AbsoluteEncoderValidator dut;
  BOOST_TEST(dut.Update(false, false, 1234, 0) == false);
  BOOST_TEST(dut.Update(false, false, 0, 1) == false);
}

// The core bug shape: a long run of valid value=0 frames must NOT
// activate the validator while inside the timeout window.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorWaitsThroughZerosUntilTimeout) {
  AbsoluteEncoderValidator dut;
  uint32_t t = 100;
  BOOST_TEST(dut.Update(false, true, 0, t) == false);

  // Anywhere short of the timeout, zeros stay rejected.
  for (uint32_t dt : {1u, 10u, 100u, kTimeoutMs - 1u}) {
    BOOST_TEST(dut.Update(false, true, 0, t + dt) == false);
  }

  // Past the timeout, accept zeros as legitimate.
  BOOST_TEST(dut.Update(false, true, 0, t + kTimeoutMs) == true);
}

// When zeros are followed by a real (non-zero) reading inside the
// wait window, we accept the real value immediately rather than
// running the timeout out.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorZerosThenNonZeroActivatesImmediately) {
  AbsoluteEncoderValidator dut;
  uint32_t t = 1;
  for (int i = 0; i < 70; i++) {
    BOOST_TEST(dut.Update(false, true, 0, t) == false);
    t += 1;
  }
  BOOST_TEST(dut.Update(false, true, 1190336, t) == true);
}

// An error reading during the zero-wait restarts the wait from the
// next valid reading -- transients don't get to keep ticking us
// closer to the timeout.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorErrorDuringZeroWaitResets) {
  AbsoluteEncoderValidator dut;
  uint32_t t = 0;
  BOOST_TEST(dut.Update(false, true, 0, t) == false);
  t += kTimeoutMs / 2;
  BOOST_TEST(dut.Update(false, true, 0, t) == false);

  // Glitch: an error frame mid-wait.
  t += 1;
  BOOST_TEST(dut.Update(false, false, 0, t) == false);

  // Wait restarts on the next valid reading.
  t += 1;
  BOOST_TEST(dut.Update(false, true, 0, t) == false);
  const uint32_t restart_t = t;

  // Up to (almost) the full timeout from restart_t we still wait.
  BOOST_TEST(dut.Update(false, true, 0, restart_t + kTimeoutMs - 1) == false);

  // Past the new timeout we activate.
  BOOST_TEST(dut.Update(false, true, 0, restart_t + kTimeoutMs) == true);
}

// Once active, valid readings (including value=0 and large jumps)
// keep the source active.  An error drops it out and forces a fresh
// qualification.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorStaysActiveThenRequalifies) {
  AbsoluteEncoderValidator dut;
  uint32_t t = 0;

  // Get active on a real reading.
  BOOST_TEST(dut.Update(false, true, 100, t) == true);

  // Stay active across various subsequent valid readings.
  t += 1;
  BOOST_TEST(dut.Update(true, true, 0, t) == true);
  t += 1;
  BOOST_TEST(dut.Update(true, true, 4000000, t) == true);

  // Sensor glitch -> drop out.
  t += 1;
  BOOST_TEST(dut.Update(true, false, 0, t) == false);

  // After dropping out we must requalify; the previous "first valid"
  // anchor is cleared.
  t += 1;
  BOOST_TEST(dut.Update(false, true, 0, t) == false);
  const uint32_t requal_t = t;
  BOOST_TEST(dut.Update(false, true, 0, requal_t + kTimeoutMs - 1) == false);
  BOOST_TEST(dut.Update(false, true, 0, requal_t + kTimeoutMs) == true);
}

// The monotonic timestamp argument is wrap-safe (uint32 ms wraps at
// ~50 days).  A wait that straddles the wrap must still resolve
// correctly because we compare via unsigned subtraction.
BOOST_AUTO_TEST_CASE(AbsoluteEncoderValidatorTimestampWraps) {
  AbsoluteEncoderValidator dut;
  const uint32_t t0 = 0xFFFFFFFFu - (kTimeoutMs / 2);
  BOOST_TEST(dut.Update(false, true, 0, t0) == false);

  // Halfway through the timeout we should still wait, on the other
  // side of the wrap.
  const uint32_t t_mid = t0 + kTimeoutMs / 4;  // wraps
  BOOST_TEST(dut.Update(false, true, 0, t_mid) == false);

  // Past the full timeout (also wrapped) we activate.
  const uint32_t t_done = t0 + kTimeoutMs;  // wraps
  BOOST_TEST(dut.Update(false, true, 0, t_done) == true);
}
