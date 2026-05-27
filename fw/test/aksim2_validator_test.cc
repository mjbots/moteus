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

#include "fw/aksim2_validator.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {
constexpr uint32_t kCpr = Aksim2Validator::kCpr;
constexpr int kCount = Aksim2Validator::kStartupCount;

// Drive the validator with a constant valid reading and return the
// number of updates until it first reports active.
int CyclesToActive(Aksim2Validator* dut, uint32_t value) {
  for (int i = 1; i < 1000; i++) {
    if (dut->Update(false, true, value)) { return i; }
  }
  return -1;
}
}

// A run of consistent, valid readings activates only after the
// required number of consecutive samples -- never on the first one.
BOOST_AUTO_TEST_CASE(Aksim2ValidatorBasicActivation) {
  Aksim2Validator dut;

  for (int i = 1; i < kCount; i++) {
    BOOST_TEST(dut.Update(false, true, 1000) == false);
  }
  // The kStartupCount'th consistent reading activates it.
  BOOST_TEST(dut.Update(false, true, 1000) == true);
}

// A reading with the error flag set never qualifies and resets any
// progress.
BOOST_AUTO_TEST_CASE(Aksim2ValidatorErrorGate) {
  Aksim2Validator dut;

  // Some progress...
  BOOST_TEST(dut.Update(false, true, 1000) == false);
  BOOST_TEST(dut.Update(false, true, 1000) == false);

  // ...wiped out by an error reading.
  BOOST_TEST(dut.Update(false, false, 1000) == false);

  // Now it takes the full count again.
  BOOST_TEST(CyclesToActive(&dut, 1000) == kCount);
}

// The core of the bug: a stale/transient first reading (e.g. a
// not-yet-acquired 0) followed by the real position must NOT activate
// on the stale value.  The large jump restarts qualification, so it
// activates only once the real value has been stable.
BOOST_AUTO_TEST_CASE(Aksim2ValidatorRejectsStaleThenJump) {
  Aksim2Validator dut;

  const uint32_t real_value = 3 * kCpr / 4;  // 0.75 rev away from 0

  // A single stale 0 frame...
  BOOST_TEST(dut.Update(false, true, 0) == false);

  // ...then the real value arrives.  The jump resets the count, so it
  // takes kStartupCount further consistent readings to activate.
  for (int i = 1; i < kCount; i++) {
    BOOST_TEST(dut.Update(false, true, real_value) == false);
  }
  BOOST_TEST(dut.Update(false, true, real_value) == true);
}

// A jump larger than the threshold mid-qualification restarts the
// count; a small change does not.
BOOST_AUTO_TEST_CASE(Aksim2ValidatorJumpThreshold) {
  {
    // Small jitter (under threshold) keeps accumulating.
    Aksim2Validator dut;
    uint32_t v = 100000;
    bool active = false;
    for (int i = 0; i < kCount; i++) {
      active = dut.Update(false, true, v);
      v += (Aksim2Validator::kMaxStartupDelta / 2);  // under threshold
    }
    BOOST_TEST(active == true);
  }
  {
    // A jump over threshold on each sample never qualifies.
    Aksim2Validator dut;
    uint32_t v = 0;
    bool active = false;
    for (int i = 0; i < 20; i++) {
      active = dut.Update(false, true, v);
      v = (v + 2 * Aksim2Validator::kMaxStartupDelta) % kCpr;
    }
    BOOST_TEST(active == false);
  }
}

// The wrap-around shortest-path delta is respected: readings straddling
// the 0/CPR boundary are treated as close together.
BOOST_AUTO_TEST_CASE(Aksim2ValidatorWrapAround) {
  Aksim2Validator dut;

  // Alternate just below and just above the wrap point.
  const uint32_t below = kCpr - 10;
  const uint32_t above = 10;  // shortest-path delta is 20, tiny

  bool active = false;
  for (int i = 0; i < kCount + 1; i++) {
    active = dut.Update(false, true, (i % 2) ? above : below);
  }
  BOOST_TEST(active == true);
}

// Once active, the source stays active across valid readings (even with
// large motion) and only drops out on an error -- after which it must
// re-qualify, modelling a power-cycle during operation.
BOOST_AUTO_TEST_CASE(Aksim2ValidatorStaysActiveThenRequalifies) {
  Aksim2Validator dut;

  BOOST_TEST(CyclesToActive(&dut, 1000) == kCount);

  // Active now.  A big legitimate move keeps it active.
  BOOST_TEST(dut.Update(true, true, kCpr / 2) == true);
  BOOST_TEST(dut.Update(true, true, 0) == true);

  // An error (e.g. the sensor was power-cycled) drops it out.
  BOOST_TEST(dut.Update(true, false, 0) == false);

  // It must re-qualify from scratch, rejecting the post-glitch
  // transient just like at power-on.
  BOOST_TEST(dut.Update(false, true, 0) == false);
  const uint32_t settled = kCpr / 3;
  for (int i = 1; i < kCount; i++) {
    BOOST_TEST(dut.Update(false, true, settled) == false);
  }
  BOOST_TEST(dut.Update(false, true, settled) == true);
}
