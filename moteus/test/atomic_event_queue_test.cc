// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "moteus/atomic_event_queue.h"

#include <boost/test/auto_unit_test.hpp>

BOOST_AUTO_TEST_CASE(AtomicEventQueueTest) {
  moteus::AtomicEventQueue<4> dut;

  int count1 = 0;
  dut.Queue([&]() {
      count1++;
    });

  BOOST_TEST(count1 == 0);
  dut.Poll();
  BOOST_TEST(count1 == 1);
  count1 = 0;

  dut.Queue([&]() {
      count1++;
    });
  int count2 = 0;
  dut.Queue([&]() {
      count2++;
    });

  BOOST_TEST(count1 == 0);
  BOOST_TEST(count2 == 0);

  dut.Poll();
  BOOST_TEST(count1 == 1);
  BOOST_TEST(count2 == 1);
}
