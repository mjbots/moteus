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

using namespace mjbots;

namespace tt = boost::test_tools;

BOOST_AUTO_TEST_CASE(MakeStop) {
  moteus::CanFrame frame;
  moteus::WriteCanFrame write_frame(&frame);
  moteus::StopMode::Make(&write_frame, {}, {});

  BOOST_TEST(frame.size == 3);
  BOOST_TEST(frame.data[0] == 0x01);
  BOOST_TEST(frame.data[1] == 0x00);
  BOOST_TEST(frame.data[2] == 0x00);
}
