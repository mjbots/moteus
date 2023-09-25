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

#include "fw/foc.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

BOOST_AUTO_TEST_CASE(FocTest1) {
  Cordic cordic;
  SinCos sin_cos = cordic.radians(0.0);

  DqTransform dq(sin_cos, 2.0, 0., 0.);
  BOOST_TEST(dq.d == 1.3333333333f);
  BOOST_TEST(dq.q == 0.0);

  ClarkTransform ct(2.0, 0., 0.);
  BOOST_TEST(ct.x == 1.3333333333f);
  BOOST_TEST(ct.y == 0.0);

  InverseClarkTransform ict(ct.x, ct.y);
  BOOST_TEST(ict.a == 1.3333333333f);
  BOOST_TEST(ict.b == -0.666666666666f);
  BOOST_TEST(ict.c == -0.666666666666f);

  ParkTransform pt(sin_cos, ct.x, ct.y);
  BOOST_TEST(pt.d == dq.d);
  BOOST_TEST(pt.q == dq.q);

  InverseParkTransform ipt(sin_cos, pt.d, pt.q);
  BOOST_TEST(ipt.x == ct.x);
  BOOST_TEST(ipt.y == ct.y);

  InverseDqTransform idq(sin_cos, pt.d, pt.q);
  BOOST_TEST(idq.a == ict.a);
  BOOST_TEST(idq.b == ict.b);
  BOOST_TEST(idq.c == ict.c);
}

BOOST_AUTO_TEST_CASE(FocTest2) {
  Cordic cordic;
  SinCos sin_cos = cordic.radians(kPi / 2.0f);

  DqTransform dq(sin_cos, 2.0, 0., 0.);
  BOOST_TEST(std::abs(dq.d) <= 1e-5);
  BOOST_TEST(dq.q == -1.33333333333f);

  ClarkTransform ct(2.0, 0., 0.);
  BOOST_TEST(ct.x == 1.333333333333f);
  BOOST_TEST(ct.y == 0.0f);

  InverseClarkTransform ict(ct.x, ct.y);
  BOOST_TEST(ict.a == 1.333333333333f);
  BOOST_TEST(ict.b == -0.66666666666f);
  BOOST_TEST(ict.c == -0.66666666666f);

  ParkTransform pt(sin_cos, ct.x, ct.y);
  BOOST_TEST(pt.d == dq.d);
  BOOST_TEST(pt.q == dq.q);

  InverseParkTransform ipt(sin_cos, pt.d, pt.q);
  BOOST_TEST(ipt.x == ct.x);
  BOOST_TEST(ipt.y == ct.y);

  InverseDqTransform idq(sin_cos, pt.d, pt.q);
  BOOST_TEST(idq.a == ict.a);
  BOOST_TEST(idq.b == ict.b);
  BOOST_TEST(idq.c == ict.c);
}
