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

#pragma once

#include <cmath>

#include "moteus/math.h"

namespace moteus {

// A helper to cache the result of sin and cos on a given quantity.
struct SinCos {
  SinCos(float theta) : s(std::sin(theta)), c(std::cos(theta)) {}

  const float s;
  const float c;
};

struct DqTransform {
  DqTransform(const SinCos& sc, float a, float b, float c)
      : d((2.0f / 3.0f) *
          (a * sc.c +
           (kSqrt3_4 * sc.s - 0.5f * sc.c) * b +
           (-kSqrt3_4 * sc.s - 0.5 * sc.c) * c)),
        q((2.0f / 3.0f) *
          (-sc.s * a -
           (-kSqrt3_4 * sc.c - 0.5 * sc.s) * b -
           (kSqrt3_4 * sc.c - 0.5 * sc.s) * c)) {}

  const float d;
  const float q;
};

struct ClarkTransform {
  ClarkTransform(float a, float b, float c)
      : x((2.0f * a - b  - c) * (1.0f / 3.0f)),
        y((b - c) * (1.0f / kSqrt3)) {}

  const float x;
  const float y;
};

struct ParkTransform {
  ParkTransform(const SinCos& sc, float x, float y)
      : d(sc.c * x + sc.s * y),
        q(sc.c * y - sc.s * x) {}

  const float d;
  const float q;
};

}
