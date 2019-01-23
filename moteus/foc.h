// Copyright 2018 Josh Pieper, jjp@pobox.com.
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

#include <algorithm>
#include <cmath>

#include "moteus/math.h"

namespace moteus {

// A helper to cache the result of sin and cos on a given quantity.
struct SinCos {
  SinCos(float theta) {
    while (theta < 0.0f) { theta += k2Pi; }
    while (theta >= k2Pi) { theta -= k2Pi; }

    int sin_index = std::max(0, std::min(511, static_cast<int>((512.0f / k2Pi) * theta)));
    int cos_index = ((128 - sin_index) + 512) % 512;

    s = table_[sin_index];
    c = table_[cos_index];
  }

  float s;
  float c;

  static const float table_[512];
};

struct DqTransform {
  DqTransform(const SinCos& sc, float a, float b, float c)
      : d((2.0f / 3.0f) *
          (a * sc.c +
           (kSqrt3_4 * sc.s - 0.5f * sc.c) * b +
           (-kSqrt3_4 * sc.s - 0.5f * sc.c) * c)),
        q((2.0f / 3.0f) *
          (-sc.s * a -
           (-kSqrt3_4 * sc.c - 0.5f * sc.s) * b -
           (kSqrt3_4 * sc.c - 0.5f * sc.s) * c)) {}

  const float d;
  const float q;
};

struct InverseDqTransform {
  InverseDqTransform(const SinCos& sc, float d, float q)
      : a(sc.c * d - sc.s * q),
        b((kSqrt3_4 * sc.s - 0.5f * sc.c) * d - (-kSqrt3_4 * sc.c - 0.5f * sc.s) * q),
        c((-kSqrt3_4 * sc.s - 0.5f * sc.c) * d - (kSqrt3_4 * sc.c - 0.5f * sc.s) * q) {}

  const float a;
  const float b;
  const float c;
};

struct ClarkTransform {
  ClarkTransform(float a, float b, float c)
      : x((2.0f * a - b  - c) * (1.0f / 3.0f)),
        y((b - c) * (1.0f / kSqrt3)) {}

  const float x;
  const float y;
};

struct InverseClarkTransform {
  InverseClarkTransform(float x, float y)
      : a(x),
        b((-x + kSqrt3 * y) / 2.0f),
        c((-x - kSqrt3 * y) / 2.0f) {}

  const float a;
  const float b;
  const float c;
};

struct ParkTransform {
  ParkTransform(const SinCos& sc, float x, float y)
      : d(sc.c * x + sc.s * y),
        q(sc.c * y - sc.s * x) {}

  const float d;
  const float q;
};

struct InverseParkTransform {
  InverseParkTransform(const SinCos& sc, float d, float q)
      : x(sc.c * d - sc.s * q),
        y(sc.c * q + sc.s * d) {}

  const float x;
  const float y;
};

}
