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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace moteus {

constexpr float k2Pi = 6.283185307179586f;
constexpr float kPi = 3.141592653589793f;
constexpr float kSqrt3_4 = 0.86602540378f;
constexpr float kSqrt6 = 2.449489742783178f;
constexpr float kSqrt3 = 1.7320508075688772f;
constexpr float kSqrt2 = 1.4142135623730951f;

float WrapZeroToTwoPi(float) __attribute__((always_inline));

inline float WrapZeroToTwoPi(float x) {
  // We would use fmodf, but we're trying to be fast here and don't
  // care too much about accuracy.

  // const float mod = ::fmodf(x, k2Pi);

  const int32_t divisor = static_cast<int>(x / k2Pi);
  const float mod = x - divisor * k2Pi;
  return (mod >= 0.0f) ? mod : (mod + k2Pi);
}

int32_t RadiansToQ31(float) __attribute__((always_inline));

inline int32_t RadiansToQ31(float x) {
  // First we scale, then wrap, and finally convert out.
  const float scaled = x / k2Pi;
  // Now we wrap to be from 0 to 1.
  const int32_t i = static_cast<int>(scaled);
  float mod = scaled - i;
  if (mod < 0) { mod += 1.0f; }

  return static_cast<int32_t>(((mod > 0.5f) ? (mod - 1.0f) : mod) * 4294967296.0f);
}

namespace detail {
union float_long {
  float f;
  long l;
};
}

// We re-implement frexpf so that it can get stuck into the CCM RAM.
// Otherwise it calls out to the libc one, which will be much slower
// since it is in normal flash.
float my_frexpf(float x, int* pw2) __attribute((always_inline));

inline float my_frexpf(float x, int *pw2) {
  detail::float_long fl;
  long int i;

  fl.f = x;
  i = (fl.l >> 23) & 0x000000ff;
  i -= 0x7e;
  *pw2 = i;
  fl.l &= 0x807fffff;
  fl.l |= 0x3f000000;
  return fl.f;
}

float log2f_approx(float X) __attribute__((always_inline));

inline float log2f_approx(float X) {
  // From: http://openaudio.blogspot.com/2017/02/faster-log10-and-pow.html
  float Y, F;
  int E;
  F = my_frexpf(fabsf(X), &E);
  Y = 1.23149591368684f;
  Y *= F;
  Y += -4.11852516267426f;
  Y *= F;
  Y += 6.02197014179219f;
  Y *= F;
  Y += -3.13396450166353f;
  Y += E;
  return X == 1.0f ? 0.0f : Y;
}

float pow2f_approx(float x) __attribute__((always_inline));

inline float pow2f_approx(float x) {
  // From: https://gist.github.com/petrsm/079de9396d63e00d5994a7cc936ae9c7

  volatile union {
    float f;
    unsigned int i;
  } cvt;

  const float pi = static_cast<int>(x);
  const float pf = x - pi;
  cvt.i = (1 << 23) * (static_cast<int>(x) + 127);
  const float pow2i = cvt.f;
  float pow2f = 7.9204240219773237e-2f;

  pow2f = pow2f * pf + 2.2433836478672357e-1f;
  pow2f = pow2f * pf + 6.9645739499350319e-1f;
  pow2f = pow2f * pf + 9.9989296565051542e-1f;

  return pow2i * pow2f;
}

inline float FastAtan2(float y, float x) {
  // Based on https://math.stackexchange.com/a/1105038/81278

  // a := min (|x|, |y|) / max (|x|, |y|)
  const float abs_y = std::abs(y);
  const float abs_x = std::abs(x);
  // inject FLT_MIN in denominator to avoid division by zero
  const float a = std::min(abs_x, abs_y) / (std::max(abs_x, abs_y) + std::numeric_limits<float>::min());
  // s := a * a
  const float s = a * a;
  // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  // if |y| > |x| then r := 1.57079637 - r
  if (abs_y > abs_x) {
    r = 1.57079637f - r;
  }
    // if x < 0 then r := 3.14159274 - r
  if (x < 0.0f) {
    r = 3.14159274f - r;
  }
  // if y < 0 then r := -r
  if (y < 0.0f) {
    r = -r;
  }

  return r;
}
}
