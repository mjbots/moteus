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

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace moteus {
namespace test {

// Forward declaration (defined in spmsm_motor_simulator.h).
struct Mj5208Params;

// Piecewise-linear interpolation from sorted (x, y) points.
// Clamps to boundary values outside the data range.
class PiecewiseLinear {
 public:
  struct Point {
    float x;
    float y;
  };

  PiecewiseLinear() = default;

  explicit PiecewiseLinear(std::vector<Point> points)
      : points_(std::move(points)) {
    std::sort(points_.begin(), points_.end(),
              [](const Point& a, const Point& b) { return a.x < b.x; });
  }

  float evaluate(float x) const {
    if (points_.empty()) { return 0.0f; }
    if (points_.size() == 1 || x <= points_.front().x) {
      return points_.front().y;
    }
    if (x >= points_.back().x) {
      return points_.back().y;
    }

    // Find the segment containing x.
    for (size_t i = 1; i < points_.size(); i++) {
      if (x <= points_[i].x) {
        const float t = (x - points_[i - 1].x) /
                        (points_[i].x - points_[i - 1].x);
        return points_[i - 1].y + t * (points_[i].y - points_[i - 1].y);
      }
    }
    return points_.back().y;
  }

 private:
  std::vector<Point> points_;
};

// Abstract motor model interface.
//
// Provides electromagnetic and mechanical parameters that may vary
// with the operating point (current-dependent inductance, etc.).
class MotorModel {
 public:
  virtual ~MotorModel() = default;

  // Effective inductances at the current operating point.
  virtual float L_d(float i_d, float i_q) const = 0;
  virtual float L_q(float i_d, float i_q) const = 0;

  // Constant electromagnetic parameters.
  virtual float R() const = 0;
  virtual float lambda_m() const = 0;
  virtual int pole_pairs() const = 0;

  // Mechanical parameters.
  virtual float J() const = 0;
  virtual float B() const = 0;

  // Torque model (including saturation).
  virtual double current_to_torque(double i_q) const = 0;
};

// Simple motor model with constant (current-independent) inductance.
// Wraps the SpmsmMotorSimulator::Params struct for backward compatibility.
class SimpleMotorModel : public MotorModel {
 public:
  struct Params {
    float lambda_m = 0.0f;
    float R = 0.0f;
    float L = 0.0f;
    float J = 0.0f;
    float B = 0.0f;
    int pole_pairs = 0;

    float rotation_current_cutoff = 0.0f;
    float rotation_current_scale = 0.0f;
    float rotation_torque_scale = 0.0f;
  };

  SimpleMotorModel() {
    // Default to Mj5208 values.
    p_.lambda_m = 8.3f / 304.0f / (1.5f * 7);
    p_.R = 0.047f;
    p_.L = 28.6e-6f;
    p_.J = 5.6e-5f;
    p_.B = 3e-5f;
    p_.pole_pairs = 7;
    p_.rotation_current_cutoff = 22.5f;
    p_.rotation_current_scale = 0.02494f;
    p_.rotation_torque_scale = 0.6638f;
  }

  explicit SimpleMotorModel(const Params& p) : p_(p) {}

  float L_d(float, float) const override { return p_.L; }
  float L_q(float, float) const override { return p_.L; }

  float R() const override { return p_.R; }
  float lambda_m() const override { return p_.lambda_m; }
  int pole_pairs() const override { return p_.pole_pairs; }

  float J() const override { return p_.J; }
  float B() const override { return p_.B; }

  double current_to_torque(double current) const override {
    const double abs_current = std::abs(current);
    const double torque_constant = 1.5 * p_.pole_pairs * p_.lambda_m;
    const double cutoff = p_.rotation_current_cutoff;

    if (abs_current < cutoff) {
      return current * torque_constant;
    }

    const double rotation_extra =
        p_.rotation_torque_scale *
        std::log2(1.0 + (abs_current - cutoff) * p_.rotation_current_scale);
    const double unsigned_torque = cutoff * torque_constant + rotation_extra;
    return std::copysign(unsigned_torque, current);
  }

  const Params& simple_params() const { return p_; }
  Params& simple_params() { return p_; }

 private:
  Params p_;
};

// Motor model with current-dependent inductance (saturation).
//
// L_d and L_q are piecewise-linear functions of i_d (the q-axis
// dependence is negligible per measurements).  All other parameters
// are the same as the simple model.
class SaturatingMotorModel : public MotorModel {
 public:
  struct Params {
    PiecewiseLinear L_d_vs_i_d;
    PiecewiseLinear L_q_vs_i_d;

    float R = 0.0f;
    float lambda_m = 0.0f;
    int pole_pairs = 0;
    float J = 0.0f;
    float B = 0.0f;

    float rotation_current_cutoff = 0.0f;
    float rotation_current_scale = 0.0f;
    float rotation_torque_scale = 0.0f;
  };

  explicit SaturatingMotorModel(const Params& p) : p_(p) {}

  float L_d(float i_d, float) const override {
    return p_.L_d_vs_i_d.evaluate(i_d);
  }

  float L_q(float i_d, float) const override {
    return p_.L_q_vs_i_d.evaluate(i_d);
  }

  float R() const override { return p_.R; }
  float lambda_m() const override { return p_.lambda_m; }
  int pole_pairs() const override { return p_.pole_pairs; }

  float J() const override { return p_.J; }
  float B() const override { return p_.B; }

  double current_to_torque(double current) const override {
    const double abs_current = std::abs(current);
    const double torque_constant = 1.5 * p_.pole_pairs * p_.lambda_m;
    const double cutoff = p_.rotation_current_cutoff;

    if (abs_current < cutoff) {
      return current * torque_constant;
    }

    const double rotation_extra =
        p_.rotation_torque_scale *
        std::log2(1.0 + (abs_current - cutoff) * p_.rotation_current_scale);
    const double unsigned_torque = cutoff * torque_constant + rotation_extra;
    return std::copysign(unsigned_torque, current);
  }

  // Factory: MJ5208 with measured saturation curves.
  static std::shared_ptr<SaturatingMotorModel> Mj5208Saturating() {
    Params p;

    // L_d vs i_d (from coupling_baseline/README.md, q_A=0 summary).
    p.L_d_vs_i_d = PiecewiseLinear({
        {+2.5f, 25e-6f},
        {-2.5f, 39e-6f},
        {-7.5f, 41e-6f},
        {-12.5f, 48e-6f},
        {-17.5f, 61e-6f},
        {-22.5f, 73e-6f},
    });

    // L_q vs i_d (from coupling_baseline/README.md, cross-saturation).
    p.L_q_vs_i_d = PiecewiseLinear({
        {+5.0f, 42.4e-6f},
        {0.0f, 48.8e-6f},
        {-5.0f, 60.0e-6f},
        {-10.0f, 58.4e-6f},
        {-15.0f, 59.2e-6f},
        {-20.0f, 61.3e-6f},
        {-25.0f, 58.3e-6f},
    });

    // Same base parameters as Mj5208Params.
    constexpr float kKv = 304.0f;
    constexpr int kPolePairs = 7;
    constexpr float kKt = 8.3f / kKv;

    p.R = 0.047f;
    p.lambda_m = kKt / (1.5f * kPolePairs);
    p.pole_pairs = kPolePairs;
    p.J = 5.6e-5f;
    p.B = 3e-5f;

    p.rotation_current_cutoff = 22.5f;
    p.rotation_current_scale = 0.02494f;
    p.rotation_torque_scale = 0.6638f;

    return std::make_shared<SaturatingMotorModel>(p);
  }

 private:
  Params p_;
};

}  // namespace test
}  // namespace moteus
