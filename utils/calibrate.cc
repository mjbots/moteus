// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "utils/calibrate.h"

#include <algorithm>
#include <cmath>
#include <deque>

#include <fmt/format.h>

#include <Eigen/Core>

#include <boost/algorithm/string.hpp>

#include "mjlib/base/system_error.h"

namespace moteus {
namespace tool {

namespace {

constexpr double kPi = 3.14159265358979323846;
}

namespace detail {
File ParseFile(const std::vector<std::string>& lines_in) {
  std::deque<std::string> lines;
  std::copy(lines_in.begin(), lines_in.end(), std::back_inserter(lines));
  if (!boost::starts_with(lines.front(), "CAL start")) {
    mjlib::base::system_error::throw_if(
        true, "Does not start with magic line: " + lines.front());
  }

  lines.pop_front();

  File result;

  while (!lines.empty()) {
    const auto line = lines.front();
    lines.pop_front();

    if (line == "CAL done") {
      return result;
    }

    std::vector<std::string> fields;
    boost::split(fields, line, boost::is_any_of(" "));

    if (fields.size() < 3) {
      mjlib::base::system_error::throw_if(
          true, "malformed line: " + line);
    }

    const auto phase = std::stoi(fields[1]);
    const auto encoder = std::stoi(fields[2]);

    auto entry = Entry{phase, encoder};

    for (size_t i = 3; i < fields.size(); i++) {
      std::vector<std::string> keyval;
      boost::split(keyval, fields[i], boost::is_any_of("="));
      if (keyval.size() == 2) {
        const auto& key = keyval[0];
        const auto& val = keyval[1];
        if (key == "i1") {
          entry.i1 = 0.001 * std::stod(val);
        } else if (key == "i2") {
          entry.i2 = 0.001 * std::stod(val);
        } else if (key == "i3") {
          entry.i3 = 0.001 * std::stod(val);
        }
      }
    }

    if (fields[0] == "1") {
      result.phase_up.push_back(entry);
    } else if (fields[0] == "2") {
      result.phase_down.push_back(entry);
    } else {
      mjlib::base::system_error::throw_if(
          true, "unknown phase code: " + line);
    }
  }

  mjlib::base::system_error::throw_if(
      true, "Does not end with magic line");
  return {};
}
}  // namespace detail

namespace {

int WrapInt16(int value) {
  if (value > 32767) {
    return value - 65536;
  } else if (value < -32768) {
    return 65536 + value;
  }
  return value;
}

double WrapNegPiToPi(double value) {
  if (value >= -kPi && value <= kPi) { return value; }
  if (value > 0.0) {
    return std::fmod(value + kPi, 2 * kPi) - kPi;
  } else {
    return std::fmod(value - kPi, 2 * kPi) + kPi;
  }
}

Eigen::VectorXd Unwrap(const Eigen::VectorXd& value) {
  Eigen::VectorXd result(value.size());
  for (int i = 0; i < value.size(); i++) {
    if (i == 0) {
      result(i) = value(i);
    } else {
      result(i) = (result(i - 1) + WrapNegPiToPi(value(i) - result(i - 1)));
    }
  }

  return result;
}

Eigen::VectorXd Linspace(double start, double end, size_t count) {
  Eigen::VectorXd result(count);
  for (size_t i = 0; i < count; i++) {
    result(i) = (
        (i + 1) == count ?
        end :
        i * (end - start) / (count - 1) + start);
  }
  return result;
}

Eigen::VectorXd Range(double start, double end, size_t count) {
  Eigen::VectorXd result(count);
  for (size_t i = 0; i < count; i++) {
    result(i) = (i * (end - start) / count + start);
  }
  return result;
}

Eigen::VectorXd Interpolate(const Eigen::VectorXd& sample_points,
                            const Eigen::VectorXd& x,
                            const Eigen::VectorXd& y) {
  BOOST_ASSERT(x.size() > 1 && x.size() == y.size());

  int xindex = 0;

  Eigen::VectorXd result(sample_points.size());
  for (int i = 0; i < sample_points.size(); i++) {
    const auto point = sample_points[i];
    result(i) = [&]() {
      if (point < x[xindex]) {
        // Whoops?  This can only legimately happen at the very beginning.
        return y[xindex];
      }
      while ((xindex + 2) < x.size() &&
             point >= x[xindex + 1]) {
        xindex++;
      }

      if (point > x[xindex + 1]) {
        // We're past the end.
        return y[xindex + 1];
      }

      // Linearly interpolate.
      const double length = x[xindex + 1] - x[xindex];
      if (length == 0.0) {
        return y[xindex + 1];
      }
      double ratio = (point - x[xindex]) / length;
      return (y[xindex + 1] - y[xindex]) * ratio + y[xindex];
    }();
  }

  return result;
}

Eigen::VectorXd WindowAverage(const Eigen::VectorXd& values, int window_size) {
  Eigen::VectorXd result(values.size());
  for (int i = 0; i < values.size(); i++) {
    int start = i - window_size / 2;
    int end = i + window_size / 2;
    Eigen::VectorXd errs(end - start);
    auto wrap = [&](int v) -> int {
      if (v < 0) { return v + values.size(); }
      if (v >= values.size()) { return v - values.size(); }
      return v;
    };
    for (int j = start; j < end; j++) {
      errs[j - start] = WrapNegPiToPi(values[wrap(j)] - values[wrap(start)]);
    }
    result[i] = values[wrap(start)] + errs.mean();
  }

  return result;
}

}

CalibrationResult Calibrate(const std::vector<std::string>& lines) {
  auto file = detail::ParseFile(lines);

  if (file.phase_up.size() < 2 ||
      file.phase_down.size() < 2) {
    mjlib::base::system_error::throw_if(
        true, "one or more phases was empty");
  }

  // We discard the first entry from the phase up side, since it will
  // be bogus.
  file.phase_up.erase(file.phase_up.begin());

  double total_delta = 0;
  for (size_t i = 0; i < file.phase_up.size() - 1; i++) {
    total_delta += WrapInt16(file.phase_up[i + 1].encoder -
                             file.phase_up[i].encoder);
  }

  if (std::abs(std::abs(total_delta) - 65536) > 5000) {
    mjlib::base::system_error::throw_if(
        true, "phase_up did not traverse appropriate encoder distance");
  }

  CalibrationResult result;

  // Figure out inversion.
  if (total_delta < 0) {
    result.invert = true;
    for (auto& item : file.phase_up) { item.encoder = 65535 - item.encoder; }
    for (auto& item : file.phase_down) { item.encoder = 65535 - item.encoder; }
    total_delta *= -1;
  }

  // Next, figure out the number of poles.  We compare the total
  // encoder delta to the total phase delta.
  double total_phase = 0;
  for (size_t i = 0; i < file.phase_up.size() - 1; i++) {
    total_phase += WrapInt16(file.phase_up[i + 1].phase -
                             file.phase_up[i].phase);
  }

  const double ratio = total_phase / total_delta;
  const double remainder = std::abs(std::round(ratio) - ratio);

  const double kMaxRemainderError = 0.1;
  if (remainder > kMaxRemainderError) {
    mjlib::base::system_error::throw_if(
        true, fmt::format(
            "encoder not an integral multiple of phase, {} > {}",
            remainder, kMaxRemainderError));
  }

  result.total_phase = total_phase;
  result.total_delta = total_delta;
  result.ratio = ratio;

  result.poles = static_cast<int>(std::round(ratio) * 2);

  // Now we need to figure out the phase offset at select points.  We
  // interpolate and average the phase up and phase down sections.
  std::vector<detail::Entry> phase_up_by_encoder = file.phase_up;
  const auto encoder_sort = [](const auto& lhs, const auto& rhs) {
    return lhs.encoder < rhs.encoder;
  };
  std::sort(phase_up_by_encoder.begin(), phase_up_by_encoder.end(),
            encoder_sort);
  std::vector<detail::Entry> phase_down_by_encoder = file.phase_down;
  std::sort(phase_down_by_encoder.begin(), phase_down_by_encoder.end(),
            encoder_sort);

  double offset = phase_down_by_encoder[0].phase - phase_up_by_encoder[0].phase;
  if (std::abs(offset) > 32767) {
    // We need to shift it so that they start from the same place.
    auto change = static_cast<int>(-65536 * std::round(offset / 65536.0));
    for (auto& item : phase_down_by_encoder) { item.phase += change; }
  }

  Eigen::VectorXd phase_up_encoder(phase_up_by_encoder.size());
  for (size_t i = 0; i < phase_up_by_encoder.size(); i++) {
    phase_up_encoder[i] = phase_up_by_encoder[i].encoder;
  }

  Eigen::VectorXd phase_up_phase(phase_up_by_encoder.size());
  for (size_t i = 0; i < phase_up_by_encoder.size(); i++) {
    phase_up_phase[i] =
        2.0 * kPi / 65536.0 * phase_up_by_encoder[i].phase;
  }
  phase_up_phase = Unwrap(phase_up_phase);

  Eigen::VectorXd phase_down_encoder(phase_down_by_encoder.size());
  for (size_t i = 0; i < phase_down_by_encoder.size(); i++) {
    phase_down_encoder[i] = phase_down_by_encoder[i].encoder;
  }

  Eigen::VectorXd phase_down_phase(phase_down_by_encoder.size());
  for (size_t i = 0; i < phase_down_by_encoder.size(); i++) {
    phase_down_phase[i] =
        2.0 * kPi / 65536.0 * phase_down_by_encoder[i].phase;
  }
  phase_down_phase = Unwrap(phase_down_phase);

  Eigen::VectorXd xpos = Linspace(0, 65535.0, 10000);

  const Eigen::VectorXd pu_interp =
      Interpolate(xpos, phase_up_encoder, phase_up_phase);
  const Eigen::VectorXd pd_interp =
      Interpolate(xpos, phase_down_encoder, phase_down_phase);
  const Eigen::VectorXd avg_interp = 0.5 * (pu_interp + pd_interp);

  const Eigen::VectorXd expected =
      (2.0 * kPi / 65536) * (result.poles / 2) * xpos;

  Eigen::VectorXd err(expected.size());
  for (int i = 0; i < expected.size(); i++) {
    err[i] = WrapNegPiToPi(avg_interp[i] - expected[i]);
  }

  // Make the error seem reasonable, so unwrap if we happen to span
  // the pi boundary.
  if ((err.maxCoeff() - err.minCoeff()) > 1.5 * kPi) {
    for (int i = 0; i < err.size(); i++) {
      err[i] = (err[i] > 0) ? err[i] : (err[i] + 2 * kPi);
    }
  }

  auto avg_window = static_cast<int>(err.size() / result.poles);
  const Eigen::VectorXd avg_err = WindowAverage(err, avg_window);

  const Eigen::VectorXd offset_x = Range(0, 65536, 64);
  const Eigen::VectorXd offsets = Interpolate(offset_x, xpos, avg_err);

  for (int i = 0; i < offsets.size(); i++) {
    result.offset.push_back(offsets[i]);
  }

  return result;
}

}
}
