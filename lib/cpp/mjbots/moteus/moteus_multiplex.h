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

#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace mjbots {
namespace moteus {

/// Each value can be sent or received as one of the following.
enum Resolution {
  kInt8 = 0,
  kInt16 = 1,
  kInt32 = 2,
  kFloat = 3,
  kIgnore,
};

/// A vocabulary type for the basic data in a CAN-FD frame.
struct CanFrame {
  uint8_t data[64] = {};
  uint8_t size = 0;
};

enum Multiplex : uint32_t {
  kWriteBase = 0x00,
  kWriteInt8 = 0x00,
  kWriteInt16 = 0x04,
  kWriteInt32 = 0x08,
  kWriteFloat = 0x0c,

  kReadBase = 0x10,
  kReadInt8 = 0x10,
  kReadInt16 = 0x14,
  kReadInt32 = 0x18,
  kReadFloat = 0x1c,

  kReplyBase = 0x20,
  kReplyInt8 = 0x20,
  kReplyInt16 = 0x24,
  kReplyInt32 = 0x28,
  kReplyFloat = 0x2c,

  kWriteError = 0x30,
  kReadError = 0x31,

  // # Tunneled Stream #
  kClientToServer = 0x40,
  kServerToClient = 0x41,
  kClientPollServer = 0x42,

  kNop = 0x50,
};

namespace {
template <typename T>
T Saturate(double value, double scale) {
  // TODO: Implement without numeric_limits
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const double scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();

  const double double_max = static_cast<T>(max);
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  if (scaled < -double_max) { return -max; }
  if (scaled > double_max) { return max; }
  return static_cast<T>(scaled);
}
}

/// This class can be used to append values to the end of a CAN-FD
/// frame.
class WriteCanFrame {
 public:
  WriteCanFrame(CanFrame* frame) : data_(&frame->data[0]), size_(&frame->size) {}
  WriteCanFrame(uint8_t* data, uint8_t* size) : data_(data), size_(size) {}

  template <typename T, typename X>
  void Write(X value_in) {
    T value = static_cast<T>(value_in);
    if (sizeof(value) + *size_ > 64) {
      abort();
    }

#ifndef __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures supported"
#endif
    ::memcpy(&data_[*size_],
             reinterpret_cast<const char*>(&value),
             sizeof(value));
    *size_ += sizeof(value);
  }

  void WriteVaruint(uint32_t value) {
    while (true) {
      auto this_byte = value & 0x7f;
      value >>= 7;
      this_byte |= ((value != 0) ? 0x80 : 0x00);
      Write<int8_t>(this_byte);

      if (value == 0) { break; }
    }
  }

  void Write(const char* data, size_t size) {
    if ((size + *size_) > 64) {
      abort();
    }
    std::memcpy(&data_[*size_], data, size);
    *size_ += size;
  }

  void WriteInt(int32_t value, Resolution res) {
    switch (res) {
      case Resolution::kInt8: {
        Write<int8_t>(std::max(-127, std::min(127, value)));
      }
      case Resolution::kInt16: {
        Write<int16_t>(std::max(-32767, std::min(32767, value)));
      }
      case Resolution::kInt32: {
        Write<int32_t>(value);
      }
      case Resolution::kFloat: {
        Write<float>(static_cast<float>(value));
      }
      case Resolution::kIgnore: {
        abort();
      }
    }
  }

  void WriteMapped(
      double value,
      double int8_scale, double int16_scale, double int32_scale,
      Resolution res) {
    switch (res) {
      case Resolution::kInt8: {
        Write<int8_t>(Saturate<int8_t>(value, int8_scale));
        break;
      }
      case Resolution::kInt16: {
        Write<int16_t>(Saturate<int16_t>(value, int16_scale));
        break;
      }
      case Resolution::kInt32: {
        Write<int32_t>(Saturate<int32_t>(value, int32_scale));
        break;
      }
      case Resolution::kFloat: {
        Write<float>(static_cast<float>(value));
        break;
      }
      case Resolution::kIgnore: {
        abort();
      }
    }
  }

  void WritePosition(double value, Resolution res) {
    WriteMapped(value, 0.01, 0.0001, 0.00001, res);
  }

  void WriteVelocity(double value, Resolution res) {
    WriteMapped(value, 0.1, 0.00025, 0.00001, res);
  }

  void WriteAccel(double value, Resolution res) {
    WriteMapped(value, 0.05, 0.001, 0.00001, res);
  }

  void WriteTorque(double value, Resolution res) {
    WriteMapped(value, 0.5, 0.01, 0.001, res);
  }

  void WritePwm(double value, Resolution res) {
    WriteMapped(value,
                1.0 / 127.0,
                1.0 / 32767.0,
                1.0 / 2147483647.0,
                res);
  }

  void WriteVoltage(double value, Resolution res) {
    WriteMapped(value, 0.5, 0.1, 0.001, res);
  }

  void WriteTemperature(float value, Resolution res) {
    WriteMapped(value, 1.0, 0.1, 0.001, res);
  }

  void WriteTime(double value, Resolution res) {
    WriteMapped(value, 0.01, 0.001, 0.000001, res);
  }

  void WriteCurrent(double value, Resolution res) {
    WriteMapped(value, 1.0, 0.1, 0.001, res);
  }

 private:
  uint8_t* const data_;
  uint8_t* const size_;
};

/// Determines how to group registers when encoding them to minimize
/// the required bytes.
class WriteCombiner {
 public:
  WriteCombiner(WriteCanFrame* frame,
                int8_t base_command,
                uint32_t start_register,
                const Resolution* resolutions,
                size_t resolutions_size)
      : frame_(frame),
        base_command_(base_command),
        start_register_(start_register),
        resolutions_(resolutions),
        resolutions_size_(resolutions_size) {}

  ~WriteCombiner() {
    if (offset_ != resolutions_size_) {
      ::abort();
    }
  }

  bool MaybeWrite() {
    const auto this_offset = offset_;
    offset_++;

    if (current_resolution_ == resolutions_[this_offset]) {
      // We don't need to write any register operations here, and the
      // value should go out only if requested.
      return current_resolution_ != Resolution::kIgnore;
    }
    // We need to do some kind of framing.  See how far ahead the new
    // resolution goes.
    const auto new_resolution = resolutions_[this_offset];
    current_resolution_ = new_resolution;

    // We are now in a new block of ignores.
    if (new_resolution == Resolution::kIgnore) {
      return false;
    }

    int count = 1;
    for (size_t i = this_offset + 1;
         i < resolutions_size_ && resolutions_[i] == new_resolution;
         i++) {
      count++;
    }

    int8_t write_command = base_command_ + [&]() {
      switch (new_resolution) {
        case Resolution::kInt8: return 0x00;
        case Resolution::kInt16: return 0x04;
        case Resolution::kInt32: return 0x08;
        case Resolution::kFloat: return 0x0c;
        case Resolution::kIgnore: {
          abort();
        }
      }
      return 0x00;
    }();

    if (count <= 3) {
      // Use the shorthand formulation.
      frame_->Write<int8_t>(write_command + count);
    } else {
      // Nope, the long form.
      frame_->Write<int8_t>(write_command);
      frame_->Write<int8_t>(count);
    }
    if ((start_register_ + this_offset) > 127) {
      abort();
    }
    frame_->Write<int8_t>(start_register_ + this_offset);
    return true;
  }

 private:
  WriteCanFrame* const frame_;
  int8_t base_command_ = 0;
  uint32_t start_register_ = 0;
  const Resolution* const resolutions_;
  size_t resolutions_size_ = 0;

  Resolution current_resolution_ = Resolution::kIgnore;
  size_t offset_ = 0;
};

/// Read typed values from a CAN frame.
class MultiplexParser {
 public:
  MultiplexParser(const CanFrame* frame)
      : data_(&frame->data[0]),
        size_(frame->size) {}
  MultiplexParser(const uint8_t* data, uint8_t size)
      : data_(data),
        size_(size) {}

  struct Result {
    bool done = true;
    uint32_t value = 0;
    Resolution resolution = kIgnore;
  };

  Result next() {
    if (offset_ >= size_) {
      // We are done.
      return Result{true, 0, Resolution::kInt8};
    }

    if (remaining_) {
      remaining_--;
      const auto this_register = current_register_++;

      // Do we actually have enough data?
      if (offset_ + ResolutionSize(current_resolution_) > size_) {
        return Result{true, 0, Resolution::kInt8};
      }

      return Result{false, this_register, current_resolution_};
    }

    // We need to look for another command.
    while (offset_ < size_) {
      const auto cmd = data_[offset_++];
      if (cmd == Multiplex::kNop) { continue; }

      // We are guaranteed to still need data.
      if (offset_ >= size_) {
        // Nope, we are out.
        break;
      }

      if (cmd >= 0x20 && cmd < 0x30) {
        // This is a regular reply of some sort.
        const auto id = (cmd >> 2) & 0x03;
        current_resolution_ = [id]() {
          switch (id) {
            case 0: return Resolution::kInt8;
            case 1: return Resolution::kInt16;
            case 2: return Resolution::kInt32;
            case 3: return Resolution::kFloat;
          }
          // we cannot reach this point
          return Resolution::kInt8;
        }();
        int count = cmd & 0x03;
        if (count == 0) {
          count = data_[offset_++];

          // We still need more data.
          if (offset_ >= size_) {
            break;
          }
        }

        if (count == 0) {
          // Empty, guess we can ignore.
          continue;
        }

        current_register_ = data_[offset_++];
        remaining_ = count - 1;

        if (offset_ + ResolutionSize(current_resolution_) > size_) {
          return Result{true, 0, Resolution::kInt8};
        }

        return Result{false, current_register_++, current_resolution_};
      }

      // For anything else, we'll just assume it is an error of some
      // sort and stop parsing.
      offset_ = size_;
      break;
    }
    return Result{true, 0, Resolution::kInt8};
  }

  template <typename T>
  T Read() {
    if (offset_ + sizeof(T) > size_) {
      abort();
    }

    T result = {};
    ::memcpy(&result, &data_[offset_], sizeof(T));
    offset_ += sizeof(T);
    return result;
  }

  template <typename T>
  double Nanify(T value) {
    if (value == std::numeric_limits<T>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value;
  }

  double ReadMapped(Resolution res,
                    double int8_scale,
                    double int16_scale,
                    double int32_scale) {
    switch (res) {
      case Resolution::kInt8: {
        return Nanify<int8_t>(Read<int8_t>()) * int8_scale;
      }
      case Resolution::kInt16: {
        return Nanify<int16_t>(Read<int16_t>()) * int16_scale;
      }
      case Resolution::kInt32: {
        return Nanify<int32_t>(Read<int32_t>()) * int32_scale;
      }
      case Resolution::kFloat: {
        return Read<float>();
      }
      default: {
        break;
      }
    }
    abort();
  }

  int ReadInt(Resolution res) {
    return static_cast<int>(ReadMapped(res, 1.0, 1.0, 1.0));
  }

  double ReadPosition(Resolution res) {
    return ReadMapped(res, 0.01, 0.0001, 0.00001);
  }

  double ReadVelocity(Resolution res) {
    return ReadMapped(res, 0.1, 0.00025, 0.00001);
  }

  double ReadTorque(Resolution res) {
    return ReadMapped(res, 0.5, 0.01, 0.001);
  }

  double ReadPwm(Resolution res) {
    return ReadMapped(res, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0);
  }

  double ReadVoltage(Resolution res) {
    return ReadMapped(res, 0.5, 0.1, 0.001);
  }

  double ReadTemperature(Resolution res) {
    return ReadMapped(res, 1.0, 0.1, 0.001);
  }

  double ReadTime(Resolution res) {
    return ReadMapped(res, 0.01, 0.001, 0.000001);
  }

  double ReadCurrent(Resolution res) {
    return ReadMapped(res, 1.0, 0.1, 0.001);
  }

  void Ignore(Resolution res) {
    offset_ += ResolutionSize(res);
  }

 private:
  int ResolutionSize(Resolution res) {
    switch (res) {
      case Resolution::kInt8: return 1;
      case Resolution::kInt16: return 2;
      case Resolution::kInt32: return 4;
      case Resolution::kFloat: return 4;
      default: { break; }
    }
    return 1;
  }

  const uint8_t* const data_;
  const uint8_t size_;
  size_t offset_ = 0;

  int remaining_ = 0;
  Resolution current_resolution_ = Resolution::kIgnore;
  uint32_t current_register_ = 0;
};

}
}
