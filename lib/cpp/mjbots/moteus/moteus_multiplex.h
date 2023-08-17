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

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

namespace mjbots {
namespace moteus {

/// Each value can be sent or received as one of the following.
enum Resolution : int8_t {
  kInt8 = 0,
  kInt16 = 1,
  kInt32 = 2,
  kFloat = 3,
  kIgnore,
};

/// A vocabulary type for the basic data in a CAN-FD frame.
struct CanData {
  uint8_t data[64] = {};
  uint8_t size = 0;
};

enum Multiplex : uint16_t {
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

namespace detail {
template <typename T>
class numeric_limits {
 public:
};

template <>
class numeric_limits<int8_t> {
 public:
  static int8_t max() { return 127; }
  static int8_t min() { return -128; }
};

template <>
class numeric_limits<int16_t> {
 public:
  static int16_t max() { return 32767; }
  static int16_t min() { return -32768; }
};

template <>
class numeric_limits<int32_t> {
 public:
  static int32_t max() { return 2147483647; }
  static int32_t min() { return -2147483648; }
};

template <>
class numeric_limits<double> {
 public:
};

template <typename T>
T max(T lhs, T rhs) {
  return (lhs >= rhs) ? lhs : rhs;
}

template <typename T>
T min(T lhs, T rhs) {
  return (lhs <= rhs) ? lhs : rhs;
}

}

namespace {
template <typename T>
T Saturate(double value, double scale) {
  // TODO: Implement without numeric_limits
  if (!::isfinite(value)) {
    return detail::numeric_limits<T>::min();
  }

  const double scaled = value / scale;
  const auto max = detail::numeric_limits<T>::max();

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
class WriteCanData {
 public:
  WriteCanData(CanData* frame) : data_(&frame->data[0]), size_(&frame->size) {}
  WriteCanData(uint8_t* data, uint8_t* size) : data_(data), size_(size) {}

  uint8_t size() const { return *size_; }

  template <typename T, typename X>
  void Write(X value_in) {
#ifndef __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures supported"
#endif

    T value = static_cast<T>(value_in);
    if (sizeof(value) + *size_ > 64) {
      abort();
    }

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

  void Write(const char* data, uint16_t size) {
    if ((size + *size_) > 64) {
      abort();
    }
    ::memcpy(&data_[*size_], data, size);
    *size_ += size;
  }

  void WriteInt(int32_t value, Resolution res) {
    switch (res) {
      case Resolution::kInt8: {
        Write<int8_t>(detail::max<int32_t>(
                          -127, detail::min<int32_t>(127, value)));
        break;
      }
      case Resolution::kInt16: {
        Write<int16_t>(detail::max<int32_t>(
                           -32767, detail::min<int32_t>(32767, value)));
        break;
      }
      case Resolution::kInt32: {
        Write<int32_t>(value);
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

/// Read typed values from a CAN frame.
class MultiplexParser {
 public:
  MultiplexParser(const CanData* frame)
      : data_(&frame->data[0]),
        size_(frame->size) {}
  MultiplexParser(const uint8_t* data, uint8_t size)
      : data_(data),
        size_(size) {}

  uint16_t ReadVaruint() {
    uint16_t result = 0;
    uint16_t shift = 0;

    for (int8_t i = 0; i < 5; i++) {
      if (remaining() == 0) { return result; }

      const auto this_byte = static_cast<uint8_t>(Read<int8_t>());
      result |= (this_byte & 0x7f) << shift;
      shift += 7;

      if ((this_byte & 0x80) == 0) {
        return result;
      }
    }
    return result;
  }

  struct Result {
    bool done = true;
    uint16_t value = 0;
    Resolution resolution = kIgnore;

    Result(bool done_in, uint16_t value_in, Resolution resolution_in)
        : done(done_in), value(value_in), resolution(resolution_in) {}

    Result() {}
  };

  Result next() {
    if (offset_ >= size_) {
      // We are done.
      return Result(true, 0, Resolution::kInt8);
    }

    if (remaining_) {
      remaining_--;
      const auto this_register = current_register_++;

      // Do we actually have enough data?
      if (offset_ + ResolutionSize(current_resolution_) > size_) {
        return Result(true, 0, Resolution::kInt8);
      }

      return Result(false, this_register, current_resolution_);
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
        int8_t count = cmd & 0x03;
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

        current_register_ = ReadVaruint();
        remaining_ = count - 1;

        if (offset_ + ResolutionSize(current_resolution_) > size_) {
          return Result(true, 0, Resolution::kInt8);
        }

        return Result(false, current_register_++, current_resolution_);
      }

      // For anything else, we'll just assume it is an error of some
      // sort and stop parsing.
      offset_ = size_;
      break;
    }
    return Result(true, 0, Resolution::kInt8);
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
    if (value == detail::numeric_limits<T>::min()) {
      return NaN;
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

  static constexpr int8_t kInt = 0;
  static constexpr int8_t kPosition = 1;
  static constexpr int8_t kVelocity = 2;
  static constexpr int8_t kTorque = 3;
  static constexpr int8_t kPwm = 4;
  static constexpr int8_t kVoltage = 5;
  static constexpr int8_t kTemperature = 6;
  static constexpr int8_t kTime = 7;
  static constexpr int8_t kCurrent = 8;
  static constexpr int8_t kTheta = 9;

  double ReadConcrete(Resolution res, int8_t concrete_type) {
#ifndef ARDUINO
    static constexpr double kMappingValues[] = {
#else
    static constexpr double PROGMEM kMappingValues[] = {
#endif
      1.0, 1.0, 1.0,           // kInt
      0.01, 0.0001, 0.00001,   // kPosition
      0.1, 0.00025, 0.00001,   // kVelocity
      0.5, 0.01, 0.001,        // kTorque
      1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0,  // kPwm
      0.5, 0.1, 0.001,         // kVoltage
      1.0, 0.1, 0.001,         // kTemperature
      0.01, 0.001, 0.000001,   // kTime
      1.0, 0.1, 0.001,         // kCurrent
      1.0 / 127.0 * M_PI, 1.0 / 32767.0 * M_PI, 1.0 / 2147483647.0 * M_PI, // kTheta
    };

#ifndef ARDUINO
    const double int8_scale = kMappingValues[concrete_type * 3 + 0];
    const double int16_scale = kMappingValues[concrete_type * 3 + 1];
    const double int32_scale = kMappingValues[concrete_type * 3 + 2];
#else
    const double int8_scale = pgm_read_float_near(kMappingValues + concrete_type * 3 + 0);
    const double int16_scale = pgm_read_float_near(kMappingValues + concrete_type * 3 + 1);
    const double int32_scale = pgm_read_float_near(kMappingValues + concrete_type * 3 + 2);
#endif

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
    return static_cast<int>(ReadConcrete(res, kInt));
  }

  double ReadPosition(Resolution res) {
    return ReadConcrete(res, kPosition);
  }

  double ReadVelocity(Resolution res) {
    return ReadConcrete(res, kVelocity);
  }

  double ReadTorque(Resolution res) {
    return ReadConcrete(res, kTorque);
  }

  double ReadPwm(Resolution res) {
    return ReadConcrete(res, kPwm);
  }

  double ReadVoltage(Resolution res) {
    return ReadConcrete(res, kVoltage);
  }

  double ReadTemperature(Resolution res) {
    return ReadConcrete(res, kTemperature);
  }

  double ReadTime(Resolution res) {
    return ReadConcrete(res, kTime);
  }

  double ReadCurrent(Resolution res) {
    return ReadConcrete(res, kCurrent);
  }

  void Ignore(Resolution res) {
    offset_ += ResolutionSize(res);
  }

  void ReadRaw(uint8_t* output, uint16_t size) {
    if ((offset_ + size) > size_) { ::abort(); }
    ::memcpy(output, &data_[offset_], size);
    offset_ += size;
  }

  uint16_t remaining() const {
    return size_ - offset_;
  }

  static int8_t ResolutionSize(Resolution res) {
    switch (res) {
      case Resolution::kInt8: return 1;
      case Resolution::kInt16: return 2;
      case Resolution::kInt32: return 4;
      case Resolution::kFloat: return 4;
      default: { break; }
    }
    return 1;
  }

 private:
  const uint8_t* const data_;
  const uint8_t size_;
  uint16_t offset_ = 0;

  int8_t remaining_ = 0;
  Resolution current_resolution_ = Resolution::kIgnore;
  uint16_t current_register_ = 0;
};

/// Determines how to group registers when encoding them to minimize
/// the required bytes.
class WriteCombiner {
 public:
  WriteCombiner(WriteCanData* frame,
                int8_t base_command,
                uint16_t start_register,
                const Resolution* resolutions,
                uint16_t resolutions_size)
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

  uint8_t reply_size() const { return reply_size_; }

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

    int16_t count = 1;
    for (uint16_t i = this_offset + 1;
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

    const auto start_size = frame_->size();
    if (count <= 3) {
      // Use the shorthand formulation.
      frame_->Write<int8_t>(write_command + count);
    } else {
      // Nope, the long form.
      frame_->Write<int8_t>(write_command);
      frame_->Write<int8_t>(count);
    }
    frame_->WriteVaruint(start_register_ + this_offset);
    const auto end_size = frame_->size();

    reply_size_ += (end_size - start_size);
    reply_size_ += count * MultiplexParser::ResolutionSize(new_resolution);

    return true;
  }

 private:
  WriteCanData* const frame_;
  int8_t base_command_ = 0;
  uint16_t start_register_ = 0;
  const Resolution* const resolutions_;
  uint16_t resolutions_size_ = 0;

  Resolution current_resolution_ = Resolution::kIgnore;
  uint16_t offset_ = 0;
  uint8_t reply_size_ = 0;
};

}
}
