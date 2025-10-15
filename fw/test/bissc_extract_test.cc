// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "fw/bissc_extract.h"

#include <boost/test/auto_unit_test.hpp>

#include <array>
#include <ostream>
#include <vector>

namespace moteus {
std::ostream& operator<<(std::ostream& os, BisscExtractError e) {
  switch (e) {
    case BisscExtractError::kNone: return os << "kNone";
    case BisscExtractError::kStartBitNotFound: return os << "kStartBitNotFound";
    case BisscExtractError::kBufferTooSmall: return os << "kBufferTooSmall";
    case BisscExtractError::kCrcMismatch: return os << "kCrcMismatch";
  }
  return os << "Unknown(" << static_cast<int>(e) << ")";
}
}

using namespace moteus;

namespace tt = boost::test_tools;

// Helper to create a buffer with specific bit pattern
// pin_bit_pos determines which bit position in each byte holds the data
std::vector<uint8_t> MakeBuffer(const std::vector<int>& bits, uint8_t pin_bit_pos) {
  std::vector<uint8_t> buffer;
  buffer.reserve(bits.size());
  for (int bit : bits) {
    buffer.push_back(bit ? (1 << pin_bit_pos) : 0);
  }
  return buffer;
}

// Reference bitwise CRC-6 implementation for verification
uint32_t ReferenceCrc6(uint64_t data, uint8_t data_bits) {
  constexpr uint32_t polynomial = 0x43;
  uint32_t crc = 0;
  for (int i = data_bits - 1; i >= 0; i--) {
    const bool bit = (data >> i) & 1;
    const bool msb = (crc >> 5) & 1;
    crc <<= 1;
    if (bit ^ msb) {
      crc ^= polynomial;
    }
  }
  return crc & 0x3F;
}

BOOST_AUTO_TEST_CASE(BisscCrcBasic) {
  // Test CRC-6 computation with known values
  // For CRC-6 with polynomial 0x43 (x^6 + x + 1)

  // A simple test: all zeros should give CRC of 0
  BOOST_TEST(ComputeBisscCRC(0, 8, 6) == 0);

  // Test with a known value
  const uint32_t crc1 = ComputeBisscCRC(0x1234, 16, 6);
  BOOST_TEST(crc1 == 56);

  // And our reference implementation should match.
  const uint32_t crc2 = ReferenceCrc6(0x1234, 16);
  BOOST_TEST(crc1 == crc2);
}

BOOST_AUTO_TEST_CASE(BisscCrcTableMatchesBitwise) {
  // Verify that the table-based CRC matches the reference bitwise
  // implementation.

  // Test all bit lengths from 1 to 24 (covering typical encoder ranges)
  for (uint8_t data_bits = 1; data_bits <= 24; data_bits++) {
    // Test with several data patterns
    for (uint64_t pattern : {0ULL, 1ULL, 0x5555555555555555ULL,
                              0xAAAAAAAAAAAAAAAAULL, 0xFFFFFFFFFFFFFFFFULL}) {
      const uint64_t mask = (1ULL << data_bits) - 1;
      const uint64_t data = pattern & mask;

      const uint32_t table_crc = ComputeBisscCRC(data, data_bits, 6);
      const uint32_t ref_crc = ReferenceCrc6(data, data_bits);

      BOOST_TEST(table_crc == ref_crc);
    }
  }
}

BOOST_AUTO_TEST_CASE(BisscCrcLookupTableValues) {
  // Verify table values match bitwise computation
  for (uint8_t i = 0; i < 64; i++) {
    BOOST_TEST(kBisscCrc6Table[i] == ReferenceCrc6(i, 6));
  }
}

BOOST_AUTO_TEST_CASE(BisscStartBitDetection) {
  // Test START bit detection
  // Frame: HIGH, HIGH, LOW, LOW, HIGH (START), data...
  //        idle  idle  ACK  ACK  START

  const uint8_t pin_pos = 3;
  auto buffer = MakeBuffer({1, 1, 0, 0, 1, 1, 0, 1}, pin_pos);

  const int start_idx = detail::FindStartBit(
      buffer.data(), buffer.size(), pin_pos, 8);

  BOOST_TEST(start_idx == 4);  // START bit at index 4
}

BOOST_AUTO_TEST_CASE(BisscStartBitNotFound) {
  // Test when no START bit is found (all LOW after entering ACK)
  const uint8_t pin_pos = 0;
  auto buffer = MakeBuffer({1, 1, 0, 0, 0, 0, 0, 0}, pin_pos);

  const int start_idx = detail::FindStartBit(
      buffer.data(), buffer.size(), pin_pos, 8);

  BOOST_TEST(start_idx == -1);
}

BOOST_AUTO_TEST_CASE(BisscStartBitNoAck) {
  // Test when no ACK is seen (all HIGH)
  const uint8_t pin_pos = 0;
  auto buffer = MakeBuffer({1, 1, 1, 1, 1, 1, 1, 1}, pin_pos);

  const int start_idx = detail::FindStartBit(
      buffer.data(), buffer.size(), pin_pos, 8);

  BOOST_TEST(start_idx == -1);
}

BOOST_AUTO_TEST_CASE(BisscMultiplyTrickAllPatterns) {
  // Verify the multiplication trick produces correct MSB-first ordering
  // for all 16 possible 4-bit patterns

  for (uint32_t pattern = 0; pattern < 16; pattern++) {
    // Create 4 samples with bits from pattern (MSB-first)
    const int b0 = (pattern >> 3) & 1;  // MSB
    const int b1 = (pattern >> 2) & 1;
    const int b2 = (pattern >> 1) & 1;
    const int b3 = (pattern >> 0) & 1;  // LSB

    // Build normalized value (bits at positions 0, 8, 16, 24)
    const uint32_t normalized = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);

    // Apply multiplication trick
    const uint32_t result = (normalized * kBisscBitCollectMSB) >> 24;

    // Result should match original pattern
    BOOST_TEST(result == pattern);
  }
}

BOOST_AUTO_TEST_CASE(BisscMultiplyTrickDifferentPinPositions) {
  // Test multiplication trick with different pin positions

  for (uint8_t pin_pos = 0; pin_pos < 8; pin_pos++) {
    // Test pattern 0b1010 (10 decimal)
    auto buffer = MakeBuffer({1, 0, 1, 0}, pin_pos);

    // Align buffer for 32-bit access
    alignas(4) std::array<uint8_t, 8> aligned_buf = {};
    std::copy(buffer.begin(), buffer.end(), aligned_buf.begin());

    const uint64_t result = detail::ExtractDataBits(
        aligned_buf.data(), 0, 4, pin_pos);

    BOOST_TEST(result == 0b1010);
  }
}

BOOST_AUTO_TEST_CASE(BisscDataExtractionSimple) {
  // Test data extraction with a known pattern
  const uint8_t pin_pos = 2;

  // Create a buffer with pattern: 1, 0, 1, 1, 0, 0, 1, 0 (0xB2 = 178)
  auto buffer = MakeBuffer({1, 0, 1, 1, 0, 0, 1, 0}, pin_pos);

  const uint64_t result = detail::ExtractDataBits(
      buffer.data(), 0, 8, pin_pos);

  BOOST_TEST(result == 0b10110010);
}

BOOST_AUTO_TEST_CASE(BisscDataExtraction13Bits) {
  // Test 13-bit extraction (common encoder resolution)
  const uint8_t pin_pos = 5;

  // Create pattern: 1_0011_0101_1010 = 0x135A
  std::vector<int> bits = {1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0};
  auto buffer = MakeBuffer(bits, pin_pos);

  const uint64_t result = detail::ExtractDataBits(
      buffer.data(), 0, 13, pin_pos);

  BOOST_TEST(result == 0x135A);
}

BOOST_AUTO_TEST_CASE(BisscDataExtractionNonZeroStart) {
  // Test extraction with non-zero start index
  const uint8_t pin_pos = 2;

  // Buffer: [prefix junk] [actual data: 10110010]
  // Prefix is 5 bytes of alternating pattern that should be skipped
  std::vector<int> bits = {1, 0, 1, 0, 1,  // prefix (indices 0-4)
                           1, 0, 1, 1, 0, 0, 1, 0};  // data at index 5
  auto buffer = MakeBuffer(bits, pin_pos);

  const uint64_t result = detail::ExtractDataBits(
      buffer.data(), 5, 8, pin_pos);

  BOOST_TEST(result == 0b10110010);
}

BOOST_AUTO_TEST_CASE(BisscDataExtractionBufferExtendsBeyond) {
  // Test that extraction stops at data_bits even if buffer is longer
  const uint8_t pin_pos = 0;

  // Buffer has 16 bytes but we only extract 8 bits from the middle
  // [4 bytes prefix] [8 bits data] [4 bytes suffix that should be ignored]
  std::vector<int> bits = {0, 0, 0, 0,              // prefix (indices 0-3)
                           1, 1, 0, 0, 1, 0, 1, 0,  // data = 0xCA (indices 4-11)
                           1, 1, 1, 1};             // suffix (indices 12-15)
  auto buffer = MakeBuffer(bits, pin_pos);

  const uint64_t result = detail::ExtractDataBits(
      buffer.data(), 4, 8, pin_pos);

  // Should get 0b11001010 = 0xCA, not affected by suffix
  BOOST_TEST(result == 0xCA);
}

BOOST_AUTO_TEST_CASE(BisscDataExtractionNonAlignedStart) {
  // Test extraction starting at a non-4-byte-aligned index
  const uint8_t pin_pos = 3;

  // Start at index 7 (not aligned to 4)
  std::vector<int> bits = {0, 0, 0, 0, 0, 0, 0,    // prefix (indices 0-6)
                           1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0};  // 12 bits data
  auto buffer = MakeBuffer(bits, pin_pos);

  const uint64_t result = detail::ExtractDataBits(
      buffer.data(), 7, 12, pin_pos);

  // Data: 1010_1110_1010 = 0xAEA
  BOOST_TEST(result == 0xAEA);
}

BOOST_AUTO_TEST_CASE(BisscCrcBitsExtraction) {
  // Test CRC bit extraction (inverted)
  const uint8_t pin_pos = 0;

  // CRC bits are inverted: signal LOW = bit 1
  // Buffer with signal: HIGH, LOW, HIGH, LOW, LOW, HIGH
  // Should give CRC:     0,    1,   0,    1,   1,   0 = 0b010110 = 22
  auto buffer = MakeBuffer({1, 0, 1, 0, 0, 1}, pin_pos);

  const uint32_t crc = detail::ExtractCrcBits(
      buffer.data(), 0, 6, pin_pos);

  BOOST_TEST(crc == 0b010110);
}

BOOST_AUTO_TEST_CASE(BisscFullFrameExtraction) {
  // Test complete frame extraction
  const uint8_t pin_pos = 0;

  // Frame structure:
  // [idle: 1,1] [ACK: 0,0] [START: 1] [CDS: 0] [data: 10110010] [E: 1] [W: 1] [CRC inverted]
  //
  // Data = 178 = 0b10110010
  // Error/Warning signals are HIGH (1), meaning no error/warning (flags = false)
  // CRC input = (178 << 2) | (1 << 1) | 1 = 715
  // CRC-6 of 715 = 58 = 0b111010
  // Inverted for transmission: 0b000101 -> bits {0, 0, 0, 1, 0, 1}
  constexpr uint32_t expected_crc = 58;

  auto buffer = MakeBuffer({1, 1,                       // idle
                            0, 0,                       // ACK
                            1,                          // START
                            0,                          // CDS
                            1, 0, 1, 1, 0, 0, 1, 0,     // data = 178
                            1, 1,                       // error, warning (HIGH = no flags)
                            0, 0, 0, 1, 0, 1}, pin_pos);  // inverted CRC

  BisscExtractConfig config;
  config.pin_bit_pos = pin_pos;
  config.data_bits = 8;
  config.crc_bits = 6;

  const auto result = ExtractBisscFrame(buffer.data(), buffer.size(), config);

  BOOST_TEST(result.error == BisscExtractError::kNone);
  BOOST_TEST(result.start_bit_index == 4);
  BOOST_TEST(result.data_value == 178);
  BOOST_TEST(result.error_flag == false);    // HIGH signal = no error
  BOOST_TEST(result.warning_flag == false);  // HIGH signal = no warning
  BOOST_TEST(result.crc_received == expected_crc);
  BOOST_TEST(result.crc_computed == expected_crc);
}

BOOST_AUTO_TEST_CASE(BisscBufferTooSmall) {
  // Test error handling when buffer is too small
  const uint8_t pin_pos = 0;

  // Minimal frame that will fail due to small buffer
  auto buffer = MakeBuffer({1, 0, 1}, pin_pos);  // Only 3 bytes

  BisscExtractConfig config;
  config.pin_bit_pos = pin_pos;
  config.data_bits = 8;
  config.crc_bits = 6;

  const auto result = ExtractBisscFrame(buffer.data(), buffer.size(), config);

  // Should fail - either no START found or buffer too small
  BOOST_TEST(result.error != BisscExtractError::kNone);
}

BOOST_AUTO_TEST_CASE(BisscNoStartBit) {
  // Test error when START bit is not found
  const uint8_t pin_pos = 0;

  // All LOW after initial HIGH - no START transition
  auto buffer = MakeBuffer({1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, pin_pos);

  BisscExtractConfig config;
  config.pin_bit_pos = pin_pos;
  config.data_bits = 8;
  config.crc_bits = 6;

  const auto result = ExtractBisscFrame(buffer.data(), buffer.size(), config);

  BOOST_TEST(result.error == BisscExtractError::kStartBitNotFound);
}

BOOST_AUTO_TEST_CASE(BisscCrcMismatch) {
  // Test CRC mismatch detection
  const uint8_t pin_pos = 0;

  // Frame with intentionally wrong CRC (all HIGH = inverted zeros)
  // Data = 0b10101010 = 170
  auto buffer = MakeBuffer({1, 1,                       // idle
                            0, 0,                       // ACK
                            1,                          // START
                            0,                          // CDS
                            1, 0, 1, 0, 1, 0, 1, 0,     // data = 170
                            1, 1,                       // error, warning (HIGH = no flags)
                            1, 1, 1, 1, 1, 1}, pin_pos);  // wrong CRC (all 1s = inverted 0)

  BisscExtractConfig config;
  config.pin_bit_pos = pin_pos;
  config.data_bits = 8;
  config.crc_bits = 6;

  const auto result = ExtractBisscFrame(buffer.data(), buffer.size(), config);

  BOOST_TEST(result.error == BisscExtractError::kCrcMismatch);
  BOOST_TEST(result.crc_received != result.crc_computed);
}

BOOST_AUTO_TEST_CASE(BisscErrorFlagSet) {
  // Test when error flag is set (LOW signal)
  const uint8_t pin_pos = 0;

  // Data = 0, Error = LOW (flag set), Warning = HIGH (no flag)
  // CRC input = (0 << 2) | (0 << 1) | 1 = 1
  // CRC-6 of 1 = 3 = 0b000011
  // Inverted for transmission: 0b111100 -> {1, 1, 1, 1, 0, 0}
  auto buffer = MakeBuffer({1, 1,                       // idle
                            0, 0,                       // ACK
                            1,                          // START
                            0,                          // CDS
                            0, 0, 0, 0, 0, 0, 0, 0,     // data = 0
                            0,                          // error (LOW = flag set)
                            1,                          // warning (HIGH = no flag)
                            1, 1, 1, 1, 0, 0}, pin_pos);  // inverted CRC = 3

  BisscExtractConfig config;
  config.pin_bit_pos = pin_pos;
  config.data_bits = 8;
  config.crc_bits = 6;

  const auto result = ExtractBisscFrame(buffer.data(), buffer.size(), config);

  BOOST_TEST(result.error == BisscExtractError::kNone);
  BOOST_TEST(result.error_flag == true);   // LOW signal = error present
  BOOST_TEST(result.warning_flag == false);
}
