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

#include "mjbots/moteus/moteus.h"

#include <boost/test/auto_unit_test.hpp>

#include <string>

using namespace mjbots;

namespace tt = boost::test_tools;

using namespace mjbots;

namespace {
struct RwPipe {
  RwPipe() {
    {
      const int result = ::pipe(read_fds);
      moteus::Fdcanusb::FailIfErrno(result < 0);
    }
    {
      const int result = ::pipe(write_fds);
      moteus::Fdcanusb::FailIfErrno(result < 0);
    }

    test_write = ::fdopen(read_fds[1], "w");
    moteus::Fdcanusb::FailIfErrno(test_write == nullptr);

    test_read = ::fdopen(write_fds[0], "r");
    moteus::Fdcanusb::FailIfErrno(test_read == nullptr);
  }

  ~RwPipe() {
    ::fclose(test_read);
    ::fclose(test_write);
    ::close(read_fds[0]);
    ::close(read_fds[1]);
    ::close(write_fds[0]);
    ::close(write_fds[1]);
  }

  void Write(const std::string& msg) {
    const auto result = ::fwrite(msg.data(), msg.size(), 1, test_write);
    moteus::Fdcanusb::FailIfErrno(result != 1);
    ::fflush(test_write);
  }

  int read_fds[2] = {};
  int write_fds[2] = {};

  FILE* test_read = nullptr;
  FILE* test_write = nullptr;
};
}

BOOST_AUTO_TEST_CASE(FdcanusbConstruct) {
  RwPipe pipe;

  moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1]);
}

namespace {
moteus::Fdcanusb::Options MakeOptions() {
  moteus::Fdcanusb::Options options;
  options.min_ok_wait_ns =   200000000;
  options.min_rcv_wait_ns =  200000000;
  options.rx_extra_wait_ns = 200000000;
  return options;
}
}

BOOST_AUTO_TEST_CASE(FdcanusbBasicSingle) {
  RwPipe pipe;
  moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1], MakeOptions());

  std::optional<int> result_errno;

  const auto completed = [&](int errno_in) {
    result_errno = errno_in;
  };

  moteus::Command command;

  command.destination = 5;
  command.source = 2;
  command.can_prefix = 0x20;
  command.reply_required = true;

  command.arbitration_id = 0x8205;
  command.data[0] = 0x20;
  command.data[1] = 0x21;
  command.data[2] = 0x22;
  command.size = 3;

  std::vector<moteus::Command> replies;

  dut.Cycle(&command, 1, &replies, completed);

  // We should be able to read a command line now.
  {
    char line_buf[4096] = {};
    const char* result = ::fgets(line_buf, sizeof(line_buf), pipe.test_read);
    moteus::Fdcanusb::FailIfErrno(result == nullptr);
    BOOST_TEST(line_buf == "can send 8205 202122\n");
  }

  BOOST_TEST(!result_errno);

  pipe.Write("OK\r\n");
  ::usleep(100000);

  BOOST_TEST(!result_errno);

  pipe.Write("rcv 0502 123456\r\n");
  ::usleep(100000);

  BOOST_TEST(!!result_errno);
  BOOST_TEST(*result_errno == 0);

  BOOST_TEST(replies.size() == 1);
  const auto& r = replies[0];
  BOOST_TEST(r.destination == 2);
  BOOST_TEST(r.source == 5);
  BOOST_TEST(r.arbitration_id == 0x0502);
  BOOST_TEST(r.can_prefix == 0);
  BOOST_TEST(r.size == 3);
  BOOST_TEST(r.data[0] == 0x12);
  BOOST_TEST(r.data[1] == 0x34);
  BOOST_TEST(r.data[2] == 0x56);
}

BOOST_AUTO_TEST_CASE(FdcanusbBasicNoResponse) {
  RwPipe pipe;
  moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1], MakeOptions());

  std::optional<int> result_errno;

  const auto completed = [&](int errno_in) {
    result_errno = errno_in;
  };

  moteus::Command command;
  command.arbitration_id = 0x123;
  command.reply_required = false;
  command.data[0] = 0x45;
  command.data[1] = 0x67;
  command.size = 2;

  std::vector<moteus::Command> replies;

  dut.Cycle(&command, 1, &replies, completed);

  // We should be able to read a command line now.
  {
    char line_buf[4096] = {};
    const char* result = ::fgets(line_buf, sizeof(line_buf), pipe.test_read);
    moteus::Fdcanusb::FailIfErrno(result == nullptr);
    BOOST_TEST(line_buf == "can send 0123 4567\n");
  }

  BOOST_TEST(!result_errno);

  pipe.Write("OK\r\n");
  ::usleep(100000);

  BOOST_TEST(!!result_errno);
  BOOST_TEST(*result_errno == 0);
}

BOOST_AUTO_TEST_CASE(FdcanusbOutputVariants) {
  using Command = moteus::Command;

  auto test = [](const Command& command, const std::string& expected_result) {
    RwPipe pipe;
    moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1], MakeOptions());

    std::optional<int> result_errno;

    const auto completed = [&](int errno_in) {
      result_errno = errno_in;
    };

    std::vector<moteus::Command> replies;

    dut.Cycle(&command, 1, &replies, completed);

    // We should be able to read a command line now.
    {
      char line_buf[4096] = {};
      const char* result = ::fgets(line_buf, sizeof(line_buf), pipe.test_read);
      moteus::Fdcanusb::FailIfErrno(result == nullptr);
      BOOST_TEST(line_buf == expected_result);
    }

    BOOST_TEST(!result_errno);

    pipe.Write("OK\r\n");
  };

  Command c;
  c.arbitration_id = 1;
  c.data[0] = 1;
  c.size = 1;

  c.brs = Command::kForceOn;

  test(c, "can send 0001 01 B\n");

  c.brs = Command::kForceOff;
  test(c, "can send 0001 01 b\n");

  c.brs = Command::kDefault;
  c.fdcan_frame = Command::kForceOff;

  test(c, "can send 0001 01 f\n");

  c.fdcan_frame = Command::kForceOn;
  test(c, "can send 0001 01 F\n");

  c.brs = Command::kForceOn;
  c.arbitration_id = 0x00050001;
  test(c, "can send 50001 01 B F\n");

  for (int i = 0; i < 64; i++) {
    c.data[i] = i + 2;
  }
  c.size = 64;
  test(c, "can send 50001 02030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041 B F\n");
}

BOOST_AUTO_TEST_CASE(FdcanusbInputVariants) {
  using Command = moteus::Command;

  auto test = [](const std::string& rcv_message,
                 auto tester) {
    RwPipe pipe;
    moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1], MakeOptions());

    std::optional<int> result_errno;

    const auto completed = [&](int errno_in) {
      result_errno = errno_in;
    };

    moteus::Command command;

    command.reply_required = true;

    command.arbitration_id = 0x1;
    command.data[0] = 0x20;
    command.size = 1;

    std::vector<moteus::Command> replies;

    dut.Cycle(&command, 1, &replies, completed);

    // We should be able to read a command line now.
    {
      char line_buf[4096] = {};
      const char* result = ::fgets(line_buf, sizeof(line_buf), pipe.test_read);
      moteus::Fdcanusb::FailIfErrno(result == nullptr);
      BOOST_TEST(line_buf == "can send 0001 20\n");
    }

    BOOST_TEST(!result_errno);

    pipe.Write("OK\n" + rcv_message);
    ::usleep(100000);

    BOOST_TEST(!!result_errno);
    BOOST_TEST(*result_errno == 0);
    BOOST_TEST(replies.size() == 1);
    tester(replies[0]);
  };

  test("rcv 1 2345\n",
       [](auto c) {
         BOOST_TEST(c.arbitration_id == 1);
         BOOST_TEST(c.source == 0);
         BOOST_TEST(c.destination == 1);
         BOOST_TEST(c.can_prefix == 0);
         BOOST_TEST(c.size == 2);
         BOOST_TEST(c.data[0] == 0x23);
         BOOST_TEST(c.data[1] == 0x45);
         BOOST_TEST(c.brs == Command::kDefault);
         BOOST_TEST(c.fdcan_frame == Command::kDefault);
       });

  test("rcv 230204 2345 B\n",
       [](auto c) {
         BOOST_TEST(c.arbitration_id == 0x230204);
         BOOST_TEST(c.source == 2);
         BOOST_TEST(c.destination == 4);
         BOOST_TEST(c.can_prefix == 0x23);
         BOOST_TEST(c.size == 2);
         BOOST_TEST(c.data[0] == 0x23);
         BOOST_TEST(c.data[1] == 0x45);
         BOOST_TEST(c.brs == Command::kForceOn);
         BOOST_TEST(c.fdcan_frame == Command::kDefault);
       });

  test("rcv 1 01 F\n",
       [](auto c) {
         BOOST_TEST(c.size == 1);
         BOOST_TEST(c.data[0] == 0x01);
         BOOST_TEST(c.brs == Command::kDefault);
         BOOST_TEST(c.fdcan_frame == Command::kForceOn);
       });

  test("rcv 1 01 B F\n",
       [](auto c) {
         BOOST_TEST(c.brs == Command::kForceOn);
         BOOST_TEST(c.fdcan_frame == Command::kForceOn);
       });

  test("rcv 1 01 b F\n",
       [](auto c) {
         BOOST_TEST(c.brs == Command::kForceOff);
         BOOST_TEST(c.fdcan_frame == Command::kForceOn);
       });

  test("rcv 1 01 b f\n",
       [](auto c) {
         BOOST_TEST(c.brs == Command::kForceOff);
         BOOST_TEST(c.fdcan_frame == Command::kForceOff);
       });

  test("rcv 1 02030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041\n",
       [](auto c) {
         BOOST_TEST(c.size == 64);
         for (int i = 0; i < 64; i++) {
           BOOST_TEST(c.data[i] == i + 2);
         }
       });
}
