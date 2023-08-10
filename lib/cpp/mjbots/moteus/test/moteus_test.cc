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

  moteus::CanFdFrame frame;

  frame.destination = 5;
  frame.source = 2;
  frame.can_prefix = 0x20;
  frame.reply_required = true;

  frame.arbitration_id = 0x8205;
  frame.data[0] = 0x20;
  frame.data[1] = 0x21;
  frame.data[2] = 0x22;
  frame.size = 3;

  std::vector<moteus::CanFdFrame> replies;

  dut.Cycle(&frame, 1, &replies, completed);

  // We should be able to read a frame line now.
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

  moteus::CanFdFrame frame;
  frame.arbitration_id = 0x123;
  frame.reply_required = false;
  frame.data[0] = 0x45;
  frame.data[1] = 0x67;
  frame.size = 2;

  std::vector<moteus::CanFdFrame> replies;

  dut.Cycle(&frame, 1, &replies, completed);

  // We should be able to read a frame line now.
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
  using CanFdFrame = moteus::CanFdFrame;

  auto test = [](const CanFdFrame& frame, const std::string& expected_result) {
    RwPipe pipe;
    moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1], MakeOptions());

    std::optional<int> result_errno;

    const auto completed = [&](int errno_in) {
      result_errno = errno_in;
    };

    std::vector<moteus::CanFdFrame> replies;

    dut.Cycle(&frame, 1, &replies, completed);

    // We should be able to read a frame line now.
    {
      char line_buf[4096] = {};
      const char* result = ::fgets(line_buf, sizeof(line_buf), pipe.test_read);
      moteus::Fdcanusb::FailIfErrno(result == nullptr);
      BOOST_TEST(line_buf == expected_result);
    }

    BOOST_TEST(!result_errno);

    pipe.Write("OK\r\n");
  };

  CanFdFrame c;
  c.arbitration_id = 1;
  c.data[0] = 1;
  c.size = 1;

  c.brs = CanFdFrame::kForceOn;

  test(c, "can send 0001 01 B\n");

  c.brs = CanFdFrame::kForceOff;
  test(c, "can send 0001 01 b\n");

  c.brs = CanFdFrame::kDefault;
  c.fdcan_frame = CanFdFrame::kForceOff;

  test(c, "can send 0001 01 f\n");

  c.fdcan_frame = CanFdFrame::kForceOn;
  test(c, "can send 0001 01 F\n");

  c.brs = CanFdFrame::kForceOn;
  c.arbitration_id = 0x00050001;
  test(c, "can send 50001 01 B F\n");

  for (int i = 0; i < 64; i++) {
    c.data[i] = i + 2;
  }
  c.size = 64;
  test(c, "can send 50001 02030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041 B F\n");
}

BOOST_AUTO_TEST_CASE(FdcanusbInputVariants) {
  using CanFdFrame = moteus::CanFdFrame;

  auto test = [](const std::string& rcv_message,
                 auto tester) {
    RwPipe pipe;
    moteus::Fdcanusb dut(pipe.read_fds[0], pipe.write_fds[1], MakeOptions());

    std::optional<int> result_errno;

    const auto completed = [&](int errno_in) {
      result_errno = errno_in;
    };

    moteus::CanFdFrame frame;

    frame.reply_required = true;

    frame.arbitration_id = 0x1;
    frame.data[0] = 0x20;
    frame.size = 1;

    std::vector<moteus::CanFdFrame> replies;

    dut.Cycle(&frame, 1, &replies, completed);

    // We should be able to read a frame line now.
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
         BOOST_TEST(c.brs == CanFdFrame::kDefault);
         BOOST_TEST(c.fdcan_frame == CanFdFrame::kDefault);
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
         BOOST_TEST(c.brs == CanFdFrame::kForceOn);
         BOOST_TEST(c.fdcan_frame == CanFdFrame::kDefault);
       });

  test("rcv 1 01 F\n",
       [](auto c) {
         BOOST_TEST(c.size == 1);
         BOOST_TEST(c.data[0] == 0x01);
         BOOST_TEST(c.brs == CanFdFrame::kDefault);
         BOOST_TEST(c.fdcan_frame == CanFdFrame::kForceOn);
       });

  test("rcv 1 01 B F\n",
       [](auto c) {
         BOOST_TEST(c.brs == CanFdFrame::kForceOn);
         BOOST_TEST(c.fdcan_frame == CanFdFrame::kForceOn);
       });

  test("rcv 1 01 b F\n",
       [](auto c) {
         BOOST_TEST(c.brs == CanFdFrame::kForceOff);
         BOOST_TEST(c.fdcan_frame == CanFdFrame::kForceOn);
       });

  test("rcv 1 01 b f\n",
       [](auto c) {
         BOOST_TEST(c.brs == CanFdFrame::kForceOff);
         BOOST_TEST(c.fdcan_frame == CanFdFrame::kForceOff);
       });

  test("rcv 1 02030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041\n",
       [](auto c) {
         BOOST_TEST(c.size == 64);
         for (int i = 0; i < 64; i++) {
           BOOST_TEST(c.data[i] == i + 2);
         }
       });
}

namespace {
// This test transport always makes the completion callback in the
// same thread.
class SyncTestTransport : public moteus::Transport {
 public:
  virtual void Cycle(const moteus::CanFdFrame* frames,
                     size_t size,
                     std::vector<moteus::CanFdFrame>* replies,
                     moteus::CompletionCallback completed_callback) {
    sent_frames = std::vector<moteus::CanFdFrame>(frames, frames + size);
    *replies = to_reply_with;

    count++;
    completed_callback(0);
  }

  int count = 0;
  std::vector<moteus::CanFdFrame> sent_frames;
  std::vector<moteus::CanFdFrame> to_reply_with;
};

std::string Hexify(const uint8_t* data, size_t size) {
  char buf[10] = {};
  std::string result;
  for (size_t i = 0; i < size; i++) {
    ::snprintf(buf, sizeof(buf) - 1, "%02x", static_cast<int>(data[i]));
    result += buf;
  }
  return result;
}
}

BOOST_AUTO_TEST_CASE(ControllerBasic) {
  auto impl = std::make_shared<SyncTestTransport>();

  moteus::Controller::Options options;
  options.transport = impl;
  moteus::Controller dut(options);

  {
    impl->sent_frames.clear();

    const auto maybe_reply = dut.SetStop();
    BOOST_TEST(impl->count == 1);
    BOOST_TEST(impl->sent_frames.size() == 1);

    // We did not queue a response.
    BOOST_TEST(!maybe_reply);

    const auto c = impl->sent_frames[0];
    BOOST_TEST(c.source == 0);
    BOOST_TEST(c.destination == 1);
    BOOST_TEST(c.can_prefix == 0);
    BOOST_TEST(Hexify(c.data, c.size) == "01000011001f01130d");
  }

  {
    impl->sent_frames.clear();

    const auto maybe_reply = dut.SetBrake();
    BOOST_TEST(impl->count == 2);
    BOOST_TEST(impl->sent_frames.size() == 1);

    // We did not queue a response.
    BOOST_TEST(!maybe_reply);

    const auto c = impl->sent_frames[0];
    BOOST_TEST(c.source == 0);
    BOOST_TEST(c.destination == 1);
    BOOST_TEST(c.can_prefix == 0);
    BOOST_TEST(Hexify(c.data, c.size) == "01000f11001f01130d");
  }

  {
    impl->sent_frames.clear();

    impl->to_reply_with.resize(1);
    auto& c = impl->to_reply_with[0];
    c.destination = 0;
    c.source = 1;
    c.arbitration_id = 0x100;
    c.data[0] = 0x27;
    c.data[1] = 0x00;
    c.data[2] = 0x0a;
    c.data[3] = 0x00;
    c.data[4] = 0x20;
    c.data[5] = 0x30;
    c.data[6] = 0x40;
    c.data[7] = 0x50;
    c.size = 8;

    moteus::PositionMode::Command cmd;
    cmd.position = 1.0;
    cmd.velocity = 2.0;

    const auto maybe_reply = dut.SetPosition(cmd);
    BOOST_TEST(impl->count == 3);

    BOOST_TEST(impl->sent_frames.size() == 1);
    BOOST_TEST(!!maybe_reply);
    const auto& r = *maybe_reply;
    BOOST_TEST(r.frame.arbitration_id == 0x100);
    BOOST_TEST(r.frame.size == 8);

    BOOST_TEST(static_cast<int>(r.values.mode) == 10);
    BOOST_TEST(r.values.position == 1.232);
    BOOST_TEST(r.values.velocity == 5.136);

    BOOST_TEST(Hexify(impl->sent_frames[0].data,
                      impl->sent_frames[0].size) ==
               "01000a0e200000803f0000004011001f01130d");
  }

  {
    impl->sent_frames.clear();
    impl->to_reply_with.clear();

    moteus::VFOCMode::Command cmd;
    cmd.theta_rad = 1.0;
    cmd.voltage = 2.0;

    const auto maybe_reply = dut.SetVFOC(cmd);
    BOOST_TEST(!maybe_reply);

    BOOST_TEST(impl->count == 4);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "0100070e1883f9a23e000000400d1e0000000011001f01130d");
  }

  {
    impl->sent_frames.clear();
    impl->to_reply_with.clear();

    moteus::CurrentMode::Command cmd;
    cmd.q_A = 3.0;
    cmd.d_A = 2.0;

    const auto maybe_reply = dut.SetCurrent(cmd);

    BOOST_TEST(!maybe_reply);

    BOOST_TEST(impl->count == 5);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "0100090e1c000040400000004011001f01130d");
  }

  {
    impl->sent_frames.clear();
    impl->to_reply_with.clear();

    moteus::StayWithinMode::Command cmd;
    cmd.lower_bound = 1.2;
    cmd.upper_bound = 2.0;

    const auto maybe_reply = dut.SetStayWithin(cmd);

    BOOST_TEST(!maybe_reply);

    BOOST_TEST(impl->count == 6);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "01000d0e409a99993f0000004011001f01130d");
  }

  {
    impl->sent_frames.clear();

    impl->to_reply_with.resize(1);
    auto& c = impl->to_reply_with[0];
    c.destination = 0;
    c.source = 1;
    c.arbitration_id = 0x100;
    c.data[0] = 0x26;
    c.data[1] = 0x00;
    c.data[2] = 0x0a;
    c.data[3] = 0x00;
    c.data[4] = 0x20;
    c.data[5] = 0x30;
    c.size = 6;

    const auto maybe_reply = dut.Query();
    BOOST_TEST(!!maybe_reply);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& k = impl->sent_frames[0];
    BOOST_TEST(Hexify(k.data, k.size) == "11001f01130d");

    const auto& r = *maybe_reply;
    BOOST_TEST(r.frame.arbitration_id == 0x100);
    BOOST_TEST(r.frame.size == 6);

    BOOST_TEST(static_cast<int>(r.values.mode) == 10);
    BOOST_TEST(r.values.position == 1.232);
  }
}

namespace {
class AsyncTestTransport : public moteus::Transport {
 public:
  virtual void Cycle(const moteus::CanFdFrame* frames,
                     size_t size,
                     std::vector<moteus::CanFdFrame>* replies,
                     moteus::CompletionCallback completed_callback) {
    sent_frames = std::vector<moteus::CanFdFrame>(frames, frames + size);
    to_reply = replies;
    to_callback = completed_callback;
  }

  std::vector<moteus::CanFdFrame> sent_frames;

  std::vector<moteus::CanFdFrame>* to_reply = nullptr;
  moteus::CompletionCallback to_callback;
};

struct TestCallback {
  void operator()(int value_in) {
    called = true;
    value = value_in;
  }

  bool called = false;
  int value = -1;
};
}

BOOST_AUTO_TEST_CASE(ControllerAsyncBasic) {
  auto impl = std::make_shared<AsyncTestTransport>();
  TestCallback cbk;
  auto cbk_wrap = [&](int v) { cbk(v); };
  moteus::Controller::Result result;

  auto start_test = [&]() {
    impl->sent_frames = {};
    impl->to_reply = nullptr;
    impl->to_callback = {};
    cbk.called = false;
    cbk.value = -1;
  };

  auto check_test = [&](const std::string& expected) {
    BOOST_TEST(!cbk.called);
    BOOST_TEST(impl->to_reply != nullptr);

    impl->to_reply->resize(1);
    auto& c = impl->to_reply->at(0);
    c.destination = 0;
    c.source = 1;
    c.arbitration_id = 0x100;
    c.data[0] = 0x27;
    c.data[1] = 0x00;
    c.data[2] = 0x0a;
    c.data[3] = 0x00;
    c.data[4] = 0x20;
    c.data[5] = 0x30;
    c.data[6] = 0x40;
    c.data[7] = 0x50;
    c.size = 8;

    impl->to_callback(0);

    BOOST_TEST(cbk.called);
    BOOST_TEST(cbk.value == 0);
    BOOST_TEST(result.frame.size == 8);
    BOOST_TEST(result.values.position == 1.232);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& r = impl->sent_frames[0];
    BOOST_TEST(Hexify(r.data, r.size) == expected);
  };

  moteus::Controller::Options options;
  options.transport = impl;
  moteus::Controller dut(options);

  start_test();
  dut.AsyncQuery(&result, cbk_wrap);
  check_test("11001f01130d");

  start_test();
  dut.AsyncStop(&result, cbk_wrap);
  check_test("01000011001f01130d");

  start_test();
  dut.AsyncBrake(&result, cbk_wrap);
  check_test("01000f11001f01130d");

  {
    start_test();
    moteus::PositionMode::Command cmd;
    cmd.position = 4.0;
    dut.AsyncPosition(cmd, &result, cbk_wrap);
    check_test("01000a0e20000080400000000011001f01130d");
  }

  {
    start_test();
    moteus::VFOCMode::Command cmd;
    cmd.theta_rad = 2.0;
    dut.AsyncVFOC(cmd, &result, cbk_wrap);
    check_test("0100070e1883f9223f000000000d1e0000000011001f01130d");
  }

  {
    start_test();
    moteus::CurrentMode::Command cmd;
    cmd.q_A = 5.0;
    dut.AsyncCurrent(cmd, &result, cbk_wrap);
    check_test("0100090e1c0000a0400000000011001f01130d");
  }

  {
    start_test();
    moteus::StayWithinMode::Command cmd;
    cmd.upper_bound = 10.0;
    cmd.lower_bound = -3.0;
    dut.AsyncStayWithin(cmd, &result, cbk_wrap);
    check_test("01000d0e40000040c00000204111001f01130d");
  }
}
