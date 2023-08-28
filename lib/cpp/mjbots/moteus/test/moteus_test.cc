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
    ::usleep(500000);

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
class PostTransport : public moteus::Transport {
 public:
  virtual void Post(std::function<void()> callback) {
    queue.push_back(std::move(callback));
  }

  void ProcessQueue() {
    while (!queue.empty()) {
      auto top = queue.front();
      queue.pop_front();
      top();
    }
  }

  std::deque<std::function<void()>> queue;
};

// This test transport always makes the completion callback in the
// same thread.
class SyncTestTransport : public PostTransport {
 public:
  virtual void Cycle(const moteus::CanFdFrame* frames,
                     size_t size,
                     std::vector<moteus::CanFdFrame>* replies,
                     moteus::CompletionCallback completed_callback) {
    std::copy(frames, frames + size,
              std::back_inserter(sent_frames));
    if (replies) {
      *replies = to_reply_with;
    }

    count++;

    Post(std::bind(completed_callback, 0));
    ProcessQueue();
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
    BOOST_TEST(c.reply_required == true);
    BOOST_TEST(c.expected_reply_size == 22);
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
    BOOST_TEST(c.reply_required == true);
    BOOST_TEST(c.expected_reply_size == 22);
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
    impl->to_reply_with.clear();

    moteus::OutputNearest::Command cmd;
    cmd.position = 2.5;
    const auto maybe_reply = dut.SetOutputNearest(cmd);
    BOOST_TEST(!maybe_reply);
    BOOST_REQUIRE(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "0db0020000204011001f01130d");
  }

  {
    impl->sent_frames.clear();
    impl->to_reply_with.clear();

    moteus::OutputExact::Command cmd;
    cmd.position = 2.5;
    const auto maybe_reply = dut.SetOutputExact(cmd);
    BOOST_TEST(!maybe_reply);
    BOOST_REQUIRE(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "0db1020000204011001f01130d");
  }

  {
    impl->sent_frames.clear();
    impl->to_reply_with.clear();

    const auto maybe_reply = dut.SetRequireReindex({});
    BOOST_TEST(!maybe_reply);
    BOOST_REQUIRE(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "01b2020111001f01130d");
  }

  {
    impl->sent_frames.clear();
    impl->to_reply_with.clear();

    moteus::ClockTrim::Command cmd;
    cmd.trim = 4;
    const auto maybe_reply = dut.SetClockTrim(cmd);
    BOOST_TEST(!maybe_reply);
    BOOST_REQUIRE(impl->sent_frames.size() == 1);
    const auto& c = impl->sent_frames[0];
    BOOST_TEST(Hexify(c.data, c.size) == "09710400000011001f01130d");
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

    const auto maybe_reply = dut.SetQuery();
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
class AsyncTestTransport : public PostTransport {
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

  void ProcessQueue() {
    while (to_callback || !queue.empty()) {
      if (to_callback) {
        auto copy = to_callback;
        to_callback = {};
        copy(0);
      }
      if (!queue.empty()) {
        auto top = queue.front();
        queue.pop_front();
        top();
      }
    }
  }
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
    impl->queue = {};
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

    impl->ProcessQueue();

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

  {
    start_test();
    moteus::OutputNearest::Command cmd;
    cmd.position = 2.5;
    dut.AsyncOutputNearest(cmd, &result, cbk_wrap);
    check_test("0db0020000204011001f01130d");
  }

  {
    start_test();
    moteus::OutputExact::Command cmd;
    cmd.position = 1.5;
    dut.AsyncOutputExact(cmd, &result, cbk_wrap);
    check_test("0db1020000c03f11001f01130d");
  }

  {
    start_test();
    dut.AsyncRequireReindex({}, &result, cbk_wrap);
    check_test("01b2020111001f01130d");
  }

  {
    start_test();
    moteus::ClockTrim::Command cmd;
    cmd.trim = 3;
    dut.AsyncClockTrim(cmd, &result, cbk_wrap);
    check_test("09710300000011001f01130d");
  }
}

BOOST_AUTO_TEST_CASE(ControllerSchemaVersion) {
  auto test = [](int offset) {
    auto impl = std::make_shared<SyncTestTransport>();

    moteus::Controller::Options options;
    options.transport = impl;
    moteus::Controller dut(options);

    impl->to_reply_with.resize(1);
    auto& f = impl->to_reply_with[0];
    f.destination = 0;
    f.source = 1;
    f.arbitration_id = 0x100;
    f.data[0] = 0x29;  // reply with 1 int32_t
    f.data[1] = 0x82;  // register varuint = 0x102
    f.data[2] = 0x02;
    f.data[3] = moteus::kCurrentRegisterMapVersion + offset;
    f.data[4] = 0;
    f.data[5] = 0;
    f.data[6] = 0;
    f.size = 7;

    dut.VerifySchemaVersion();
    BOOST_TEST(impl->count == 1);
    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& s = impl->sent_frames[0];
    BOOST_TEST(Hexify(s.data, s.size) == "19820211001f01130d");
  };

  test(0);
  auto verify_exception = [](const std::runtime_error& re) {
    BOOST_TEST(re.what() ==
               "Register map version mismatch device is 6 but library requires 5");
    return true;
  };
  BOOST_CHECK_EXCEPTION(test(1), std::runtime_error, verify_exception);
}

namespace {
bool StartsWith(const std::string& haystack, const std::string& needle) {
  return haystack.substr(0, needle.size()) == needle;
}

// This pretends to be a diagnostic channel server.
class DiagnosticTestTransport : public PostTransport {
 public:
  virtual void Cycle(const moteus::CanFdFrame* frames,
                     size_t size,
                     std::vector<moteus::CanFdFrame>* replies,
                     moteus::CompletionCallback completed_callback) {
    for (size_t i = 0; i < size; i++) {
      ProcessFrame(frames[i], replies);
    }

    Post(std::bind(completed_callback, 0));
    ProcessQueue();
  }

  void ProcessFrame(const moteus::CanFdFrame& frame,
                    std::vector<moteus::CanFdFrame>* replies) {
    // Any frame we care about will be at least 3 bytes long.
    if (frame.size < 3) { return; }

    if (frame.data[0] == moteus::kClientToServer) {
      if ((frame.arbitration_id & 0x8000) != 0) {
        throw std::logic_error(
            "we dont currently handle writes combined with poll");
      }
      const int channel = frame.data[1];
      const int size = frame.data[2];
      if (size + 3 > frame.size) {
        throw std::logic_error("malformed frame");
      }

      // We only look at channel 1.
      if (channel != 1) { return; }

      client_to_server +=
          std::string(reinterpret_cast<const char*>(&frame.data[3]), size);

      ProcessClientToServer();
    } else if (frame.data[0] == moteus::kClientPollServer) {
      const int channel = frame.data[1];
      const int max_size = frame.data[2];
      if (channel != 1) { return; }

      if ((frame.arbitration_id & 0x8000) == 0) { return; }

      // We need to generate a response.
      moteus::CanFdFrame response;
      response.source = frame.destination;
      response.destination = frame.source;
      response.arbitration_id = (response.source << 8) | response.destination;
      response.data[0] = moteus::kServerToClient;
      response.data[1] = channel;

      // We arbitrarily limit our responses to 5 bytes to force the
      // client to assemble multiple frames together.
      const auto to_write = std::min<size_t>(
          5, std::min<size_t>(server_to_client.size(), max_size));
      response.data[2] = to_write;
      std::memcpy(&response.data[3], server_to_client.data(), to_write);
      server_to_client = server_to_client.substr(to_write);
      response.size = 3 + to_write;

      replies->push_back(response);
    }
  }

  void ProcessClientToServer() {
    const auto maybe_newline = client_to_server.find_first_of("\r\n");
    if (maybe_newline == std::string::npos) { return; }

    const auto line = client_to_server.substr(0, maybe_newline);
    client_to_server = client_to_server.substr(maybe_newline + 1);
    ProcessClientToServerLine(line);
  }

  void ProcessClientToServerLine(const std::string& line) {
    if (StartsWith(line, "conf get ")) {
      // We'll reply with all conf gets in the same way.
      server_to_client += "4.0000\r\n";
    } else if (StartsWith(line, "conf set ")) {
      server_to_client += "OK\r\n";
    } else if (line == "conf enumerate") {
      server_to_client += "id.id 0\r\n";
      server_to_client += "stuff.bar 1\r\n";
      server_to_client += "bing.baz 234\r\n";
      server_to_client += "OK\r\n";
    } else {
      throw std::logic_error("unhandled diagnostic cmd: " + line);
    }
  }

  std::string client_to_server;
  std::string server_to_client;

};
}

BOOST_AUTO_TEST_CASE(ControllerDiagnosticTest) {
  auto transport = std::make_shared<DiagnosticTestTransport>();
  moteus::Controller::Options options;
  options.transport = transport;
  moteus::Controller dut(options);

  {
    const auto result = dut.DiagnosticCommand("conf set id.id 5");
    BOOST_TEST(result == "");
  }

  {
    const auto result = dut.DiagnosticCommand(
        "conf get servo.pid_position.kp", moteus::Controller::kExpectSingleLine);
    BOOST_TEST(result == "4.0000");
  }

  {
    const auto result = dut.DiagnosticCommand("conf enumerate");
    BOOST_TEST(result == "id.id 0\r\nstuff.bar 1\r\nbing.baz 234\r\n");
  }
}

BOOST_AUTO_TEST_CASE(ControllerAsyncDiagnosticTest) {
  auto transport = std::make_shared<DiagnosticTestTransport>();
  moteus::Controller::Options options;
  options.transport = transport;
  moteus::Controller dut(options);

  {
    TestCallback cbk;
    auto cbk_wrap = [&](int v) { cbk(v); };

    std::string result;
    dut.AsyncDiagnosticCommand("conf set id.id 5", &result, cbk_wrap);
    BOOST_TEST(cbk.called);
    BOOST_TEST(result == "");
  }

  {
    TestCallback cbk;
    auto cbk_wrap = [&](int v) { cbk(v); };

    std::string result;
    dut.AsyncDiagnosticCommand(
        "conf get servo.pid_position.kp", &result, cbk_wrap,
        moteus::Controller::kExpectSingleLine);
    BOOST_TEST(cbk.called);
    BOOST_TEST(result == "4.0000");
  }

  {
    TestCallback cbk;
    auto cbk_wrap = [&](int v) { cbk(v); };

    std::string result;
    dut.AsyncDiagnosticCommand("conf enumerate", &result, cbk_wrap);
    BOOST_TEST(cbk.called);
    BOOST_TEST(result == "id.id 0\r\nstuff.bar 1\r\nbing.baz 234\r\n");
  }
}

BOOST_AUTO_TEST_CASE(ControllerDiagnosticWrite) {
  auto transport = std::make_shared<SyncTestTransport>();
  moteus::Controller::Options options;
  options.transport = transport;
  moteus::Controller dut(options);

  const std::string message = "long test of stuff that will keep going for a very long time and span multiple CAN frames";
  dut.DiagnosticWrite(message, 2);

  BOOST_TEST(transport->sent_frames.size() == 2);
  const auto& f1 = transport->sent_frames[0];
  BOOST_TEST(f1.source == 0);
  BOOST_TEST(f1.destination == 1);
  BOOST_TEST(f1.data[0] == 0x40);
  BOOST_TEST(f1.data[1] == 2);
  BOOST_TEST(f1.data[2] == 48);
  BOOST_TEST(std::string(reinterpret_cast<const char*>(&f1.data[3]), 48)
             == message.substr(0, 48));

  const auto& f2 = transport->sent_frames[1];
  BOOST_TEST(f2.source == 0);
  BOOST_TEST(f2.destination == 1);
  BOOST_TEST(f2.data[0] == 0x40);
  BOOST_TEST(f2.data[1] == 2);
  BOOST_TEST(f2.data[2] == 41);
  BOOST_TEST(std::string(reinterpret_cast<const char*>(&f2.data[3]), 41)
             == message.substr(48));
}

BOOST_AUTO_TEST_CASE(ControllerDiagnosticRead) {
  auto transport = std::make_shared<SyncTestTransport>();
  moteus::Controller::Options options;
  options.transport = transport;
  moteus::Controller dut(options);

  transport->to_reply_with.resize(1);
  auto& f = transport->to_reply_with[0];
  f.source = 1;
  f.destination = 0;
  f.arbitration_id = 0x100;
  f.data[0] = 0x41;
  f.data[1] = 0x02;
  f.data[2] = 0x04;
  f.data[3] = 't';
  f.data[4] = 'e';
  f.data[5] = 's';
  f.data[6] = 't';
  f.size = 7;

  const auto result = dut.DiagnosticRead(2);

  BOOST_TEST(result == "test");
}

namespace {
class PositionWaitTestTransport : public PostTransport {
 public:
  virtual void Cycle(const moteus::CanFdFrame* frames,
                     size_t size,
                     std::vector<moteus::CanFdFrame>* replies,
                     moteus::CompletionCallback completed_callback) {
    // Report trajectory complete on the 5th frame.
    sent_frames.clear();
    std::copy(frames, frames + size,
              std::back_inserter(sent_frames));
    BOOST_REQUIRE(size > 0);
    const auto& f = frames[0];

    count++;

    moteus::CanFdFrame qr;
    qr.source = f.destination;
    qr.destination = f.source;
    qr.can_prefix = f.can_prefix;
    qr.arbitration_id = (qr.source << 8) | qr.destination;
    qr.data[0] = 0x23;
    qr.data[1] = 0x00;
    qr.data[2] = 0x0a;
    qr.data[3] = 0x20;
    qr.data[4] = 0x30;
    qr.data[5] = 0x21;
    qr.data[6] = 0x0b;
    qr.data[7] = count >= 5 ? 1 : 0;
    qr.size = 8;
    replies->push_back(qr);

    Post(std::bind(completed_callback, 0));
    ProcessQueue();
  }

  int count = 0;
  std::vector<moteus::CanFdFrame> sent_frames;
};
}

BOOST_AUTO_TEST_CASE(ControllerPositionWait) {
  auto impl = std::make_shared<PositionWaitTestTransport>();

  moteus::Controller::Options options;
  options.transport = impl;
  moteus::Controller dut(options);

  moteus::PositionMode::Command cmd;
  cmd.position = 2.0;
  auto maybe_result = dut.SetPositionWaitComplete(cmd, 0.01);
  BOOST_TEST(!!maybe_result);
  BOOST_TEST(impl->count == 5);
  BOOST_TEST(maybe_result->values.position == 0.32);
}

BOOST_AUTO_TEST_CASE(ControllerNoQuery) {
  auto impl = std::make_shared<SyncTestTransport>();

  moteus::Controller::Options options;
  options.default_query = false;
  options.transport = impl;
  moteus::Controller dut(options);

  {
    dut.SetStop();

    BOOST_TEST(impl->count == 1);
    BOOST_REQUIRE(impl->sent_frames.size() == 1);
    const auto& f = impl->sent_frames[0];
    BOOST_TEST(Hexify(f.data, f.size) == "010000");
    BOOST_TEST(f.reply_required == false);
    BOOST_TEST(f.expected_reply_size == 0);
  }

  // Set up our common reply for the next tests.
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

  {
    impl->sent_frames.clear();

    const auto maybe_reply = dut.SetQuery();
    BOOST_TEST(!!maybe_reply);
    const auto& r = *maybe_reply;
    BOOST_TEST(r.frame.arbitration_id == 0x100);
    BOOST_TEST(r.frame.size == 8);
    BOOST_TEST(static_cast<int>(r.values.mode) == 10);
    BOOST_TEST(r.values.position == 1.232);
    BOOST_TEST(r.values.velocity == 5.136);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& f = impl->sent_frames[0];
    BOOST_TEST(Hexify(f.data, f.size) == "11001f01130d");
    BOOST_TEST(f.arbitration_id == 0x8001);
    BOOST_TEST(f.reply_required == true);
    BOOST_TEST(f.expected_reply_size == 22);
  }

  {
    impl->sent_frames.clear();

    moteus::Query::Format query_override;
    query_override.trajectory_complete = moteus::kInt8;

    const auto maybe_reply = dut.SetQuery(&query_override);
    BOOST_TEST(!!maybe_reply);
    const auto& r = *maybe_reply;
    BOOST_TEST(r.frame.arbitration_id == 0x100);
    BOOST_TEST(r.frame.size == 8);
    BOOST_TEST(static_cast<int>(r.values.mode) == 10);
    BOOST_TEST(r.values.position == 1.232);
    BOOST_TEST(r.values.velocity == 5.136);

    BOOST_TEST(impl->sent_frames.size() == 1);
    const auto& f = impl->sent_frames[0];
    BOOST_TEST(Hexify(f.data, f.size) == "11001f01110b130d");
    BOOST_TEST(f.arbitration_id == 0x8001);
    BOOST_TEST(f.reply_required == true);
    BOOST_TEST(f.expected_reply_size == 25);
  }
}
