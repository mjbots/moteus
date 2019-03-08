// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include <inttypes.h>

#include <functional>

#include "mbed.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/multiplex_protocol.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "moteus/imu_junction_hw.h"
#include "moteus/millisecond_timer.h"
#include "moteus/multiplex_protocol_bridge.h"
#include "moteus/stm32f446_async_uart.h"
#include "moteus/stm32_flash.h"
#include "moteus/system_info.h"

using namespace moteus;
namespace micro = mjlib::micro;

class Debug {
 public:
  Debug(MillisecondTimer* timer, micro::TelemetryManager* telemetry_manager)
      : timer_(timer) {
    telemetry_manager->Register("imu_dbg", &data_);
    spi_.format(8);
    spi_.frequency(5000000);
  }

  void Start() {
    // First switch the accelerometer into SPI mode.

    auto write_accel = [&](uint8_t reg, uint8_t val) {
      imu_accel_cs_.write(0);
      spi_.write(reg);
      spi_.write(val);
      imu_accel_cs_.write(1);
      timer_->wait_us(10);
    };

    write_accel(0x00, 0x00);  // dummy to switch to SPI mode
    write_accel(0x7d, 0x04);  // ACC_PWR_CTRL - on
    write_accel(0x53, 0x08);  // INT1_IO_CONF - INT1 as output
    write_accel(0x58, 0x04);  // INT1_INT2_DATA_MAP - drd -> int1

    auto write_gyro = [&](uint8_t reg, uint8_t val) {
      imu_gyro_cs_.write(0);
      spi_.write(reg);
      spi_.write(val);
      imu_gyro_cs_.write(1);
      timer_->wait_us(10);
    };

    write_gyro(0x15, 0x80);  // GYRO_INT_CTRL - enable interrupts
    write_gyro(0x16, 0x00);  // push pull active high
    write_gyro(0x18, 0x01);  // data ready mapped to int3
  }

  void PollMillisecond() {
    count_++;
    if (count_ % 10 != 0) { return; }

    data_.accel_int = imu_accel_int_.read();
    data_.gyro_int = imu_gyro_int_.read();

    imu_accel_cs_.write(0);
    uint8_t buf[6] = {};
    spi_.write(0x80 | 0x12);  // ACC_X_LSB
    // Read the dummy byte.
    spi_.write(0x00);
    for (auto& value : buf) {
      value = static_cast<uint8_t>(spi_.write(0));
    }
    imu_accel_cs_.write(1);

    data_.accelx = static_cast<int16_t>(buf[1] * 256 + buf[0]);
    data_.accely = static_cast<int16_t>(buf[3] * 256 + buf[2]);
    data_.accelz = static_cast<int16_t>(buf[5] * 256 + buf[4]);

    imu_gyro_cs_.write(0);
    spi_.write(0x80 | 0x02);  // RATE_X_LSB
    for (auto& value : buf) {
      value = static_cast<uint8_t>(spi_.write(0));
    }
    imu_gyro_cs_.write(1);

    data_.gyrox = static_cast<int16_t>(buf[1] * 256 + buf[0]);
    data_.gyroy = static_cast<int16_t>(buf[3] * 256 + buf[2]);
    data_.gyroz = static_cast<int16_t>(buf[5] * 256 + buf[4]);
  }

  struct Data {
    int16_t accelx = 0;
    int16_t accely = 0;
    int16_t accelz = 0;

    int16_t gyrox = 0;
    int16_t gyroy = 0;
    int16_t gyroz = 0;

    uint8_t accel_int = 0;
    uint8_t gyro_int = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(accelx));
      a->Visit(MJ_NVP(accely));
      a->Visit(MJ_NVP(accelz));
      a->Visit(MJ_NVP(gyrox));
      a->Visit(MJ_NVP(gyroy));
      a->Visit(MJ_NVP(gyroz));
      a->Visit(MJ_NVP(accel_int));
      a->Visit(MJ_NVP(gyro_int));
    }
  };

  MillisecondTimer* const timer_;
  SPI spi_{IMU_MOSI, IMU_MISO, IMU_SCK};
  DigitalOut imu_accel_cs_{IMU_ACCEL_CS, 1};
  DigitalOut imu_gyro_cs_{IMU_GYRO_CS, 1};
  DigitalIn imu_accel_int_{IMU_ACCEL_INT, PullNone};
  DigitalIn imu_gyro_int_{IMU_GYRO_INT, PullNone};

  uint16_t count_ = 0;
  Data data_;
};

int main(void) {
  MillisecondTimer timer;

  micro::SizedPool<12288> pool;

  Stm32F446AsyncUart rs485(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PC_10;
      options.rx = PC_11;
      options.dir = PC_12;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
      options.baud_rate = 3000000;
      options.rx_buffer_size = 256;
      return options;
    }());

  Stm32F446AsyncUart slave1(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PA_2;
      options.rx = PA_3;
      options.dir = PA_1;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
      options.baud_rate = 3000000;
      options.rx_buffer_size = 256;
      return options;
    }());

  Stm32F446AsyncUart slave2(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PC_6;
      options.rx = PC_7;
      options.dir = PC_8;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
      options.baud_rate = 3000000;
      options.rx_buffer_size = 256;
      return options;
    }());

  micro::MultiplexProtocolServer multiplex_protocol(&pool, &rs485, nullptr, []() {
      micro::MultiplexProtocolServer::Options options;
      options.default_id = 64;
      return options;
    }());

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  SystemInfo system_info(pool, telemetry_manager);

  Bridge<2> bridge(&telemetry_manager, &multiplex_protocol, &slave1, &slave2);
  Debug debug(&timer, &telemetry_manager);

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  persistent_config.Load();

  command_manager.AsyncStart();
  multiplex_protocol.Start();
  bridge.Start();
  debug.Start();

  auto old_time = timer.read_ms();

  for (;;) {
    rs485.Poll();
    slave1.Poll();
    slave2.Poll();

    const auto new_time = timer.read_ms();

    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();
      system_info.PollMillisecond();
      debug.PollMillisecond();

      old_time = new_time;
    }
    SystemInfo::idle_count++;
  }

  return 0;
}
