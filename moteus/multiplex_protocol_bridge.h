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

#pragma once

#include <functional>

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/error_code.h"
#include "mjlib/micro/multiplex_protocol.h"
#include "mjlib/micro/telemetry_manager.h"

#include "moteus/stream_writer.h"

/// Bridge a master multiplex protocol bus to one or more slaves.
namespace moteus {
template <size_t Size>
class Bridge {
 public:
  struct Slave {
    Bridge* parent = nullptr;
    mjlib::micro::AsyncStream* const stream;
    StreamWriter<256> writer{stream};
    char buffer[256] = {};

    Slave(mjlib::micro::AsyncStream* stream_in)
        : stream(stream_in) {}
  };

  template <typename ... Args>
  Bridge(mjlib::micro::TelemetryManager* telemetry_manager,
         mjlib::micro::MultiplexProtocolServer* master,
         Args&&... slaves)
      : master_(master),
        slaves_{{slaves...}} {
    for (auto& item : slaves_) {
      item.parent = this;
    }
    telemetry_manager->Register("bridge", &data_);
  }

  void Start() {
    StartMasterRead();
    for (auto& slave : slaves_) {
      StartSlaveRead(&slave);
    }
  }

 private:
  void StartMasterRead() {
    master_->AsyncReadUnknown(
        master_buffer_, std::bind(&Bridge::HandleMasterRead, this,
                                  std::placeholders::_1, std::placeholders::_2));
  }

  void HandleMasterRead(const mjlib::micro::error_code& ec, size_t size) {
    MJ_ASSERT(!ec);

    data_.master.rx++;

    // Write this out to each of our slaves.
    for (auto& slave : slaves_) {
      slave.writer.AsyncWrite(
          std::string_view(master_buffer_, size),
          std::bind(&Bridge::StaticHandleSlaveWrite,
                    std::placeholders::_1, &slave));
    }

    StartMasterRead();
  }

  static void StaticHandleSlaveWrite(const mjlib::micro::error_code& ec,
                                     Slave* slave) {
    MJ_ASSERT(!ec);
    slave->parent->HandleSlaveWrite(slave);
  }

  void HandleSlaveWrite(Slave* slave) {
    const size_t slave_index = slave - &slaves_[0];
    data_.slaves[slave_index].tx++;
  }

  void StartSlaveRead(Slave* slave) {
    slave->stream->AsyncReadSome(
        slave->buffer,
        std::bind(
            &Bridge::StaticHandleSlaveRead,
            std::placeholders::_1, std::placeholders::_2,
            slave));
  }

  static void StaticHandleSlaveRead(
      const mjlib::micro::error_code& ec, size_t size,
      Slave* slave) {
    slave->parent->HandleSlaveRead(ec, size, slave);
  }

  void HandleSlaveRead(const mjlib::micro::error_code& ec, size_t size,
                       Slave* slave) {
    MJ_ASSERT(!ec);

    const size_t slave_index = slave - &slaves_[0];
    data_.slaves[slave_index].rx++;

    master_writer_.AsyncWrite(
        std::string_view(slave->buffer, size),
        std::bind(&Bridge::StaticHandleMasterWrite,
                  std::placeholders::_1, slave));
  }

  static void StaticHandleMasterWrite(
      const mjlib::micro::error_code& ec, Slave* slave) {
    MJ_ASSERT(!ec);

    slave->parent->data_.master.tx++;
    slave->parent->StartSlaveRead(slave);
  }


  mjlib::micro::MultiplexProtocolServer* const master_;
  char master_buffer_[256] = {};
  StreamWriter<256> master_writer_{master_->raw_write_stream()};

  std::array<Slave, Size> slaves_;

  struct EndpointStats {
    int32_t tx = 0;
    int32_t rx = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(tx));
      a->Visit(MJ_NVP(rx));
    }
  };

  struct Data {
    EndpointStats master;
    std::array<EndpointStats, Size> slaves;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(master));
      a->Visit(MJ_NVP(slaves));
    }
  };

  Data data_;
};
}
