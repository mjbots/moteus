// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "EventQueue.h"
#include "PinNames.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"

namespace moteus {

/// Presents a single USART on the STM32F446 as an AsyncStream.
class Stm32F446AsyncUart : public mjlib::micro::AsyncStream {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;
    int baud_rate = 115200;
  };

  /// @param event_queue - All callbacks will be invoked from this, it
  /// is aliased internally and must live as long as the instance.
  Stm32F446AsyncUart(mjlib::micro::Pool* pool,
                     events::EventQueue* event_queue,
                     const Options&);
  ~Stm32F446AsyncUart() override;

  void AsyncReadSome(const mjlib::base::string_span&,
                     const mjlib::micro::SizeCallback&) override;
  void AsyncWriteSome(const std::string_view&,
                      const mjlib::micro::SizeCallback&) override;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
