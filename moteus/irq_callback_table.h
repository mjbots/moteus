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

#include "mjlib/micro/static_function.h"

namespace moteus {

/// A helper class to bind arbitrary callbacks into IRQ handlers.
class IrqCallbackTable {
 public:
  using IrqFunction = void (*)();

  /// A RAII class that deregisters the callback when destroyed.  A
  /// default constructed instance is valid and has a null
  /// irq_function.
  class Callback {
   public:
    Callback(IrqFunction in = nullptr) : irq_function(in) {}
    ~Callback();

    Callback(const Callback&) = delete;
    Callback& operator=(const Callback&) = delete;

    Callback(Callback&& rhs) : irq_function(rhs.irq_function) {
      rhs.irq_function = nullptr;
    }

    Callback& operator=(Callback&& rhs) {
      irq_function = rhs.irq_function;
      rhs.irq_function = nullptr;
      return *this;
    }

    IrqFunction irq_function = nullptr;
  };

  /// Given an arbitrary callback, return a function pointer suitable
  /// for use as an interrupt handler.  When invoked, the given
  /// callback will be called.
  static Callback MakeFunction(mjlib::micro::StaticFunction<void()> callback);
};

}
