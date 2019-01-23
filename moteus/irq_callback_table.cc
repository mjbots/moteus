// Copyright 2018 Josh Pieper, jjp@pobox.com.
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

#include "moteus/irq_callback_table.h"

namespace moteus {
namespace {

void g_handle(int);

extern "C" {
void g_handle0() { g_handle(0); }
void g_handle1() { g_handle(1); }
void g_handle2() { g_handle(2); }
void g_handle3() { g_handle(3); }
void g_handle4() { g_handle(4); }
void g_handle5() { g_handle(5); }
void g_handle6() { g_handle(6); }
void g_handle7() { g_handle(7); }
void g_handle8() { g_handle(8); }
void g_handle9() { g_handle(9); }
}

IrqCallbackTable::IrqFunction g_handlers[] = {
  g_handle0,
  g_handle1,
  g_handle2,
  g_handle3,
  g_handle4,
  g_handle5,
  g_handle6,
  g_handle7,
  g_handle8,
  g_handle9,
};

std::array<mjlib::micro::StaticFunction<void()>, 10> g_callbacks;

void g_handle(int slot) {
  g_callbacks[slot]();
}
}  // namespace

IrqCallbackTable::Callback::~Callback() {
  if (irq_function == nullptr) { return; }

  for (size_t i = 0; i < g_callbacks.size(); i++) {
    if (g_handlers[i] == irq_function) {
      g_callbacks[i] = {};
      break;
    }
  }
}

IrqCallbackTable::Callback IrqCallbackTable::MakeFunction(
    mjlib::micro::StaticFunction<void()> callback) {
  // Find an empty entry in the callback list.
  for (size_t i = 0; i < g_callbacks.size(); i++) {
    auto& entry = g_callbacks[i];
    if (!entry.valid()) {
      entry = callback;
      return g_handlers[i];
    }
  }

  MJ_ASSERT(false);
  return Callback();
}

}
