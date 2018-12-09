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

#include <atomic>
#include <array>

#include "mjlib/micro/static_function.h"

namespace moteus {

template <size_t Size>
class AtomicEventQueue {
 public:
  // This may be called from any thread context.
  void Queue(const mjlib::micro::StaticFunction<void()>& function) {
    for (auto& entry : entries_) {
      const int current = entry.state.load();
      if (current == 0) {
        // We will try to claim this.
        const int old = entry.state.fetch_add(1);
        if (old != 0) {
          // Someone else got to it first.
          continue;
        }

        entry.function = function;

        const int should_be_1 = entry.state.fetch_add(1);
        MJ_ASSERT(should_be_1 == 1);
        return;
      }
    }

    // We must have been all full.
    MJ_ASSERT(false);
  }

  // Run any queued events.  This may be called from any thread
  // context.
  void Poll() {
    for (auto& entry: entries_) {
      const int current = entry.state.load();
      if (current == 2) {
        const int old = entry.state.fetch_add(1);
        if (old != 2) {
          // Someone else got to it first.
          continue;
        }

        auto copy = entry.function;
        entry.function = {};

        const int should_be_3 = entry.state.fetch_and(0);
        MJ_ASSERT(should_be_3 == 3);

        copy();
      }
    }
  }

 private:
  struct Entry {
    std::atomic<int> state = 0;
    mjlib::micro::StaticFunction<void()> function;
  };
  std::array<Entry, Size> entries_;
};

}
