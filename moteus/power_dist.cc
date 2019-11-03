// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include "mbed.h"

#include "moteus/millisecond_timer.h"
#include "moteus/power_dist_hw.h"

int main(void) {
  // Drop our speed down to nothing, because we don't really need to
  // go fast for this and we might as well save the battery.


  DigitalOut led1(DEBUG_LED1);
  DigitalOut led2(DEBUG_LED2);
  moteus::MillisecondTimer timer;

  bool value = false;
  for (;;) {
    value = !value;
    led1.write(value);
    led2.write(!value);
    timer.wait_ms(500);
  }
}
