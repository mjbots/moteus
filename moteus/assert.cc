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

#include "mbed_assert.h"
#include "mbed.h"

#include "hal/gpio_api.h"

#include "moteus/hw.h"

namespace mjlib {
namespace base {

void assertion_failed(const char* expression, const char* filename, int line) {
  mbed_assert_internal(expression, filename, line);
}

}
}

extern "C" {
void mbed_die(void) {
  // We want to ensure the motor controller is disabled and flash an
  // LED which exists.
  gpio_t power;
  gpio_init_out(&power, DRV8323_HIZ);
  gpio_write(&power, 0);

  // Also, disable the DRV8323 entirely, because, hey, why not.
  gpio_t enable;
  gpio_init_out(&enable, DRV8323_ENABLE);
  gpio_write(&enable, 0);

  gpio_t led;
  gpio_init_out(&led, DEBUG_LED1);

  // Now flash an actual LED.
  for (;;) {
    gpio_write(&led, 0);
    wait_ms(200);
    gpio_write(&led, 1);
    wait_ms(200);
  }
}
}
