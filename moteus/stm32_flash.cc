// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.
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

#include "moteus/stm32_flash.h"

#include "mjlib/base/assert.h"

#if defined(TARGET_STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#elif defined(TARGET_STM32G4)
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#else
#error "Unknown target"
#endif

namespace moteus {

namespace {
char* const kFlashSector1 = (char*) 0x08004000;
constexpr size_t kSectorSize = 16384;
}

Stm32Flash::Stm32Flash() {
}

Stm32Flash::~Stm32Flash() {}

Stm32Flash::Info Stm32Flash::GetInfo() {
  Info result;
  result.start = kFlashSector1;
  result.end = result.start + kSectorSize;
  return result;
}

void Stm32Flash::Erase() {
#if defined(TARGET_STM32F4)
  uint32_t error = 0;

  FLASH_EraseInitTypeDef erase;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = 0;
  erase.Sector = FLASH_SECTOR_1;
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7 to 3.6

  HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&erase, &error);

  MJ_ASSERT(result == 0);
  MJ_ASSERT(error == 0xffffffff);
#elif defined(TARGET_STM32G4)
  MJ_ASSERT(false);
#else
#error "Unknown target"
#endif
}

void Stm32Flash::Unlock() {
#if defined(TARGET_STM32F4)
  FLASH->SR |= FLASH_SR_PGSERR;
  FLASH->SR |= FLASH_SR_PGPERR;
  HAL_FLASH_Unlock();
#elif defined(TARGET_STM32G4)
  MJ_ASSERT(false);
#else
#error "Unknown target"
#endif
}

void Stm32Flash::Lock() {
  HAL_FLASH_Lock();
}

void Stm32Flash::ProgramByte(char* ptr, uint8_t value) {
#if defined(TARGET_STM32F4)
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                    reinterpret_cast<uint32_t>(ptr),
                    value);
#elif defined(TARGET_STM32G4)
  MJ_ASSERT(false);
#else
#error "Unknown target"
#endif
}

}
