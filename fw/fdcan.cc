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

#include "fw/fdcan.h"

#include "PeripheralPins.h"

extern const PinMap PinMap_CAN_TD[];
extern const PinMap PinMap_CAN_RD[];

namespace moteus {
namespace {
constexpr uint32_t RoundUpDlc(size_t size) {
  if (size == 0) { return FDCAN_DLC_BYTES_0; }
  if (size == 1) { return FDCAN_DLC_BYTES_1; }
  if (size == 2) { return FDCAN_DLC_BYTES_2; }
  if (size == 3) { return FDCAN_DLC_BYTES_3; }
  if (size == 4) { return FDCAN_DLC_BYTES_4; }
  if (size == 5) { return FDCAN_DLC_BYTES_5; }
  if (size == 6) { return FDCAN_DLC_BYTES_6; }
  if (size == 7) { return FDCAN_DLC_BYTES_7; }
  if (size == 8) { return FDCAN_DLC_BYTES_8; }
  if (size <= 12) { return FDCAN_DLC_BYTES_12; }
  if (size <= 16) { return FDCAN_DLC_BYTES_16; }
  if (size <= 20) { return FDCAN_DLC_BYTES_20; }
  if (size <= 24) { return FDCAN_DLC_BYTES_24; }
  if (size <= 32) { return FDCAN_DLC_BYTES_32; }
  if (size <= 48) { return FDCAN_DLC_BYTES_48; }
  if (size <= 64) { return FDCAN_DLC_BYTES_64; }
  return 0;
}

FDCan::Rate MakeTime(int bitrate, int max_time_seg1, int max_time_seg2) {
  FDCan::Rate result;

  result.prescaler = 1;

  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t total_divisor = 0;

  while (true) {
    total_divisor = (pclk1 / result.prescaler) / bitrate;

    // One of the divisor counts comes for free.
    const auto actual_divisor = total_divisor - 1;

    // Split up the remainder roughly 3/1
    result.time_seg2 = actual_divisor / 3;
    result.time_seg1 = actual_divisor - result.time_seg2;

    result.sync_jump_width = std::min(16, result.time_seg2);

    if (result.time_seg1 > max_time_seg1 ||
        result.time_seg2 > max_time_seg2) {
      result.prescaler++;
      continue;
    }

    break;
  }

  return result;
}

FDCan::Rate ApplyRateOverride(FDCan::Rate base, FDCan::Rate overlay) {
  if (overlay.prescaler >= 0) {
    base.prescaler = overlay.prescaler;
  }
  if (overlay.sync_jump_width >= 0) {
    base.sync_jump_width = overlay.sync_jump_width;
  }
  if (overlay.time_seg1 >= 0) {
    base.time_seg1 = overlay.time_seg1;
  }
  if (overlay.time_seg2 >= 0) {
    base.time_seg2 = overlay.time_seg2;
  }
  return base;
}
}

FDCan::FDCan(const Options& options)
    : options_(options) {
  Init();
}

void FDCan::ConfigureFilters(const FilterConfig& filters) {
  options_.filters = filters;
  Init();
}

void FDCan::Init() {
  const auto& options = options_;

  __HAL_RCC_FDCAN_CLK_ENABLE();

  {
    const auto can_td = pinmap_peripheral(options.td, PinMap_CAN_TD);
    const auto can_rd = pinmap_peripheral(options.rd, PinMap_CAN_RD);
    can_ = reinterpret_cast<FDCAN_GlobalTypeDef*>(
        pinmap_merge(can_td, can_rd));
  }

  pinmap_pinout(options.td, PinMap_CAN_TD);
  pinmap_pinout(options.rd, PinMap_CAN_RD);

  auto& can = hfdcan1_;

  can.Instance = can_;
  can.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  can.Init.FrameFormat = [&]() {
    if (options.fdcan_frame && options.bitrate_switch) {
      return FDCAN_FRAME_FD_BRS;
    } else if (options.fdcan_frame) {
      return FDCAN_FRAME_FD_NO_BRS;
    }
    return FDCAN_FRAME_CLASSIC;
  }();
  can.Init.Mode = [&]() {
    if (options.bus_monitor) {
      return FDCAN_MODE_BUS_MONITORING;
    } else if (options.restricted_mode) {
      return FDCAN_MODE_RESTRICTED_OPERATION;
    }
    return FDCAN_MODE_NORMAL;
  }();
  can.Init.AutoRetransmission =
      options.automatic_retransmission ? ENABLE : DISABLE;
  can.Init.TransmitPause = ENABLE;
  can.Init.ProtocolException = DISABLE;

  auto nominal = ApplyRateOverride(MakeTime(options.slow_bitrate, 255, 127),
                                   options.rate_override);
  auto fast = ApplyRateOverride(MakeTime(options.fast_bitrate, 31, 15),
                                options.fdrate_override);

  config_.clock = HAL_RCC_GetPCLK1Freq();
  config_.nominal = nominal;
  config_.data = fast;

  can.Init.NominalPrescaler = nominal.prescaler;
  can.Init.NominalSyncJumpWidth = nominal.sync_jump_width;
  can.Init.NominalTimeSeg1 = nominal.time_seg1;
  can.Init.NominalTimeSeg2 = nominal.time_seg2;

  can.Init.DataPrescaler = fast.prescaler;
  can.Init.DataSyncJumpWidth = fast.sync_jump_width;
  can.Init.DataTimeSeg1 = fast.time_seg1;
  can.Init.DataTimeSeg2 = fast.time_seg2;

  can.Init.StdFiltersNbr =
      std::count_if(
          options.filters.begin, options.filters.end,
          [](const auto& filter) {
            return (filter.action != FilterAction::kDisable &&
                    filter.type == FilterType::kStandard);
          });
  can.Init.ExtFiltersNbr =
      std::count_if(
          options.filters.begin, options.filters.end,
          [](const auto& filter) {
            return (filter.action != FilterAction::kDisable &&
                    filter.type == FilterType::kExtended);
          });
  can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&can) != HAL_OK) {
    mbed_die();
  }

  int standard_index = 0;
  int extended_index = 0;
  std::for_each(
      options.filters.begin, options.filters.end,
      [&](const auto& filter) {
        if (filter.action == FilterAction::kDisable) {
          return;
        }

        FDCAN_FilterTypeDef sFilterConfig;
        sFilterConfig.IdType = [&]() {
          switch (filter.type) {
            case FilterType::kStandard: return FDCAN_STANDARD_ID;
            case FilterType::kExtended: return FDCAN_EXTENDED_ID;
          }
          mbed_die();
        }();
        sFilterConfig.FilterIndex = [&]() {
          switch (filter.type) {
            case FilterType::kStandard: {
              return standard_index++;
            }
            case FilterType::kExtended: {
              return extended_index++;
            }
          }
          mbed_die();
        }();
        sFilterConfig.FilterType = [&]() {
          switch (filter.mode) {
            case FilterMode::kRange: return FDCAN_FILTER_RANGE;
            case FilterMode::kDual: return FDCAN_FILTER_DUAL;
            case FilterMode::kMask: return FDCAN_FILTER_MASK;
          }
          mbed_die();
        }();

        sFilterConfig.FilterConfig = [&]() {
          switch (filter.action) {
            case FilterAction::kDisable:
            case FilterAction::kReject: return FDCAN_FILTER_REJECT;
            case FilterAction::kAccept: return FDCAN_FILTER_TO_RXFIFO0;
          }
          mbed_die();
        }();
        sFilterConfig.FilterID1 = filter.id1;
        sFilterConfig.FilterID2 = filter.id2;

        if (HAL_FDCAN_ConfigFilter(&can, &sFilterConfig) != HAL_OK)
        {
          mbed_die();
        }

      });

  auto map_filter_action = [](auto value) {
    switch (value) {
      case FilterAction::kDisable:
      case FilterAction::kAccept: {
        return FDCAN_ACCEPT_IN_RX_FIFO0;
      }
      case FilterAction::kReject: {
        return FDCAN_REJECT;
      }
    }
    mbed_die();
  };

  auto map_remote_action = [](auto value) {
    switch (value) {
      case FilterAction::kDisable:
      case FilterAction::kAccept: {
        return FDCAN_FILTER_REMOTE;
      }
      case FilterAction::kReject: {
        return FDCAN_REJECT_REMOTE;
      }
    }
    mbed_die();
  };

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(
          &can,
          map_filter_action(options.filters.global_std_action),
          map_filter_action(options.filters.global_ext_action),
          map_remote_action(options.filters.global_remote_std_action),
          map_remote_action(options.filters.global_remote_ext_action)) != HAL_OK) {
    mbed_die();
  }

  if (options.delay_compensation) {
    if (HAL_FDCAN_ConfigTxDelayCompensation(
            &can, options.tdc_offset, options.tdc_filter) != HAL_OK) {
      mbed_die();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&can) != HAL_OK) {
      mbed_die();
    }
  } else {
    if (HAL_FDCAN_DisableTxDelayCompensation(&can) != HAL_OK) {
      mbed_die();
    }
  }

  if (HAL_FDCAN_Start(&can) != HAL_OK) {
    mbed_die();
  }

  if (HAL_FDCAN_ActivateNotification(
          &can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    mbed_die();
  }
}

namespace {
bool ApplyOverride(bool value, FDCan::Override o) {
  using OV = FDCan::Override;
  switch (o) {
    case OV::kDefault: return value;
    case OV::kRequire: return true;
    case OV::kDisable: return false;
  }
  mbed_die();
}
}

void FDCan::Send(uint32_t dest_id,
                 std::string_view data,
                 const SendOptions& send_options) {

  // Abort anything we have started that hasn't finished.
  if (last_tx_request_) {
    HAL_FDCAN_AbortTxRequest(&hfdcan1_, last_tx_request_);
  }

  FDCAN_TxHeaderTypeDef tx_header;
  tx_header.Identifier = dest_id;
  tx_header.IdType = ApplyOverride(
      dest_id >= 2048, send_options.extended_id) ?
      FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx_header.TxFrameType =
      ApplyOverride(options_.remote_frame,
                    send_options.remote_frame) ?
      FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  tx_header.DataLength = RoundUpDlc(data.size());
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch =
      ApplyOverride(options_.bitrate_switch,
                    send_options.bitrate_switch) ?
      FDCAN_BRS_ON : FDCAN_BRS_OFF;
  tx_header.FDFormat =
      ApplyOverride(options_.fdcan_frame,
                    send_options.fdcan_frame) ?
      FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(
          &hfdcan1_, &tx_header,
          const_cast<uint8_t*>(
              reinterpret_cast<const uint8_t*>(data.data()))) != HAL_OK) {
    mbed_die();
  }
  last_tx_request_ = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1_);
}

bool FDCan::Poll(FDCAN_RxHeaderTypeDef* header,
                 mjlib::base::string_span data) {
  if (HAL_FDCAN_GetRxMessage(
          &hfdcan1_, FDCAN_RX_FIFO0, header,
          reinterpret_cast<uint8_t*>(data.data())) != HAL_OK) {
    return false;
  }

  return true;
}

void FDCan::RecoverBusOff() {
  hfdcan1_.Instance->CCCR &= ~FDCAN_CCCR_INIT;
}


FDCAN_ProtocolStatusTypeDef FDCan::status() {
  HAL_FDCAN_GetProtocolStatus(&hfdcan1_, &status_result_);
  return status_result_;
}

FDCan::Config FDCan::config() const {
  return config_;
}

int FDCan::ParseDlc(uint32_t dlc_code) {
  if (dlc_code == FDCAN_DLC_BYTES_0) { return 0; }
  if (dlc_code == FDCAN_DLC_BYTES_1) { return 1; }
  if (dlc_code == FDCAN_DLC_BYTES_2) { return 2; }
  if (dlc_code == FDCAN_DLC_BYTES_3) { return 3; }
  if (dlc_code == FDCAN_DLC_BYTES_4) { return 4; }
  if (dlc_code == FDCAN_DLC_BYTES_5) { return 5; }
  if (dlc_code == FDCAN_DLC_BYTES_6) { return 6; }
  if (dlc_code == FDCAN_DLC_BYTES_7) { return 7; }
  if (dlc_code == FDCAN_DLC_BYTES_8) { return 8; }
  if (dlc_code == FDCAN_DLC_BYTES_12) { return 12; }
  if (dlc_code == FDCAN_DLC_BYTES_16) { return 16; }
  if (dlc_code == FDCAN_DLC_BYTES_20) { return 20; }
  if (dlc_code == FDCAN_DLC_BYTES_24) { return 24; }
  if (dlc_code == FDCAN_DLC_BYTES_32) { return 32; }
  if (dlc_code == FDCAN_DLC_BYTES_48) { return 48; }
  if (dlc_code == FDCAN_DLC_BYTES_64) { return 64; }
  mbed_die();
  return 0;
}

}
