#!/bin/bash

openocd \
    -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
    -f /usr/share/openocd/scripts/target/stm32f4x.cfg \
    -c "init" \
    -c "reset_config none separate; program bazel-out/stm32f4-opt/bin/moteus/moteus.08000000.bin verify 0x8000000; program bazel-out/stm32f4-opt/bin/moteus/bootloader.0800c000.bin verify 0x800c000; program bazel-out/stm32f4-opt/bin/moteus/moteus.08010000.bin verify reset exit 0x8010000"
