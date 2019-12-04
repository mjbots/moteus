#!/bin/bash

openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "init" \
    -c "reset_config none separate; program bazel-out/stm32g4-opt/bin/moteus/moteus.08000000.bin verify 0x8000000; program bazel-out/stm32g4-opt/bin/moteus/moteus.08010000.bin verify reset exit 0x8010000"
