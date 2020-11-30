#!/bin/bash

openocd \
    -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
    -f /usr/share/openocd/scripts/target/stm32f4x_stlink.cfg \
    -c "reset_config none separate"
