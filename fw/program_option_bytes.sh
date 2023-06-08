#!/bin/bash

# This clears the nSWBOOT0 option byte, which forces the device to
# start executing from user code regardless of the state of the PB8
# pin.

openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "init" \
    -c "reset halt" \
    -c "stm32l4x option_write 0 0x20 0x00000000 0x04000000" \
    -c "stm32l4x option_load 0" \
    -c "reset" \
    -c "exit"
