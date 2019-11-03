#!/bin/bash

openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c init -c "reset_config none separate; reset init"
