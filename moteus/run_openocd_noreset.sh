#!/bin/bash

openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4.cfg \
    -c "reset_config none separate"
