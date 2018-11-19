#!/bin/bash

openocd -f /usr/share/openocd/scripts/board/st_nucleo_f4.cfg -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg -c init -c "reset_config none separate; reset init"
