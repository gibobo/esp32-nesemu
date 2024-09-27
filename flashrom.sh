#!/bin/bash
. ${IDF_PATH}/add_path.sh
esptool.py --chip esp32 --port "/dev/ttyUSB0" --baud $((115200*1)) write_flash -fs 4MB 0x100000 "$1"
