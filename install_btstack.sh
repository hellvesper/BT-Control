#!/bin/sh

export BLUEPAD32="$(pwd)/components/bluepad32"

cd ${BLUEPAD32}/external/btstack/port/esp32
# This will install BTstack as a component inside Bluepad32 source code (recommended).
# Remove "IDF_PATH=../../../../src" if you want it installed in the ESP-IDF folder
IDF_PATH=../../../../src ./integrate_btstack.py
