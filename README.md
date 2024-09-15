This is simple Bluetooth CRSF receiver to control Flight Controllers with regular bluetooth gamepads, it based on bluepad32 and btstack and partly on betaflight code for CRSF protocol.

| Supported Targets | ESP32 | 
| ----------------- | ----- | 

Note: ESP32S3 support only BLE gamepads, for example DualSense doesn't support BLE and don't with ESP32S3, look at espressif docs and bluepad32 project for supported hardware.

## To Install
`git clone <this repo>`
`cd components`
`git clone https://github.com/ricardoquesada/bluepad32.git`
`cd bluepad32 && git checkout tags/4.1.0`
`git submodule update --init --recursive` - to download btstack
The follow install steps from bluepad32 for esp32 instruction: https://github.com/ricardoquesada/bluepad32/tree/main/examples/esp32

after all use idf to build and flash esp32. I recommend IDF extension for VSCode to get fancy develope experience.

Make attension on sdkonfig options from bluepad32 example and ensure FreeRTOS tick rate set to 1000HZ or you can't achieve 200-1000Hz CRSF frames speed.
