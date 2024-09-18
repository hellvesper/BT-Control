This is simple Bluetooth CRSF receiver to control Flight Controllers with regular bluetooth gamepads, it based on bluepad32 and btstack and partly on betaflight code for CRSF protocol.

| Supported Targets | ESP32 | 
| ----------------- | ----- | 

[![.github/workflows/build.yml](https://github.com/hellvesper/BT-Control/actions/workflows/build.yml/badge.svg)](https://github.com/hellvesper/BT-Control/actions/workflows/build.yml)


Note: ESP32S3 support only BLE gamepads, for example DualSense doesn't support BLE and don't with ESP32S3, look at espressif docs and bluepad32 project for supported hardware.

## To Install
* Install ESP-IDF - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation
    Or VSCode IDF Extension https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md or https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension
* Clone repo `git clone https://github.com/hellvesper/BT-Control.git`
`git submodule update --init --recursive` - to download bluepad32 component and btstack
* If you use IDF Extenstion then click to `Open ESP-IDF Terminal`, if you use manual ESP-IDF installation then run `. <path_to_esp_idf>/export.sh` to source environment
* Integrate btstack running `install_btstack.sh`

Now your project is ready to build. Run `idf.py build` or *Buld Project* in VSCode


Make attension on sdkonfig options from bluepad32 example and ensure FreeRTOS tick rate set to 1000HZ or you can't achieve 200-1000Hz CRSF frames speed. Use `idf.py menuconfig` or *SDK Configuration Editor* to set tick rate and bluepad32 options
