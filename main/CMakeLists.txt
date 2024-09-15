set(srcs "uart_async_rxtxtasks_main.c" "gamepad.c" "crsf.c" "crc.c")

set(requires "bluepad32" "btstack")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")

# idf_component_register(SRCS "uart_async_rxtxtasks_main.c"
#                     INCLUDE_DIRS ".")
