/* 
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "queue_handler.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

// bluepad32
#include <stdlib.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "sdkconfig.h"

// crsf
#include "crsf.h"

// uncomment for official baud rate 416666 Kbit/s
// #define USE_CRSF_OFFICIAL_SPEC

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

static const char *TAG = "main";

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(uni_gamepad_t)

QueueHandle_t gamepad_queue;
uni_gamepad_t gamepad_data;

void init_gamepad_queue() {
    gamepad_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (gamepad_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create gamepad queue");
    } else {
        ESP_LOGI(TAG, "Gamepad queue created successfully");
    }
}

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const uint8_t* data, const int len)
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    static uint8_t rc_channels_buf[CRSF_FRAME_SIZE_RC] = {0};
    /**
     * init channels to default values: AETR1234, AER - MID, T - MIN
     * Roll[A] - MID
     * Pitch[E] - MID
     * Yaw[R] - MID
     * Throttle[T] - MIN
     * 1234 - doesn't matter
     */
    static rcChannelsUnpacked_t channel = {CRSF_CHANNEL_MIN};
    channel[0] = CRSF_CHANNEL_MID; // A
    channel[1] = CRSF_CHANNEL_MID; // E
    channel[2] = CRSF_CHANNEL_MIN; // T
    channel[3] = CRSF_CHANNEL_MID; // R

    while (1) {
        if (xQueueReceive(gamepad_queue, &gamepad_data, 0) == pdPASS) {
            // Virtual Gamepad layout:
            //
            //  Left             Center            Right
            //
            //  brake: 0-1023    Menu button       throttle: 0-1023
            //  L-shoulder button                  R-shoulder button
            //  L-trigger button                   R-trigger button
            //  d-pad                              buttons: A,B,X,Y,
            //  L-joypad (axis: -512, 511)         R-joypad (axis: -512, 511)
            //  axis-L button                      axis-R button
            //  Gyro: is measured in degress/second
            //  Accelerometer: is measured in "G"s
            // #define CRSF_CHANNEL_MIN    172
            // #define CRSF_CHANNEL_MID    992
            // #define CRSF_CHANNEL_MAX    1811
            // Transform signed values in range -512..511 to 0..1023
            // AETR1234, AER - MID, T - MIN

            int16_t stick_lx  =  (((gamepad_data.axis_x + 512) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            // int16_t stick_ly =  gamepad_data.axis_y + 512;
            int16_t stick_rx  = (((gamepad_data.axis_rx + 512) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            int16_t stick_ry  = (((gamepad_data.axis_ry + 512) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            uint32_t throttle = ((gamepad_data.throttle * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            // uint32_t brake =    ((gamepad_data.brake    * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            channel[0] = (uint32_t)stick_rx; // Roll    [A]
            channel[1] = (uint32_t)stick_ry; // Pitch   [E]
            channel[2] = throttle;           // Throttle[T]
            channel[3] = (uint32_t)stick_lx; // Yaw     [R]
            /*
            Add additional channels if needed
            */
            build_frame(channel, rc_channels_buf, CRSF_FRAME_SIZE_RC);

        }
        sendData(TX_TASK_TAG, rc_channels_buf, CRSF_FRAME_SIZE_RC);
        vTaskDelay(5 / portTICK_PERIOD_MS); // 5ms delay for 200HZ
    }
}
#if 0
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}
#endif
static void bt_task(void *arg) {
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

    // Don't use BTstack buffered UART. It conflicts with the console.
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    // Does not return.
    btstack_run_loop_execute();

}


void app_main(void)
{
    init();
    init_gamepad_queue(); // Ensure the queue is initialized before creating tasks
    // xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(bt_task, "bt_gpad_task", 1024 * 5, NULL, configMAX_PRIORITIES - 2, NULL);
}
