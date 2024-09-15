#ifndef QUEUE_HANDLER_H
#define QUEUE_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t gamepad_queue;

void init_gamepad_queue(void);

#endif // QUEUE_HANDLER_H