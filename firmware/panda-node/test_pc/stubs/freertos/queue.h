#pragma once
#include "FreeRTOS.h"
typedef struct QueueDef* QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t, UBaseType_t);
BaseType_t xQueuePeek(QueueHandle_t, void*, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t);
BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t);
BaseType_t xQueueOverwrite(QueueHandle_t, const void*);
