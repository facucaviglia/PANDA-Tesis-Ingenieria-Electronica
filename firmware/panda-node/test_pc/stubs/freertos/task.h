#pragma once
#include "FreeRTOS.h"
typedef struct TaskDef* TaskHandle_t; typedef void (*TaskFunction_t)(void*);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*, BaseType_t);
TickType_t xTaskGetTickCount(); void vTaskDelayUntil(TickType_t*, TickType_t); void vTaskDelay(TickType_t);
