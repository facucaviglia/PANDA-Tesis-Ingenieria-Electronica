#pragma once
#include "esp_timer.h"
#include "freertos/task.h"
esp_err_t esp_task_wdt_init(uint32_t, bool); esp_err_t esp_task_wdt_add(TaskHandle_t); esp_err_t esp_task_wdt_reset();
