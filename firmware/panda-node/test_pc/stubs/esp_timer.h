#pragma once
#include <cstdint>
typedef int esp_err_t;
#define ESP_OK 0
typedef struct esp_timer* esp_timer_handle_t; typedef void (*esp_timer_cb_t)(void*);
typedef enum { ESP_TIMER_TASK } esp_timer_dispatch_t;
typedef struct { esp_timer_cb_t callback; void* arg; esp_timer_dispatch_t dispatch_method; const char* name; bool skip_unhandled_events; } esp_timer_create_args_t;
int64_t esp_timer_get_time(); esp_err_t esp_timer_create(const esp_timer_create_args_t*, esp_timer_handle_t*);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t, uint64_t);
