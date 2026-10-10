#pragma once
#include <cstdint>
typedef uint32_t TickType_t; typedef int BaseType_t; typedef unsigned UBaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
typedef struct { int x; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
void portENTER_CRITICAL(portMUX_TYPE*); void portEXIT_CRITICAL(portMUX_TYPE*);
