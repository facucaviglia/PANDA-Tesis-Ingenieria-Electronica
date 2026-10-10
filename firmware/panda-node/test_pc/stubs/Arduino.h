#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
struct HardwareSerial {
  int printf(const char*, ...) __attribute__((format(printf, 2, 3)));
  size_t println(const char*); size_t println(); size_t print(const char*); size_t print(char);
  int available(); int read(); void begin(unsigned long); void flush(); size_t write(uint8_t);
  size_t write(const uint8_t*, size_t);
};
extern HardwareSerial Serial;
struct EspClass { uint32_t getFreePsram(); uint32_t getFreeHeap(); void restart(); };
extern EspClass ESP;
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define INPUT 0x01
unsigned long millis(); unsigned long micros(); void delay(uint32_t);
void pinMode(uint8_t, uint8_t);
double ledcSetup(uint8_t, double, uint8_t); void ledcAttachPin(uint8_t, uint8_t);
void ledcWrite(uint8_t, uint32_t); double ledcWriteTone(uint8_t, double);
