#include "button.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "board_pins.h"
#include "config.h"

namespace button {

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_pressed = false;
static volatile int64_t s_pressStartUs = 0;
static volatile int64_t s_lastEdgeUs = 0;
static volatile uint32_t s_markCount = 0;
static volatile int64_t s_markUs = 0;
static volatile uint32_t s_longCount = 0;
static uint32_t s_markReported = 0;
static uint32_t s_longReported = 0;

// La ISR ve los dos flancos. Al soltar decide si fue corta o larga según la
// duración. Los rebotes (flancos a menos de kButtonDebounceUs) se ignoran.
static void IRAM_ATTR onEdgeIsr() {
  const int64_t now = esp_timer_get_time();
  const bool down = gpio_get_level(static_cast<gpio_num_t>(pins::kButton)) == 0;
  portENTER_CRITICAL_ISR(&s_mux);
  if (now - s_lastEdgeUs >= static_cast<int64_t>(cfg::ui::kButtonDebounceUs)) {
    if (down && !s_pressed) {
      s_pressed = true;
      s_pressStartUs = now;
    } else if (!down && s_pressed) {
      s_pressed = false;
      if (now - s_pressStartUs >= static_cast<int64_t>(cfg::ui::kLongPressUs)) {
        s_longCount = s_longCount + 1;
      } else {
        s_markCount = s_markCount + 1;
        s_markUs = s_pressStartUs;
      }
    }
    s_lastEdgeUs = now;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

void begin() {
  pinMode(pins::kButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pins::kButton), onEdgeIsr, CHANGE);
}

void injectMark() {
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&s_mux);
  s_markCount = s_markCount + 1;
  s_markUs = now;
  portEXIT_CRITICAL(&s_mux);
}

Event poll(uint32_t& markNumber, int64_t& markUs) {
  portENTER_CRITICAL(&s_mux);
  const uint32_t marks = s_markCount;
  const int64_t mUs = s_markUs;
  const uint32_t longs = s_longCount;
  portEXIT_CRITICAL(&s_mux);

  if (marks != s_markReported) {
    s_markReported = marks;
    markNumber = marks;
    markUs = mUs;
    return Event::Mark;
  }
  if (longs != s_longReported) {
    s_longReported = longs;
    return Event::LongPress;
  }
  return Event::None;
}

}  // namespace button
