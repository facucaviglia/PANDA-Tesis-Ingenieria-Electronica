#include "crossing_io.h"

#include <Arduino.h>
#include <atomic>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "board_pins.h"
#include "config.h"
#include "system_state.h"

namespace crossio {

static constexpr uint8_t kBuzzerChannel = 0;
static constexpr uint8_t kBuzzerResolution = 10;

static std::atomic<int64_t> s_lastApplyUs{0};
static std::atomic<uint32_t> s_trips{0};
static std::atomic<bool> s_tripped{false};
static esp_timer_handle_t s_watchdog = nullptr;

static void writePin(int pin, bool high) {
  gpio_set_level(static_cast<gpio_num_t>(pin), high ? 1 : 0);
}

static void buzzer(bool on) {
  // ledcWriteTone con frecuencia 0 apaga el PWM.
  ledcWriteTone(kBuzzerChannel, on ? cfg::crossing::kBeepHz : 0);
}

// Estado seguro: PANDA pide cierre y se declara no operativo, la señal
// peatonal queda encendida y el sonido apagado (un sonido permanente por una
// falla solo enseña a ignorarlo).
static void forceSafe() {
  writePin(pins::kOutPandaLibre, false);
  writePin(pins::kOutPandaOk, false);
  writePin(pins::kOutPedestrian, true);
  buzzer(false);
}

// Corre en la tarea de esp_timer, en el core 0, independiente del core 1.
static void onWatchdog(void*) {
  const int64_t last = s_lastApplyUs.load();
  if (last == 0) {
    return;  // Todavía no arrancó la decisión: los pines siguen en su estado inicial
  }
  if (esp_timer_get_time() - last > static_cast<int64_t>(cfg::crossing::kHeartbeatTimeoutMs) * 1000) {
    forceSafe();
    if (!s_tripped.exchange(true)) {
      s_trips.fetch_add(1);
      logNote(NoteCode::WatchdogTrip, s_trips.load());
    }
  }
}

void begin() {
  // Primero los pines en bajo, lo antes posible después del arranque.
  for (int pin : {pins::kOutPandaLibre, pins::kOutPandaOk, pins::kOutPedestrian}) {
    pinMode(pin, OUTPUT);
    writePin(pin, false);
  }
  // Mientras arranca, la señal peatonal encendida hace además de prueba de
  // lámpara: el que mira ve que el LED funciona.
  writePin(pins::kOutPedestrian, true);

  ledcSetup(kBuzzerChannel, cfg::crossing::kBeepHz, kBuzzerResolution);
  ledcAttachPin(pins::kOutBuzzer, kBuzzerChannel);
  buzzer(false);

  pinMode(pins::kInTrackCircuit, INPUT_PULLUP);

  const esp_timer_create_args_t args = {
      .callback = onWatchdog,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "cruce-wdt",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&args, &s_watchdog) == ESP_OK) {
    esp_timer_start_periodic(s_watchdog, 50000);
  }
}

void apply(const Outputs& out) {
  // Sonido: un toque de kBeepOnMs cada kBeepPeriodMs, con la fase tomada del
  // reloj para que el ritmo sea regular aunque la tarea se atrase un poco.
  const uint32_t phaseMs = static_cast<uint32_t>((esp_timer_get_time() / 1000) % cfg::crossing::kBeepPeriodMs);
  buzzer(out.sound && phaseMs < cfg::crossing::kBeepOnMs);

  writePin(pins::kOutPandaLibre, out.pandaLibre);
  writePin(pins::kOutPandaOk, out.pandaOk);
  writePin(pins::kOutPedestrian, out.pedestrian);

  s_lastApplyUs.store(esp_timer_get_time());
  s_tripped.store(false);
}

bool readTrackOccupied() {
  return gpio_get_level(static_cast<gpio_num_t>(pins::kInTrackCircuit)) != 0;
}

uint32_t watchdogTrips() {
  return s_trips.load();
}

}  // namespace crossio
