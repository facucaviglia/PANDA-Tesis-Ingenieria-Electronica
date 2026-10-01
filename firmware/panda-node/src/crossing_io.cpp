#include "crossing_io.h"

#include <Arduino.h>
#include <algorithm>
#include <atomic>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "board_pins.h"
#include "config.h"
#include "system_state.h"

namespace crossio {

static constexpr uint8_t kBuzzerChannel = 0;
static constexpr uint8_t kBuzzerResolution = 10;
// Canal 2: usa otro timer del LEDC que el buzzer (canal 0), así cada uno
// tiene su frecuencia. 50 Hz con 14 bits: 1,22 µs por cuenta.
static constexpr uint8_t kServoChannel = 2;
static constexpr uint8_t kServoResolution = 14;
static constexpr uint32_t kServoHz = 50;
static constexpr uint32_t kTickUs = 50000;

static std::atomic<int64_t> s_lastApplyUs{0};
static std::atomic<uint32_t> s_trips{0};
static std::atomic<bool> s_tripped{false};
static esp_timer_handle_t s_timer = nullptr;

// Contactos de PANDA hacia el controlador de barrera. Con la maqueta en la
// placa son estas variables, con la barrera externa además son los relés.
static std::atomic<bool> s_contactLibre{false};
static std::atomic<bool> s_contactOk{false};
static std::atomic<bool> s_pandaSound{false};
static std::atomic<bool> s_otherTrain{false};
static std::atomic<bool> s_muted{false};
static std::atomic<bool> s_simTrack{false};
static std::atomic<uint8_t> s_phase{static_cast<uint8_t>(BarrierPhase::Arriba)};

static void writePin(int pin, bool high) {
  gpio_set_level(static_cast<gpio_num_t>(pin), high ? 1 : 0);
}

// Solo se llama desde el timer: el buzzer tiene un único dueño.
static void buzzer(bool on) {
  static bool last = false;
  static bool first = true;
  if (!first && on == last) return;
  first = false;
  last = on;
  if (cfg::crossing::kBuzzerIsActive) {
    writePin(pins::kOutBuzzer, on);
    return;
  }
  // ledcWriteTone con frecuencia 0 apaga el PWM.
  ledcWriteTone(kBuzzerChannel, on ? cfg::crossing::kBeepHz : 0);
}

static void writeContacts(bool libre, bool ok) {
  s_contactLibre.store(libre);
  s_contactOk.store(ok);
  if (!cfg::crossing::kBarrierOnBoard) {
    writePin(pins::kOutPandaLibre, libre);
    writePin(pins::kOutPandaOk, ok);
  }
}

// Estado seguro: PANDA pide cierre y se declara no operativo, la señal
// peatonal queda encendida y el aviso de PANDA apagado (un sonido permanente
// por una falla solo enseña a ignorarlo). La maqueta sigue funcionando y pasa
// a obedecer solo al circuito de vía.
static void forceSafe() {
  writeContacts(false, false);
  writePin(pins::kOutPedestrian, true);
  writePin(pins::kOutOtherTrain, false);
  s_pandaSound.store(false);
  s_otherTrain.store(false);
}

bool readTrackOccupied() {
  return gpio_get_level(static_cast<gpio_num_t>(pins::kInTrackCircuit)) != 0;
}

bool trackOccupied() {
  return cfg::crossing::kTrackCircuitEnabled ? readTrackOccupied() : s_simTrack.load();
}

void setSimulatedTrack(bool occupied) {
  s_simTrack.store(occupied);
}

// ---------------------------------------------------------------------------
// Maqueta de barrera
// ---------------------------------------------------------------------------
static float s_armPos = 0.0f;  // 0 = vertical (abierta), 1 = horizontal (cerrada)
static int64_t s_phaseSinceUs = 0;

static void setServo(float pos) {
  const float us = cfg::crossing::kServoUpUs + pos * (static_cast<float>(cfg::crossing::kServoDownUs) -
                                                      static_cast<float>(cfg::crossing::kServoUpUs));
  const uint32_t maxCount = (1u << kServoResolution) - 1u;
  ledcWrite(kServoChannel, static_cast<uint32_t>(us / (1e6f / kServoHz) * maxCount));
}

static bool barrierTick(int64_t now) {
  const bool ok = s_contactOk.load();
  const bool request = ok ? !s_contactLibre.load() : trackOccupied();
  BarrierPhase p = static_cast<BarrierPhase>(s_phase.load());
  const float dt = kTickUs / 1e6f;
  const auto enter = [&](BarrierPhase next) {
    p = next;
    s_phaseSinceUs = now;
  };

  switch (p) {
    case BarrierPhase::Arriba:
      if (request) enter(BarrierPhase::Fono);
      break;
    case BarrierPhase::Fono:
      if (!request) {
        enter(BarrierPhase::Arriba);
      } else if (now - s_phaseSinceUs >= static_cast<int64_t>(cfg::crossing::kFonoluminosaS * 1e6f)) {
        enter(BarrierPhase::Bajando);
      }
      break;
    case BarrierPhase::Bajando:
      s_armPos = std::min(1.0f, s_armPos + dt / cfg::crossing::kModelArmDownS);
      if (s_armPos >= 1.0f) enter(BarrierPhase::Abajo);
      break;
    case BarrierPhase::Abajo:
      if (!request) enter(BarrierPhase::Subiendo);
      break;
    case BarrierPhase::Subiendo:
      if (request) {
        enter(BarrierPhase::Bajando);
        break;
      }
      s_armPos = std::max(0.0f, s_armPos - dt / cfg::crossing::kModelArmUpS);
      if (s_armPos <= 0.0f) enter(BarrierPhase::Arriba);
      break;
  }
  s_phase.store(static_cast<uint8_t>(p));
  setServo(s_armPos);

  const bool signals = (p == BarrierPhase::Fono || p == BarrierPhase::Bajando || p == BarrierPhase::Abajo);
  const bool aOn = ((now / 1000) / cfg::crossing::kBarrierLightHalfMs) % 2 == 0;
  writePin(pins::kOutBarrierLightA, signals && aOn);
  writePin(pins::kOutBarrierLightB, signals && !aOn);
  return signals;  // La campana suena mientras hay señales
}

// ---------------------------------------------------------------------------
// Timer del core 0: vigilancia, maqueta y sonido
// ---------------------------------------------------------------------------
static void onTick(void*) {
  const int64_t now = esp_timer_get_time();
  const int64_t last = s_lastApplyUs.load();
  if (last != 0 && now - last > static_cast<int64_t>(cfg::crossing::kHeartbeatTimeoutMs) * 1000) {
    forceSafe();
    if (!s_tripped.exchange(true)) {
      s_trips.fetch_add(1);
      logNote(NoteCode::WatchdogTrip, s_trips.load());
    }
  }

  const bool bell = cfg::crossing::kBarrierOnBoard ? barrierTick(now) : false;
  const uint32_t period =
      s_otherTrain.load() ? cfg::crossing::kBeepPeriodOtherTrainMs : cfg::crossing::kBeepPeriodMs;
  const uint32_t phaseMs = static_cast<uint32_t>((now / 1000) % period);
  const bool soundOn = (s_pandaSound.load() || bell) && !s_muted.load();
  buzzer(soundOn && phaseMs < cfg::crossing::kBeepOnMs);
}

void begin() {
  // Primero los pines en bajo, lo antes posible después del arranque.
  const int outs[] = {pins::kOutPandaLibre, pins::kOutPandaOk, pins::kOutPedestrian, pins::kOutOtherTrain,
                      pins::kOutBarrierLightB};
  for (int pin : outs) {
    pinMode(pin, OUTPUT);
    writePin(pin, false);
  }
  // Mientras arranca, la señal peatonal encendida hace además de prueba de
  // lámpara: el que mira ve que el LED funciona.
  writePin(pins::kOutPedestrian, true);

  if (cfg::crossing::kBuzzerIsActive) {
    pinMode(pins::kOutBuzzer, OUTPUT);
  } else {
    ledcSetup(kBuzzerChannel, cfg::crossing::kBeepHz, kBuzzerResolution);
    ledcAttachPin(pins::kOutBuzzer, kBuzzerChannel);
  }

  if (cfg::crossing::kBarrierOnBoard) {
    ledcSetup(kServoChannel, kServoHz, kServoResolution);
    ledcAttachPin(pins::kOutServo, kServoChannel);
    setServo(0.0f);
  }

  pinMode(pins::kInTrackCircuit, INPUT_PULLUP);

  const esp_timer_create_args_t args = {
      .callback = onTick,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "cruce-io",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&args, &s_timer) == ESP_OK) {
    esp_timer_start_periodic(s_timer, kTickUs);
  }
}

void apply(const Outputs& out) {
  writeContacts(out.pandaLibre, out.pandaOk);
  writePin(pins::kOutPedestrian, out.pedestrian);
  writePin(pins::kOutOtherTrain, out.otherTrain);
  s_pandaSound.store(out.sound);
  s_otherTrain.store(out.otherTrain);
  s_muted.store(out.muted);

  s_lastApplyUs.store(esp_timer_get_time());
  s_tripped.store(false);
  // TODO(tpl5010): pulso en el pin DONE del watchdog externo (ver config.h).
}

BarrierPhase barrierPhase() {
  return static_cast<BarrierPhase>(s_phase.load());
}

const char* barrierPhaseName(BarrierPhase p) {
  switch (p) {
    case BarrierPhase::Arriba:
      return "ARRIBA";
    case BarrierPhase::Fono:
      return "FONO";
    case BarrierPhase::Bajando:
      return "BAJANDO";
    case BarrierPhase::Abajo:
      return "ABAJO";
    case BarrierPhase::Subiendo:
      return "SUBIENDO";
  }
  return "?";
}

uint32_t watchdogTrips() {
  return s_trips.load();
}

}  // namespace crossio
