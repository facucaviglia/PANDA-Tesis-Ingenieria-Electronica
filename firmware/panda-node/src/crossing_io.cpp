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

// Solo se llama desde el timer: el buzzer tiene un único dueño. Con low, el
// nivel reducido de la campana con el brazo horizontal (Anexo XII 5.5).
static void buzzer(bool on, bool low) {
  static int last = -1;  // 0 apagado, 1 pleno, 2 reducido
  const int level = !on ? 0 : (low ? 2 : 1);
  if (level == last) return;
  last = level;
  if (cfg::crossing::kBuzzerIsActive) {
    // Un buzzer activo no se puede atenuar.
    writePin(pins::kOutBuzzer, on);
    return;
  }
  // ledcWriteTone con frecuencia 0 apaga el PWM. Con otra frecuencia deja el
  // ciclo de trabajo al 50 % (nivel pleno), que después se baja si hace falta.
  ledcWriteTone(kBuzzerChannel, on ? cfg::crossing::kBeepHz : 0);
  if (level == 2) ledcWrite(kBuzzerChannel, cfg::crossing::kBellLowDuty);
}

// Fase de una señal intermitente del Anexo XII que arrancó en sinceUs:
// medio segundo encendida, medio segundo apagada, empezando encendida.
static bool flashOn(int64_t nowUs, int64_t sinceUs) {
  return ((nowUs - sinceUs) / 1000 / cfg::crossing::kFlashHalfMs) % 2 == 0;
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

struct Bell {
  bool on;   // La campana suena mientras hay señales
  bool low;  // Brazo horizontal: nivel reducido
};

static Bell barrierTick(int64_t now) {
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
      // Las señales siguen encendidas durante la subida, así que si el pedido
      // vuelve, el descenso empieza con las luces ya encendidas sin corte
      // desde la fonoluminosa: el preaviso de 5 s del SETOP 8.6.6 se cumple.
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

  // SETOP 8.6.6: las luces siguen hasta que el brazo recupera la vertical. El
  // Anexo XII (punto 20) las corta al iniciar el ascenso, pero el propio
  // Anexo (4.1) obliga a cumplir el SETOP, que es obligatorio y no admite
  // acuerdos que lo violen (SETOP 1.2 y 1.4).
  const bool signals = p != BarrierPhase::Arriba;
  const bool aOn = ((now / 1000) / cfg::crossing::kBarrierLightHalfMs) % 2 == 0;
  writePin(pins::kOutBarrierLightA, signals && aOn);
  writePin(pins::kOutBarrierLightB, signals && !aOn);
  return {signals, p == BarrierPhase::Abajo};
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

  // Un solo buzzer para dos fuentes: la campana de la maqueta (un toque por
  // segundo, SETOP 8.6.7) y el aviso de PANDA (uno por segundo, dos con OTRO
  // TREN). Suena si toca cualquiera de las dos. Con el brazo horizontal baja
  // todo el buzzer, porque en la maqueta es un único emisor.
  const Bell bell = cfg::crossing::kBarrierOnBoard ? barrierTick(now) : Bell{false, false};
  const int64_t ms = now / 1000;
  const bool bellBeat = bell.on && (ms % cfg::crossing::kBeepPeriodMs) < cfg::crossing::kBeepOnMs;
  const uint32_t pandaPeriod =
      s_otherTrain.load() ? cfg::crossing::kBeepPeriodOtherTrainMs : cfg::crossing::kBeepPeriodMs;
  const bool pandaBeat = s_pandaSound.load() && (ms % pandaPeriod) < cfg::crossing::kBeepOnMs;
  buzzer((bellBeat || pandaBeat) && !s_muted.load(), bell.low);
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
  // Solo la llama la tarea de decisión, así que estos estáticos tienen un
  // único dueño. Las intermitencias se calculan acá, a 20 Hz.
  static bool lastWarn = false;
  static bool lastOther = false;
  static int64_t warnSinceUs = 0;
  static int64_t otherSinceUs = 0;
  const int64_t now = esp_timer_get_time();
  if (out.sound && !lastWarn) warnSinceUs = now;
  if (out.otherTrain && !lastOther) otherSinceUs = now;
  lastWarn = out.sound;
  lastOther = out.otherTrain;

  // Anexo XII 4.2: el rojo peatonal es intermitente durante t_p desde que
  // arranca el aviso y después fijo. En FALLA o al arrancar queda fijo.
  const bool pedFlash =
      out.sound && (now - warnSinceUs) < static_cast<int64_t>(cfg::crossing::kPedestrianCrossS * 1e6f);
  const bool pedOn = out.pedestrian && (!pedFlash || flashOn(now, warnSinceUs));
  // Anexo XII 4.2: OTRO TREN intermitente cada medio segundo.
  const bool otherOn = out.otherTrain && flashOn(now, otherSinceUs);

  writeContacts(out.pandaLibre, out.pandaOk);
  writePin(pins::kOutPedestrian, pedOn);
  writePin(pins::kOutOtherTrain, otherOn);
  s_pandaSound.store(out.sound);
  s_otherTrain.store(out.otherTrain);
  s_muted.store(out.muted);

  s_lastApplyUs.store(now);
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
