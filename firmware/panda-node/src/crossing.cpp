#include "crossing.h"

#include <Arduino.h>
#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include "config.h"
#include "crossing_io.h"
#include "gnss_manager.h"
#include "persist.h"
#include "radio_link.h"
#include "system_state.h"

namespace crossing {

// ---------------------------------------------------------------------------
// Nombres para pantalla, consola y registros
// ---------------------------------------------------------------------------
const char* stateName(CrossState s) {
  switch (s) {
    case CrossState::Iniciando:
      return "INICIANDO";
    case CrossState::Apagado:
      return "APAGADO";
    case CrossState::NoSeguro:
      return "NO SEGURO";
    case CrossState::Falla:
      return "FALLA";
  }
  return "?";
}

const char* reasonName(CrossReason r) {
  switch (r) {
    case CrossReason::Ninguno:
      return "sin trenes";
    case CrossReason::TrenAproxima:
      return "tren aproxima";
    case CrossReason::TrenEnZona:
      return "tren en zona";
    case CrossReason::SinDatos:
      return "SIN DATOS";
    case CrossReason::SinPosicion:
      return "tren sin posicion";
    case CrossReason::NoVerificable:
      return "tren no verificable";
    case CrossReason::ViaOcupada:
      return "via ocupada";
    case CrossReason::FallaRadio:
      return "falla de radio";
    case CrossReason::FallaTiempo:
      return "sin tiempo GPS";
    case CrossReason::FallaReferencia:
      return "sin posicion cruce";
    case CrossReason::Arranque:
      return "arrancando";
  }
  return "?";
}

const char* phaseName(TrainPhase p) {
  switch (p) {
    case TrainPhase::Lejos:
      return "LEJOS";
    case TrainPhase::Aproxima:
      return "APROXIMA";
    case TrainPhase::EnZona:
      return "EN_ZONA";
    case TrainPhase::Alejandose:
      return "SE_ALEJA";
    case TrainPhase::SinDatos:
      return "SIN_DATOS";
  }
  return "?";
}

const char* refSourceName(RefSource s) {
  switch (s) {
    case RefSource::Ninguna:
      return "ninguna";
    case RefSource::Gnss:
      return "GNSS";
    case RefSource::Guardada:
      return "guardada";
    case RefSource::Config:
      return "config";
    case RefSource::Pc:
      return "PC";
  }
  return "?";
}

void beginIo() {
  crossio::begin();
}

}  // namespace crossing

#if defined(PANDA_ROLE_CRUCE)

namespace crossing {

// ---------------------------------------------------------------------------
// Estado por tren
// ---------------------------------------------------------------------------
struct Train {
  bool used;
  uint16_t id;
  int64_t lastAuthUs;       // Último beacon autenticado, de cualquier tipo
  int64_t lastValidUs;      // Último beacon válido con fix
  BeaconData last;          // Contenido de ese beacon
  bool haveValid;
  bool heardWithoutPos;     // Lo último autenticado fue válido pero sin fix
  bool heardUnverified;     // Lo último autenticado no se pudo verificar en tiempo
  bool newValid;            // Llegó un beacon válido que todavía no se registró

  TrainPhase phase;
  bool alerting;            // Este tren sostiene el NO SEGURO
  CrossReason alertReason;
  int64_t clearSinceUs;     // Desde cuándo cumple "fuera de peligro" (0 = no)
  int64_t silentSinceUs;    // Desde cuándo está en SIN DATOS (0 = no)
  bool trackSeenOccupied;   // La vía se ocupó durante el silencio

  float distM;
  float speedMps;
  float closingMps;
  float etaCvS;
  float etaMinS;
  int32_t ageMs;

  // Detección de paso: mínimo de distancia en una aproximación
  bool approachSeen;
  bool passageLogged;
  float minDistM;
};

static constexpr size_t kMaxTrains = 4;
static constexpr double kMaxProjectionS = 2.0;
static constexpr int64_t kStartupUs = 3000000;
static constexpr uint32_t kMinRefSamples = 50;     // 5 s de fix a 10 Hz
static constexpr uint32_t kMinSaveSamples = 100;   // 10 s de fix
static constexpr uint32_t kMaxRefHAccMm = 5000;

static Train s_trains[kMaxTrains];
static QueueHandle_t s_statusQ = nullptr;

// Pedidos desde otras tareas.
static std::atomic<bool> s_reqSaveRef{false};
static std::atomic<bool> s_reqClearRef{false};
static std::atomic<bool> s_simTrack{false};
static std::atomic<bool> s_muted{false};
static portMUX_TYPE s_pcRefMux = portMUX_INITIALIZER_UNLOCKED;
static int32_t s_pcRefLat = 0;
static int32_t s_pcRefLon = 0;
static bool s_pcRefSet = false;

void requestSaveRef() {
  s_reqSaveRef.store(true);
}

void requestClearRef() {
  s_reqClearRef.store(true);
}

void setPcRef(int32_t latE7, int32_t lonE7) {
  if (!cfg::sim::kAllowInjection) {
    return;
  }
  portENTER_CRITICAL(&s_pcRefMux);
  s_pcRefLat = latE7;
  s_pcRefLon = lonE7;
  s_pcRefSet = true;
  portEXIT_CRITICAL(&s_pcRefMux);
}

void toggleSimulatedTrack() {
  const bool v = !s_simTrack.load();
  s_simTrack.store(v);
  Serial.printf("[CRUCE] Circuito de vía simulado: %s%s\n", v ? "OCUPADA" : "LIBRE",
                cfg::crossing::kTrackCircuitEnabled ? " (ignorado: la entrada física está habilitada)" : "");
}

void toggleMute() {
  const bool v = !s_muted.load();
  s_muted.store(v);
  Serial.printf("[CRUCE] Sonido %s\n", v ? "silenciado" : "activo");
}

bool latest(CrossingStatus& out) {
  return s_statusQ != nullptr && xQueuePeek(s_statusQ, &out, 0) == pdTRUE;
}

// ---------------------------------------------------------------------------
// Referencia del cruce: promedio móvil del GNSS propio
// ---------------------------------------------------------------------------
struct RefAverager {
  int32_t lat[cfg::crossing::kRefAvgWindow];
  int32_t lon[cfg::crossing::kRefAvgWindow];
  uint32_t count = 0;
  uint32_t head = 0;
  int64_t sumLat = 0;
  int64_t sumLon = 0;
  int64_t lastFixUs = 0;

  void add(int32_t la, int32_t lo) {
    if (count == cfg::crossing::kRefAvgWindow) {
      sumLat -= lat[head];
      sumLon -= lon[head];
    } else {
      ++count;
    }
    lat[head] = la;
    lon[head] = lo;
    sumLat += la;
    sumLon += lo;
    head = (head + 1) % cfg::crossing::kRefAvgWindow;
  }
  int32_t meanLat() const { return static_cast<int32_t>(sumLat / static_cast<int64_t>(count)); }
  int32_t meanLon() const { return static_cast<int32_t>(sumLon / static_cast<int64_t>(count)); }
};

// Vive en memoria estática (4,8 KB), no en la pila de la tarea.
static RefAverager s_avg;

// ---------------------------------------------------------------------------
// ETA mínimo con el perfil de tracción máximo
//
// maxDistance(v0, t): distancia máxima que el tren puede recorrer hacia el
// cruce en t segundos partiendo de v0, en tres tramos con forma cerrada:
//   A) a = a0 constante hasta vk:           x = v·t + a0·t²/2
//   B) potencia constante hasta v_max:      v² = v1² + 2·a0·vk·t
//                                           x = (v2³ - v1³) / (3·a0·vk)
//   C) velocidad máxima de la línea:        x = v_max·t
// minEta(c, d): menor t con maxDistance(c, t) >= d, por bisección (30 pasos,
// error menor a 1 ms). Si el tren se aleja (c < 0), primero frena a a0 hasta
// cero y recién ahí vuelve: es físicamente exagerado y por eso seguro.
// ---------------------------------------------------------------------------
static double maxDistance(double v0, double t) {
  const double a0 = cfg::crossing::kMaxAccelMps2;
  const double vk = cfg::crossing::kAccelKneeMps;
  const double vmax = std::max<double>(cfg::crossing::kLineMaxSpeedMps, v0);
  double x = 0.0;
  double v = v0;
  double rem = t;
  if (v < vk) {
    const double tA = std::min(rem, (vk - v) / a0);
    x += v * tA + 0.5 * a0 * tA * tA;
    v += a0 * tA;
    rem -= tA;
    if (rem <= 0.0) return x;
  }
  if (v < vmax) {
    const double k = 2.0 * a0 * vk;
    const double tB = std::min(rem, (vmax * vmax - v * v) / k);
    const double v2 = std::sqrt(v * v + k * tB);
    x += (v2 * v2 * v2 - v * v * v) / (3.0 * a0 * vk);
    v = v2;
    rem -= tB;
    if (rem <= 0.0) return x;
  }
  return x + v * rem;
}

static double minEta(double closing, double d) {
  const double a0 = cfg::crossing::kMaxAccelMps2;
  double tBrake = 0.0;
  if (closing < 0.0) {
    tBrake = -closing / a0;
    d += closing * closing / (2.0 * a0);
    closing = 0.0;
  }
  if (d <= 0.0) return tBrake;
  double lo = 0.0;
  double hi = 600.0;
  if (maxDistance(closing, hi) < d) return tBrake + hi;
  for (int i = 0; i < 30; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (maxDistance(closing, mid) >= d) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return tBrake + hi;
}

// ---------------------------------------------------------------------------
// Cinemática de un tren respecto del cruce
//
// Plano tangente local centrado en el cruce (Norte, Este). A unos km el error
// frente al elipsoide es despreciable comparado con el del GNSS.
// ---------------------------------------------------------------------------
static void computeKinematics(Train& t, int64_t towNowUs, int32_t refLat, int32_t refLon) {
  constexpr double kDegE7ToRad = 1e-7 * M_PI / 180.0;
  constexpr double kR = 6371008.8;

  const int64_t ageUs = towDiffUs(towNowUs, static_cast<int64_t>(t.last.itowMs) * 1000);
  t.ageMs = static_cast<int32_t>(ageUs / 1000);
  // Se proyecta con la antigüedad medida, pero nunca más de 2 s: más allá el
  // dato ya está fuera de la ventana de validez.
  const double ageS = std::min(std::max(ageUs / 1e6, 0.0), kMaxProjectionS);

  const double cosRef = std::cos(refLat * kDegE7ToRad);
  double dN = static_cast<double>(t.last.latE7 - refLat) * kDegE7ToRad * kR;
  double dE = static_cast<double>(t.last.lonE7 - refLon) * kDegE7ToRad * kR * cosRef;

  const double v = t.last.speedCms / 100.0;
  const double h = t.last.headingCdeg / 100.0 * M_PI / 180.0;
  const double vN = v * std::cos(h);
  const double vE = v * std::sin(h);
  dN += vN * ageS;
  dE += vE * ageS;

  const double d = std::hypot(dN, dE);
  // Velocidad de acercamiento: proyección de la velocidad sobre la dirección
  // tren -> cruce. Positiva si se acerca.
  const double closing = d > 1.0 ? -(dN * vN + dE * vE) / d : v;

  t.distM = static_cast<float>(d);
  t.speedMps = static_cast<float>(v);
  t.closingMps = static_cast<float>(closing);
  t.etaCvS = closing > cfg::crossing::kMinClosingMps ? static_cast<float>(d / closing) : NAN;
  t.etaMinS = static_cast<float>(minEta(closing, d));
}

static Train* findTrain(uint16_t id) {
  for (auto& t : s_trains) {
    if (t.used && t.id == id) return &t;
  }
  return nullptr;
}

static Train& addTrain(uint16_t id, int64_t now) {
  Train* slot = nullptr;
  for (auto& t : s_trains) {
    if (!t.used) {
      slot = &t;
      break;
    }
  }
  if (slot == nullptr) {
    // Tabla llena: se reemplaza al que hace más que no se escucha y no alerta.
    for (auto& t : s_trains) {
      if (!t.alerting && (slot == nullptr || t.lastAuthUs < slot->lastAuthUs)) slot = &t;
    }
    if (slot == nullptr) slot = &s_trains[0];
  }
  *slot = Train{};
  slot->used = true;
  slot->id = id;
  slot->lastAuthUs = now;
  slot->minDistM = INFINITY;
  slot->distM = NAN;
  slot->etaCvS = NAN;
  slot->etaMinS = NAN;
  slot->ageMs = INT32_MIN;
  logNote(NoteCode::TrainNew, id);
  Serial.printf("[CRUCE] Tren nuevo %04X\n", id);
  return *slot;
}

// Prioridad del motivo para elegir el tren principal: el más grave primero.
static int severity(const Train& t) {
  if (!t.alerting) return 0;
  switch (t.alertReason) {
    case CrossReason::SinDatos:
      return 5;
    case CrossReason::TrenEnZona:
      return 4;
    case CrossReason::SinPosicion:
    case CrossReason::NoVerificable:
      return 3;
    case CrossReason::TrenAproxima:
      return 2;
    default:
      return 1;
  }
}

static void logDecision(const Train& t, const CrossingStatus& st) {
  LogRecord rec{};
  rec.type = LogType::Decision;
  DecisionLog& d = rec.dec;
  d.tUs = esp_timer_get_time();
  d.distM = t.distM;
  d.speedMps = t.speedMps;
  d.closingMps = t.closingMps;
  d.etaCvS = t.etaCvS;
  d.etaMinS = t.etaMinS;
  d.ageMs = t.ageMs;
  d.trainId = t.id;
  d.state = static_cast<uint8_t>(st.state);
  d.reason = static_cast<uint8_t>(st.reason);
  d.phase = static_cast<uint8_t>(t.phase);
  d.alerting = t.alerting ? 1 : 0;
  d.outputs = static_cast<uint8_t>((st.pandaLibre ? 1 : 0) | (st.pandaOk ? 2 : 0) | (st.pedestrian ? 4 : 0) |
                                   (st.trackOccupied ? 8 : 0) | (st.barrierDown ? 16 : 0));
  logPush(rec);
}

// ---------------------------------------------------------------------------
// Tarea de decisión
// ---------------------------------------------------------------------------
static void decisionTask(void*) {
  // Watchdog de tareas del ESP-IDF con reinicio: si esta tarea no vuelve a
  // pasar por esp_task_wdt_reset() en 5 s, el micro se reinicia. Durante el
  // reinicio los pines quedan sin manejar y los pull-down externos llevan
  // todas las salidas al estado seguro.
  esp_task_wdt_init(5, true);
  esp_task_wdt_add(nullptr);

  CrossingStatus st{};
  st.state = CrossState::Iniciando;
  st.reason = CrossReason::Arranque;
  st.trackEnabled = cfg::crossing::kTrackCircuitEnabled;

  int32_t savedLat = 0;
  int32_t savedLon = 0;
  bool haveSaved = persist::loadCrossingRef(savedLat, savedLon);
  if (haveSaved) {
    Serial.printf("[CRUCE] Referencia guardada: %.7f, %.7f\n", savedLat / 1e7, savedLon / 1e7);
  }

  const int64_t bootUs = esp_timer_get_time();
  int64_t lastTimeUs = bootUs;
  st.stateSinceUs = bootUs;

  // Circuito de vía con antirrebote por muestras consecutivas iguales.
  const uint32_t debounceSamples = std::max<uint32_t>(1, cfg::crossing::kTrackDebounceMs / cfg::crossing::kPeriodMs) + 1;
  bool trackRawPrev = false;
  uint32_t trackStable = 0;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(cfg::crossing::kPeriodMs));
    esp_task_wdt_reset();
    const int64_t now = esp_timer_get_time();

    // --- 1. Beacons autenticados que dejó el enlace -------------------------
    radiolink::AuthBeacon ab;
    QueueHandle_t q = radiolink::authBeaconQueue();
    while (q != nullptr && xQueueReceive(q, &ab, 0) == pdTRUE) {
      Train* t = findTrain(ab.data.nodeId);
      if (t == nullptr) t = &addTrain(ab.data.nodeId, ab.tEndUs);
      t->lastAuthUs = ab.tEndUs;
      const bool fix = (ab.data.flags & BeaconFlag::kFixOk) != 0;
      if (ab.result == static_cast<uint8_t>(RxResult::Ok) && fix) {
        t->last = ab.data;
        t->lastValidUs = ab.tEndUs;
        t->haveValid = true;
        t->heardWithoutPos = false;
        t->heardUnverified = false;
        t->newValid = true;
      } else if (ab.result == static_cast<uint8_t>(RxResult::Ok)) {
        t->heardWithoutPos = true;
      } else {
        t->heardUnverified = true;
      }
    }

    // --- 2. Tiempo propio y salud del nodo ----------------------------------
    int64_t towNow = 0;
    const TimeQuality tq = gpsTowAt(now, towNow);
    const bool ownTimeOk = tq != TimeQuality::None;
    if (ownTimeOk) lastTimeUs = now;
    const bool timeLost = (now - lastTimeUs) > static_cast<int64_t>(cfg::crossing::kOwnTimeLossMs) * 1000 ||
                          (!ownTimeOk && lastTimeUs == bootUs);
    const bool linkOk = g_stats.radioOnline.load() && (millis() - g_stats.linkAliveMs.load()) < 1000;

    // --- 3. Referencia del cruce --------------------------------------------
    GnssFix own{};
    if (xQueuePeek(g_latestFix, &own, 0) == pdTRUE && own.fixOk && own.hAccMm <= kMaxRefHAccMm &&
        own.tRxUs != s_avg.lastFixUs) {
      s_avg.add(own.latE7, own.lonE7);
      s_avg.lastFixUs = own.tRxUs;
    }
    if (s_reqClearRef.exchange(false)) {
      persist::clearCrossingRef();
      haveSaved = false;
      Serial.println("[CRUCE] Referencia guardada borrada");
    }
    if (s_reqSaveRef.exchange(false)) {
      if (s_avg.count >= kMinSaveSamples) {
        savedLat = s_avg.meanLat();
        savedLon = s_avg.meanLon();
        persist::saveCrossingRef(savedLat, savedLon);
        haveSaved = true;
        logNote(NoteCode::RefSaved, s_avg.count);
        Serial.printf("[CRUCE] Referencia guardada: %.7f, %.7f (promedio de %lu fixes)\n", savedLat / 1e7,
                      savedLon / 1e7, static_cast<unsigned long>(s_avg.count));
      } else {
        Serial.printf("[CRUCE] Hacen falta %lu fixes buenos para guardar (hay %lu)\n",
                      static_cast<unsigned long>(kMinSaveSamples), static_cast<unsigned long>(s_avg.count));
      }
    }
    bool pcRef;
    int32_t pcLat;
    int32_t pcLon;
    portENTER_CRITICAL(&s_pcRefMux);
    pcRef = s_pcRefSet;
    pcLat = s_pcRefLat;
    pcLon = s_pcRefLon;
    portEXIT_CRITICAL(&s_pcRefMux);

    if (pcRef) {
      st.refSource = RefSource::Pc;
      st.refLatE7 = pcLat;
      st.refLonE7 = pcLon;
    } else if (haveSaved) {
      st.refSource = RefSource::Guardada;
      st.refLatE7 = savedLat;
      st.refLonE7 = savedLon;
    } else if (cfg::crossing::kFixedRefLatE7 != 0 || cfg::crossing::kFixedRefLonE7 != 0) {
      st.refSource = RefSource::Config;
      st.refLatE7 = cfg::crossing::kFixedRefLatE7;
      st.refLonE7 = cfg::crossing::kFixedRefLonE7;
    } else if (s_avg.count >= kMinRefSamples) {
      st.refSource = RefSource::Gnss;
      st.refLatE7 = s_avg.meanLat();
      st.refLonE7 = s_avg.meanLon();
    } else {
      st.refSource = RefSource::Ninguna;
    }
    st.refSamples = s_avg.count;
    const bool haveRef = st.refSource != RefSource::Ninguna;

    // --- 4. Circuito de vía -------------------------------------------------
    const bool trackRaw = cfg::crossing::kTrackCircuitEnabled ? crossio::readTrackOccupied() : s_simTrack.load();
    trackStable = (trackRaw == trackRawPrev) ? trackStable + 1 : 0;
    trackRawPrev = trackRaw;
    const bool trackWas = st.trackOccupied;
    if (trackStable >= debounceSamples && trackRaw != st.trackOccupied) {
      st.trackOccupied = trackRaw;
      logNote(NoteCode::TrackChange, trackRaw ? 1 : 0);
      Serial.printf("[CRUCE] Circuito de vía: %s\n", trackRaw ? "OCUPADA" : "LIBRE");
    }
    const bool trackRose = !trackWas && st.trackOccupied;
    const bool trackFell = trackWas && !st.trackOccupied;
    if (trackRose && st.pandaOk && st.pandaLibre) {
      // La vía detectó un tren mientras PANDA decía vía libre: o el tren no
      // tiene nodo, o PANDA no lo vio. Es la métrica clave de validación.
      ++st.trackWithoutPanda;
      logNote(NoteCode::TrackWithoutPanda, st.haveTrain ? st.trainId : 0);
      Serial.println("[CRUCE] ATENCION: vía ocupada con PANDA en vía libre");
    }

    // --- 5. Evaluación de cada tren -----------------------------------------
    for (auto& t : s_trains) {
      if (!t.used) continue;
      const int64_t sinceValid = t.haveValid ? now - t.lastValidUs : INT64_MAX;
      const int64_t sinceAuth = now - t.lastAuthUs;
      const int64_t linkTimeoutUs = static_cast<int64_t>(cfg::beacon::kLinkTimeoutMs) * 1000;
      const bool fresh = sinceValid <= linkTimeoutUs;

      if (t.silentSinceUs != 0 && st.trackOccupied) t.trackSeenOccupied = true;

      if (fresh && ownTimeOk && haveRef) {
        computeKinematics(t, towNow, st.refLatE7, st.refLonE7);
        t.silentSinceUs = 0;
        t.trackSeenOccupied = false;

        const bool relevant = t.distM <= cfg::crossing::kRelevantRadiusM;
        const bool inZone = t.distM <= cfg::crossing::kOccupiedRadiusM;
        const bool danger = relevant && (inZone || t.etaMinS <= cfg::crossing::kAlertEtaS);

        if (inZone) {
          t.phase = TrainPhase::EnZona;
        } else if (danger) {
          t.phase = TrainPhase::Aproxima;
        } else if (t.closingMps < -cfg::crossing::kMinClosingMps) {
          t.phase = TrainPhase::Alejandose;
        } else {
          t.phase = TrainPhase::Lejos;
        }

        if (danger) {
          t.alerting = true;
          t.alertReason = inZone ? CrossReason::TrenEnZona : CrossReason::TrenAproxima;
          t.clearSinceUs = 0;
        } else if (t.alerting) {
          // Fuera de peligro: se apaga recién después de kClearHoldMs seguidos.
          if (t.clearSinceUs == 0) t.clearSinceUs = now;
          if (now - t.clearSinceUs >= static_cast<int64_t>(cfg::crossing::kClearHoldMs) * 1000) {
            t.alerting = false;
            t.clearSinceUs = 0;
          }
        }

        // Paso: la distancia pasa por un mínimo y el tren empieza a alejarse.
        if (t.closingMps > cfg::crossing::kMinClosingMps && relevant) t.approachSeen = true;
        if (t.approachSeen) {
          t.minDistM = std::min(t.minDistM, t.distM);
          if (!t.passageLogged && t.closingMps < -cfg::crossing::kMinClosingMps && t.distM > t.minDistM + 5.0f) {
            t.passageLogged = true;
            ++st.passages;
            logNote(NoteCode::Passage,
                    (static_cast<uint32_t>(t.id) << 16) | static_cast<uint32_t>(std::min(t.minDistM, 65535.0f)));
            Serial.printf("[CRUCE] PASO del tren %04X, distancia mínima %.0f m\n", t.id,
                          static_cast<double>(t.minDistM));
          }
        }
        if (!t.alerting && !inZone && t.phase != TrainPhase::Aproxima && t.passageLogged) {
          t.approachSeen = false;
          t.passageLogged = false;
          t.minDistM = INFINITY;
        }
        continue;
      }

      // Se lo escucha, pero no se puede saber dónde está o si el dato es
      // fresco: NO SEGURO. Va antes que el caso "se calló" porque el tren no
      // está callado: el reloj de liberación por silencio no debe correr.
      if (sinceAuth <= linkTimeoutUs && (t.heardWithoutPos || t.heardUnverified)) {
        t.alerting = true;
        t.alertReason = t.heardWithoutPos ? CrossReason::SinPosicion : CrossReason::NoVerificable;
        t.phase = TrainPhase::SinDatos;
        t.silentSinceUs = 0;
        t.clearSinceUs = 0;
        continue;
      }

      // Sin dato válido reciente de este tren. La pregunta es si durante el
      // silencio pudo haber entrado en la zona de alerta: su ETA mínimo del
      // último dato, descontado el tiempo en silencio, cae bajo el umbral.
      const float silentS = t.haveValid ? static_cast<float>(sinceValid / 1e6) : 0.0f;
      const bool couldBeNear = t.haveValid && !std::isnan(t.etaMinS) &&
                               t.distM <= cfg::crossing::kRelevantRadiusM &&
                               (t.etaMinS - silentS) <= cfg::crossing::kAlertEtaS;
      if (t.alerting || couldBeNear) {
        // Estaba en peligro (o no se lo podía ubicar) y se calló: SIN DATOS.
        // Sin datos nunca es vía libre (E-9).
        if (t.silentSinceUs == 0) {
          t.silentSinceUs = now;
          t.trackSeenOccupied = st.trackOccupied;
          Serial.printf("[CRUCE] SIN DATOS del tren %04X\n", t.id);
        }
        t.alerting = true;
        t.alertReason = CrossReason::SinDatos;
        t.phase = TrainPhase::SinDatos;

        // Liberación: por el circuito de vía (se ocupó y se liberó durante el
        // silencio: el tren pasó) o por tiempo, como último recurso.
        const bool byTrack = t.trackSeenOccupied && trackFell;
        const bool byTime = (now - t.silentSinceUs) > static_cast<int64_t>(cfg::crossing::kSilentReleaseMs) * 1000;
        if (byTrack || byTime) {
          ++st.silentReleases;
          logNote(NoteCode::SilentRelease, (static_cast<uint32_t>(t.id) << 8) | (byTrack ? 2u : 1u));
          Serial.printf("[CRUCE] Tren %04X liberado %s\n", t.id, byTrack ? "por el circuito de vía" : "por tiempo");
          logNote(NoteCode::TrainForgotten, t.id);
          t = Train{};
        }
        continue;
      }

      // Un tren que se alejaba (o que nunca dio posición, o que está fuera del
      // radio relevante) y dejó de escucharse: es lo normal cuando sale del
      // alcance. Se olvida después de kForgetMs. Uno detenido o que se acercaba
      // lejos se sigue: si calla lo suficiente, couldBeNear lo pasa a SIN DATOS.
      const bool wasReceding = t.closingMps < -cfg::crossing::kMinClosingMps;
      const bool farAway = !(t.distM <= cfg::crossing::kRelevantRadiusM);
      if ((!t.haveValid || wasReceding || farAway) &&
          sinceAuth > static_cast<int64_t>(cfg::crossing::kForgetMs) * 1000) {
        logNote(NoteCode::TrainForgotten, t.id);
        Serial.printf("[CRUCE] Tren %04X fuera de alcance\n", t.id);
        t = Train{};
      }
    }

    // --- 6. Estado del cruce: cierre en OR, apertura en AND ------------------
    const Train* principal = nullptr;
    uint8_t count = 0;
    bool anyAlert = false;
    for (const auto& t : s_trains) {
      if (!t.used) continue;
      ++count;
      anyAlert |= t.alerting;
      if (principal == nullptr || severity(t) > severity(*principal) ||
          (severity(t) == severity(*principal) && t.distM < principal->distM)) {
        principal = &t;
      }
    }

    CrossState state;
    CrossReason reason = CrossReason::Ninguno;
    uint16_t reasonTrain = 0;
    if (now - bootUs < kStartupUs) {
      state = CrossState::Iniciando;
      reason = CrossReason::Arranque;
    } else if (!linkOk) {
      state = CrossState::Falla;
      reason = CrossReason::FallaRadio;
    } else if (timeLost) {
      state = CrossState::Falla;
      reason = CrossReason::FallaTiempo;
    } else if (!haveRef) {
      state = CrossState::Falla;
      reason = CrossReason::FallaReferencia;
    } else if (anyAlert) {
      state = CrossState::NoSeguro;
      reason = principal->alertReason;
      reasonTrain = principal->id;
    } else if (st.trackOccupied) {
      state = CrossState::NoSeguro;
      reason = CrossReason::ViaOcupada;
    } else {
      state = CrossState::Apagado;
    }

    st.pandaOk = (state == CrossState::Apagado || state == CrossState::NoSeguro);
    st.pandaLibre = st.pandaOk && !anyAlert;
    st.pedestrian = (state != CrossState::Apagado);
    st.muted = s_muted.load();
    // Controlador de barrera de referencia. En el producto vive en el
    // controlador de la barrera, no en PANDA. Acá se calcula para mostrar y
    // registrar que el invariante se cumple:
    //   PANDA operativo:    baja si PANDA pide cierre O la vía está ocupada
    //   PANDA no operativo: baja si la vía está ocupada (comportamiento actual)
    st.barrierDown = st.pandaOk ? (!st.pandaLibre || st.trackOccupied) : st.trackOccupied;

    crossio::apply({st.pandaLibre, st.pandaOk, st.pedestrian, state == CrossState::NoSeguro && !st.muted});

    const bool changed = (state != st.state) || (reason != st.reason) || (reasonTrain != st.reasonTrain);
    st.state = state;
    st.reason = reason;
    st.reasonTrain = reasonTrain;
    if (changed) {
      st.stateSinceUs = now;
      logNote(NoteCode::StateChange, (static_cast<uint32_t>(state) << 8) | static_cast<uint32_t>(reason));
      Serial.printf("[CRUCE] >>> %s: %s", stateName(state), reasonName(reason));
      if (reasonTrain != 0) Serial.printf(" (tren %04X)", reasonTrain);
      Serial.println();
    }

    st.trainCount = count;
    st.haveTrain = principal != nullptr;
    if (principal != nullptr) {
      st.trainId = principal->id;
      st.phase = principal->phase;
      st.distM = principal->distM;
      st.speedMps = principal->speedMps;
      st.closingMps = principal->closingMps;
      st.etaCvS = principal->etaCvS;
      st.etaMinS = principal->etaMinS;
      st.ageMs = principal->ageMs;
      st.silentMs = principal->haveValid ? static_cast<uint32_t>((now - principal->lastValidUs) / 1000) : 0;
    }
    st.watchdogTrips = crossio::watchdogTrips();

    // --- 7. Registro -------------------------------------------------------
    for (auto& t : s_trains) {
      if (t.used && (t.newValid || (changed && &t == principal))) {
        logDecision(t, st);
        t.newValid = false;
      }
    }

    xQueueOverwrite(s_statusQ, &st);
  }
}

void startTask() {
  s_statusQ = xQueueCreate(1, sizeof(CrossingStatus));
  xTaskCreatePinnedToCore(decisionTask, "decision", cfg::task::kStackDecision, nullptr, cfg::task::kPrioDecision,
                          nullptr, cfg::task::kCoreSafety);
}

}  // namespace crossing

#else  // Otros roles: la lógica del cruce no existe, solo las funciones vacías

namespace crossing {
void startTask() {}
bool latest(CrossingStatus&) {
  return false;
}
void requestSaveRef() {}
void requestClearRef() {}
void setPcRef(int32_t, int32_t) {}
void toggleSimulatedTrack() {}
void toggleMute() {}
}  // namespace crossing

#endif  // PANDA_ROLE_CRUCE
