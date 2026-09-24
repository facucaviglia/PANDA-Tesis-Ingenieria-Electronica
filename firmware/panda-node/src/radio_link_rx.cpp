// =============================================================================
// Tarea de enlace del NODO CRUCE: recepción continua y validación de beacons.
//
// Cadena de validación de cada paquete, en este orden (lo barato primero y
// nada se descifra sin autenticar):
//   1. CRC de la radio            falla -> CrcError
//   2. Versión y largo            falla -> BadFormat
//   3. CMAC                       falla -> BadTag (no viene de un nodo con clave)
//   4. Contador > último del tren falla -> Replay
//   5. Tiempo GPS en las 2 puntas falta -> Unverified (solo métrica de enlace)
//   6. Antigüedad en [-0,5 s, 2 s] falla -> Stale o Future
//   7. Todo bien                        -> Ok, es dato válido
// Los beacons que pasan el CMAC y el contador se le entregan a la tarea de
// decisión (crossing), que es la que resuelve el estado del cruce.
// Complejidad: O(1) por paquete más O(N) en la tabla de trenes (N = 4).
// =============================================================================
#if defined(PANDA_ROLE_CRUCE)

#include <Arduino.h>
#include <RadioLib.h>
#include <algorithm>
#include <climits>
#include <cmath>
#include <esp_timer.h>

#include "beacon.h"
#include "geo.h"
#include "gnss_manager.h"
#include "radio_link_internal.h"
#include "radio_manager.h"
#include "system_state.h"

namespace radiolink {

// Seguimiento por tren, para el contador anti-repetición y la tasa de pérdida.
struct TrainTrack {
  bool used;
  uint16_t nodeId;
  uint32_t lastCounter;
  int64_t lastSeenUs;
  uint32_t winRx;
  uint32_t winLost;
};

static constexpr size_t kMaxTrains = 4;
static constexpr uint32_t kGapResync = 1000;
static constexpr int64_t kPerWindowUs = 5000000;
static TrainTrack s_trains[kMaxTrains];
static RxStatus s_status;

// Busca el tren o le asigna un lugar (reemplaza al que hace más que no se ve).
static TrainTrack& trackFor(uint16_t nodeId) {
  TrainTrack* oldest = &s_trains[0];
  for (auto& t : s_trains) {
    if (t.used && t.nodeId == nodeId) {
      return t;
    }
    if (!t.used || t.lastSeenUs < oldest->lastSeenUs) {
      oldest = &t;
    }
  }
  *oldest = TrainTrack{true, nodeId, 0, 0, 0, 0};
  return *oldest;
}

// Valida un paquete (de radio o inyectado), lo registra, actualiza el estado
// del enlace y, si está autenticado, se lo pasa a la lógica de decisión.
static void processPacket(BeaconCodec& codec, const uint8_t* buf, int16_t radioSt, const PacketInfo& info,
                          int64_t tEnd, bool injected) {
  RxLog r{};
  r.tEndUs = tEnd;
  r.rssiDbm = info.rssiDbm;
  r.snrDb = info.snrDb;
  r.freqErrHz = info.freqErrHz;
  r.ageMs = INT32_MIN;
  r.distM = NAN;
  r.profile = activeProfileIndex();
  r.injected = injected ? 1 : 0;
  int64_t towRx = 0;
  const TimeQuality q = gpsTowAt(tEnd, towRx);
  r.timeQ = static_cast<uint8_t>(q);

  RxResult result;
  BeaconData b{};
  bool authenticated = false;
  if (radioSt == RADIOLIB_ERR_CRC_MISMATCH) {
    result = RxResult::CrcError;
  } else if (radioSt != RADIOLIB_ERR_NONE) {
    result = RxResult::BadFormat;
  } else {
    const BeaconOpen o = codec.open(buf, cfg::radio::kBeaconLength, b);
    if (o == BeaconOpen::BadTag) {
      result = RxResult::BadTag;
    } else if (o != BeaconOpen::Ok) {
      result = RxResult::BadFormat;
    } else {
      r.nodeId = b.nodeId;
      r.counter = b.counter;
      r.itowMs = b.itowMs;
      r.latE7 = b.latE7;
      r.lonE7 = b.lonE7;
      r.speedCms = b.speedCms;
      r.headingCdeg = b.headingCdeg;
      r.hAccCm = b.hAccCm;
      r.numSv = b.numSv;
      r.flags = b.flags;

      TrainTrack& t = trackFor(b.nodeId);
      if (t.lastCounter != 0 && b.counter <= t.lastCounter) {
        result = RxResult::Replay;
      } else {
        uint32_t gap = (t.lastCounter != 0) ? (b.counter - t.lastCounter - 1) : 0;
        if (gap > kGapResync) {
          gap = 0;  // El tren se reinició y reservó otro bloque de contador
        }
        t.lastCounter = b.counter;
        t.lastSeenUs = tEnd;
        t.winRx += 1;
        t.winLost += gap;
        r.gap = static_cast<uint16_t>(std::min<uint32_t>(gap, 65535));
        authenticated = true;

        // Frescura: solo se puede verificar con tiempo GPS en las dos puntas.
        if (q != TimeQuality::None && (b.flags & BeaconFlag::kTimeSync)) {
          const int64_t ageUs = towDiffUs(towRx, static_cast<int64_t>(b.itowMs) * 1000);
          r.ageMs = static_cast<int32_t>(ageUs / 1000);
          if (r.ageMs > cfg::beacon::kMaxAgeMs) {
            result = RxResult::Stale;
          } else if (r.ageMs < cfg::beacon::kMinAgeMs) {
            result = RxResult::Future;
          } else {
            result = RxResult::Ok;
          }
        } else {
          result = RxResult::Unverified;
        }

        // Distancia entre nodos (métrica de enlace), si los dos tienen fix.
        GnssFix own{};
        if ((b.flags & BeaconFlag::kFixOk) && xQueuePeek(g_latestFix, &own, 0) == pdTRUE && own.fixOk &&
            (tEnd - own.tRxUs) < 2000000) {
          r.distM = static_cast<float>(geo::haversineM(own.latE7, own.lonE7, b.latE7, b.lonE7));
        }
      }
    }
  }
  r.result = static_cast<uint8_t>(result);

  LogRecord rec{};
  rec.type = LogType::Rx;
  rec.rx = r;
  logPush(rec);

  if (authenticated) {
    AuthBeacon ab{};
    ab.data = b;
    ab.tEndUs = tEnd;
    ab.ageMs = (result == RxResult::Unverified) ? INT32_MIN : r.ageMs;
    ab.result = r.result;
    ab.injected = injected;
    xQueueSend(g_authBeaconQ, &ab, 0);
  }

  // Estado publicado para pantalla y telemetría.
  s_status.lastResult = r.result;
  s_status.timeQ = r.timeQ;
  s_status.rssiDbm = r.rssiDbm;
  s_status.snrDb = r.snrDb;
  s_status.freqErrHz = r.freqErrHz;
  switch (result) {
    case RxResult::Ok:
      ++s_status.ok;
      break;
    case RxResult::Unverified:
      ++s_status.unverified;
      break;
    case RxResult::CrcError:
      ++s_status.crcError;
      break;
    case RxResult::BadTag:
      ++s_status.badTag;
      break;
    case RxResult::Replay:
      ++s_status.replay;
      break;
    case RxResult::Stale:
    case RxResult::Future:
      ++s_status.stale;
      break;
    default:
      ++s_status.other;
      break;
  }
  if (authenticated) {
    s_status.haveTrain = true;
    s_status.nodeId = b.nodeId;
    s_status.last = b;
    s_status.ageMs = r.ageMs;
    s_status.distM = r.distM;
    s_status.lastAuthUs = tEnd;
    if (result == RxResult::Ok) {
      s_status.lastValidUs = tEnd;
    }
  }
}

static void rxTask(void*) {
  BeaconCodec codec(cfg::beacon::kEncKey, cfg::beacon::kMacKey);
  g_radio.setIrqTask(xTaskGetCurrentTaskHandle());
  g_radio.startReceive();
  Serial.printf("[CRUCE] Escuchando en %.1f MHz, %s\n", static_cast<double>(cfg::radio::kFrequencyMHz),
                activeProfile().name);

  s_status = RxStatus{};
  s_status.ageMs = INT32_MIN;
  s_status.distM = NAN;
  int64_t perWindowStart = esp_timer_get_time();

  for (;;) {
    // Se borran todos los bits al salir. Si vence el tiempo sin aviso, no hay
    // nada que atender (y no se mira el valor, que podría no estar limpio).
    uint32_t bits = 0;
    if (xTaskNotifyWait(0, ULONG_MAX, &bits, pdMS_TO_TICKS(100)) != pdTRUE) {
      bits = 0;
    }

    if (bits & kReqMask) {
      handleRequests(bits);
      // Durante el barrido pudo entrar alguna IRQ de otro canal: se descarta.
      ulTaskNotifyValueClear(nullptr, kNotifyRadioIrq);
      g_radio.startReceive();
      bits &= ~kNotifyRadioIrq;
    }

    if (bits & kNotifyRadioIrq) {
      const int64_t tEnd = g_radio.lastIrqUs();
      uint8_t buf[cfg::radio::kBeaconLength];
      PacketInfo info{};
      const int16_t radioSt = g_radio.readPacket(buf, sizeof(buf), info);
      processPacket(codec, buf, radioSt, info, tEnd, false);
    }

    // Paquetes del simulador de la PC: mismo camino, marcados como inyectados.
    InjectedPacket inj;
    while (g_injectQ != nullptr && xQueueReceive(g_injectQ, &inj, 0) == pdTRUE) {
      const PacketInfo info{inj.rssiDbm, inj.snrDb, 0.0f};
      processPacket(codec, inj.data, RADIOLIB_ERR_NONE, info, esp_timer_get_time(), true);
    }

    // Tasa de pérdida del tren principal en ventanas de 5 s.
    const int64_t now = esp_timer_get_time();
    if (now - perWindowStart >= kPerWindowUs) {
      perWindowStart = now;
      if (s_status.haveTrain) {
        TrainTrack& t = trackFor(s_status.nodeId);
        const uint32_t expected = t.winRx + t.winLost;
        // Si no llegó nada en toda la ventana, la pérdida es total.
        s_status.perPct = expected > 0 ? 100.0f * t.winLost / expected : 100.0f;
        t.winRx = 0;
        t.winLost = 0;
      }
    }

    xQueueOverwrite(g_rxStatusQ, &s_status);
    // Señal de vida para la lógica de decisión: si la tarea de enlace se
    // cuelga, el cruce pasa a FALLA en menos de 1 s.
    g_stats.linkAliveMs.store(millis());
  }
}

void startTask() {
  xTaskCreatePinnedToCore(rxTask, "link-rx", cfg::task::kStackRadio, nullptr, cfg::task::kPrioRadio, &g_linkTask,
                          cfg::task::kCoreSafety);
}

}  // namespace radiolink

#endif  // PANDA_ROLE_CRUCE
