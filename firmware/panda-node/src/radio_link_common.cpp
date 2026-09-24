#include <Arduino.h>
#include <algorithm>
#include <cstring>

#include "persist.h"
#include "radio_link_internal.h"
#include "radio_manager.h"
#include "system_state.h"

namespace radiolink {

TaskHandle_t g_linkTask = nullptr;
QueueHandle_t g_txStatusQ = nullptr;
QueueHandle_t g_rxStatusQ = nullptr;
QueueHandle_t g_authBeaconQ = nullptr;
QueueHandle_t g_injectQ = nullptr;

QueueHandle_t authBeaconQueue() {
  return g_authBeaconQ;
}

bool injectPacket(const uint8_t* pkt, size_t len, float rssiDbm, float snrDb) {
  if (!cfg::sim::kAllowInjection || g_injectQ == nullptr || len != sizeof(InjectedPacket::data)) {
    return false;
  }
  InjectedPacket p{};
  memcpy(p.data, pkt, len);
  p.rssiDbm = rssiDbm;
  p.snrDb = snrDb;
  if (xQueueSend(g_injectQ, &p, 0) != pdTRUE) {
    return false;
  }
  if (g_linkTask != nullptr) {
    xTaskNotify(g_linkTask, kNotifyInject, eSetBits);
  }
  return true;
}

static uint8_t s_profile = cfg::radio::kDefaultProfile;
static bool s_fieldPower = false;

const cfg::radio::LoraProfile& activeProfile() {
  return cfg::radio::kProfiles[s_profile];
}

uint8_t activeProfileIndex() {
  return s_profile;
}

int8_t activePowerDbm() {
  return s_fieldPower ? cfg::radio::kTxPowerFieldDbm : cfg::radio::kTxPowerBenchDbm;
}

bool begin() {
  s_profile = persist::loadProfile();
  s_fieldPower = persist::loadFieldPower();
  g_txStatusQ = xQueueCreate(1, sizeof(TxStatus));
  g_rxStatusQ = xQueueCreate(1, sizeof(RxStatus));
#if defined(PANDA_ROLE_CRUCE)
  g_authBeaconQ = xQueueCreate(32, sizeof(AuthBeacon));
  g_injectQ = xQueueCreate(8, sizeof(InjectedPacket));
#endif

  g_stats.profile.store(s_profile);
  g_stats.txPowerDbm.store(activePowerDbm());
  return g_radio.begin(activeProfile(), activePowerDbm()) == 0;
}

static void notify(uint32_t bits) {
  if (g_linkTask != nullptr) {
    xTaskNotify(g_linkTask, bits, eSetBits);
  }
}

void requestNextProfile() {
  notify(kReqProfile);
}

void requestTogglePower() {
  notify(kReqPower);
}

void requestScan() {
  notify(kReqScan);
}

bool latestTx(TxStatus& out) {
  return g_txStatusQ != nullptr && xQueuePeek(g_txStatusQ, &out, 0) == pdTRUE;
}

bool latestRx(RxStatus& out) {
  return g_rxStatusQ != nullptr && xQueuePeek(g_rxStatusQ, &out, 0) == pdTRUE;
}

const char* rxResultName(uint8_t result) {
  switch (static_cast<RxResult>(result)) {
    case RxResult::Ok:
      return "OK";
    case RxResult::Unverified:
      return "SIN_TIEMPO";
    case RxResult::CrcError:
      return "CRC";
    case RxResult::BadFormat:
      return "FORMATO";
    case RxResult::BadTag:
      return "CMAC";
    case RxResult::Replay:
      return "REPETIDO";
    case RxResult::Stale:
      return "VIEJO";
    case RxResult::Future:
      return "FUTURO";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// Barrido de canales
//
// Mide el RSSI instantáneo en cada canal candidato de 500 kHz, con la radio en
// recepción, durante kScanDwellMs. Reporta el promedio (piso de ruido) y el
// máximo (ráfagas de otros equipos). El mejor canal es el de menor promedio.
// Bloquea la tarea de enlace unos 3 s: el tren deja de transmitir y el cruce
// de escuchar mientras dura. Es una herramienta de relevamiento del lugar.
// ---------------------------------------------------------------------------
static void runScan() {
  Serial.println();
  Serial.println("[SCAN] Barrido de canales de 500 kHz (piso de ruido, menor es mejor)");
  float bestAvg = 0.0f;
  float bestMHz = cfg::radio::kFrequencyMHz;

  for (uint8_t ch = 0; ch < cfg::radio::kScanChannels; ++ch) {
    const float mhz = cfg::radio::kScanFirstMHz + ch * cfg::radio::kScanStepMHz;
    if (g_radio.setFrequency(mhz) != 0) {
      continue;
    }
    g_radio.startReceive();
    vTaskDelay(pdMS_TO_TICKS(5));

    float sum = 0.0f;
    float peak = -200.0f;
    uint32_t n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < cfg::radio::kScanDwellMs) {
      const float r = g_radio.channelRssi();
      sum += r;
      peak = std::max(peak, r);
      ++n;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    const float avg = n > 0 ? sum / n : 0.0f;
    if (ch == 0 || avg < bestAvg) {
      bestAvg = avg;
      bestMHz = mhz;
    }

    // Barra proporcional al piso de ruido, de -130 dBm (vacía) a -80 dBm.
    char bar[26];
    const int len = std::max(0, std::min(25, static_cast<int>((avg + 130.0f) / 2.0f)));
    memset(bar, '#', len);
    bar[len] = '\0';
    Serial.printf("[SCAN] %6.1f MHz  prom %7.1f dBm  max %7.1f dBm  %s%s\n", static_cast<double>(mhz),
                  static_cast<double>(avg), static_cast<double>(peak), bar,
                  mhz == cfg::radio::kFrequencyMHz ? "  <- actual" : "");
  }

  g_radio.standbyXosc();
  g_radio.setFrequency(cfg::radio::kFrequencyMHz);
  Serial.printf("[SCAN] Canal más silencioso: %.1f MHz (%.1f dBm). Para usarlo, cambiar kFrequencyMHz en config.h\n",
                static_cast<double>(bestMHz), static_cast<double>(bestAvg));
  Serial.println("[SCAN] en los DOS nodos y volver a compilar.");
}

bool handleRequests(uint32_t bits) {
  bool changed = false;

  if (bits & kReqProfile) {
    s_profile = static_cast<uint8_t>((s_profile + 1) % cfg::radio::kProfileCount);
    const int16_t st = g_radio.applyProfile(activeProfile());
    persist::saveProfile(s_profile);
    g_stats.profile.store(s_profile);
    logNote(NoteCode::ProfileChange, s_profile);
    Serial.printf("[RADIO] Perfil %u: %s (código %d). Cambiarlo también en el otro nodo.\n", s_profile,
                  activeProfile().name, st);
    changed = true;
  }

  if (bits & kReqPower) {
    s_fieldPower = !s_fieldPower;
    const int16_t st = g_radio.setPower(activePowerDbm());
    persist::saveFieldPower(s_fieldPower);
    g_stats.txPowerDbm.store(activePowerDbm());
    logNote(NoteCode::PowerChange, static_cast<uint32_t>(activePowerDbm()));
    Serial.printf("[RADIO] Potencia %s: %d dBm (código %d)\n", s_fieldPower ? "CAMPO" : "BANCO", activePowerDbm(), st);
  }

  if (bits & kReqScan) {
    runScan();
    changed = true;
  }

  return changed;
}

}  // namespace radiolink
