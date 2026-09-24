#include "telemetry.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <climits>
#include <cmath>
#include <esp_timer.h>

#include "config.h"
#include "crossing.h"
#include "gnss_manager.h"
#include "radio_link.h"
#include "system_state.h"

Telemetry g_telemetry;

namespace {

constexpr double kMpsToKnots = 1.943844;  // OsmAnd espera la velocidad en nudos

// Identificadores de dispositivo en Traccar según lo que se sube.
#if defined(PANDA_ROLE_REGISTRADOR)
constexpr const char* kOwnId = TRACCAR_ID_PREFIX "-reg";
#elif defined(PANDA_ROLE_TREN)
constexpr const char* kOwnId = TRACCAR_ID_PREFIX "-tren";
#else
constexpr const char* kOwnId = TRACCAR_ID_PREFIX "-cruce";
constexpr const char* kTrainViaLoraId = TRACCAR_ID_PREFIX "-tren-lora";
#endif

// Posición a subir, sin importar de dónde vino.
struct Report {
  const char* deviceId;
  int32_t latE7;
  int32_t lonE7;
  double speedMps;
  double headingDeg;
  double altitudeM;
  double accuracyM;
  uint32_t unixS;  // 0 si no se conoce
  char extra[240]; // Atributos adicionales ya formateados ("&clave=valor...")
};

String buildQuery(const Report& r) {
  char buf[480];
  int n = snprintf(buf, sizeof(buf), "/?id=%s&lat=%.7f&lon=%.7f&speed=%.2f&bearing=%.1f&altitude=%.1f&accuracy=%.1f%s",
                   r.deviceId, r.latE7 / 1e7, r.lonE7 / 1e7, r.speedMps * kMpsToKnots, r.headingDeg, r.altitudeM,
                   r.accuracyM, r.extra);
  // Sin fecha válida se omite el timestamp y Traccar usa la hora del servidor.
  if (r.unixS != 0 && n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
    snprintf(buf + n, sizeof(buf) - n, "&timestamp=%lu", static_cast<unsigned long>(r.unixS));
  }
  return String(buf);
}

// Posición propia a partir del último fix del GNSS de esta placa.
bool ownReport(Report& r) {
  GnssFix f{};
  if (xQueuePeek(g_latestFix, &f, 0) != pdTRUE || !f.fixOk) {
    return false;
  }
  if (esp_timer_get_time() - f.tRxUs > static_cast<int64_t>(cfg::telemetry::kMaxFixAgeMs) * 1000) {
    return false;
  }
  r.deviceId = kOwnId;
  r.latE7 = f.latE7;
  r.lonE7 = f.lonE7;
  r.speedMps = f.gSpeedMms / 1000.0;
  r.headingDeg = f.headMotE5 / 1e5;
  r.altitudeM = f.hMslMm / 1000.0;
  r.accuracyM = f.hAccMm / 1000.0;
  r.unixS = f.timeValid ? f.unixS : 0;
  snprintf(r.extra, sizeof(r.extra), "&sats=%u&pdop=%.2f&batt=%ld&quieto=%u&marcas=%lu", f.numSv, f.pDopE2 / 100.0,
           static_cast<long>(g_stats.battPercent.load()), g_stats.imuStill.load() ? 1u : 0u,
           static_cast<unsigned long>(g_stats.marks.load()));
#if defined(PANDA_ROLE_TREN)
  radiolink::TxStatus tx{};
  if (radiolink::latestTx(tx)) {
    const size_t len = strlen(r.extra);
    snprintf(r.extra + len, sizeof(r.extra) - len, "&beacons=%lu&ranura=%u",
             static_cast<unsigned long>(tx.txCount), tx.slot);
  }
#endif
  return true;
}

#if defined(PANDA_ROLE_CRUCE)
// Estado del cruce como atributos: sirve para ver en el mapa, sobre la
// trayectoria del tren, en qué punto el cruce pasó a NO SEGURO.
void appendCrossing(Report& r) {
  CrossingStatus cs{};
  if (!crossing::latest(cs)) {
    return;
  }
  const size_t len = strlen(r.extra);
  int n = snprintf(r.extra + len, sizeof(r.extra) - len, "&estado=%s&barrera=%s", crossing::stateName(cs.state),
                   cs.barrierDown ? "baja" : "alta");
  if (n > 0 && cs.haveTrain && !std::isnan(cs.etaMinS) && !std::isnan(cs.distM)) {
    const size_t len2 = strlen(r.extra);
    snprintf(r.extra + len2, sizeof(r.extra) - len2, "&dist_cruce=%.0f&eta_min=%.1f", static_cast<double>(cs.distM),
             static_cast<double>(cs.etaMinS));
  }
  // Los espacios de "NO SEGURO" no van en una URL.
  for (char* p = r.extra; *p != '\0'; ++p) {
    if (*p == ' ') *p = '_';
  }
}

// Posición del tren tal como llegó al cruce por LoRa. Es lo que vería el
// servidor en el producto, donde la telemetría sale desde el cruce.
bool trainViaLoraReport(Report& r) {
  radiolink::RxStatus rx{};
  if (!radiolink::latestRx(rx) || !rx.haveTrain || !(rx.last.flags & BeaconFlag::kFixOk)) {
    return false;
  }
  if (esp_timer_get_time() - rx.lastAuthUs > static_cast<int64_t>(cfg::telemetry::kMaxFixAgeMs) * 1000) {
    return false;
  }
  r.deviceId = kTrainViaLoraId;
  r.latE7 = rx.last.latE7;
  r.lonE7 = rx.last.lonE7;
  r.speedMps = rx.last.speedCms / 100.0;
  r.headingDeg = rx.last.headingCdeg / 100.0;
  r.altitudeM = 0.0;
  r.accuracyM = rx.last.hAccCm / 100.0;

  // Hora del beacon: se lleva su iTOW a UTC con la relación iTOW/UTC del GNSS
  // propio del cruce (los dos nodos comparten la semana GPS).
  r.unixS = 0;
  GnssFix own{};
  if ((rx.last.flags & BeaconFlag::kTimeSync) && xQueuePeek(g_latestFix, &own, 0) == pdTRUE && own.timeValid) {
    const int64_t diffMs = towDiffUs(static_cast<int64_t>(rx.last.itowMs) * 1000,
                                     static_cast<int64_t>(own.itowMs) * 1000) / 1000;
    r.unixS = static_cast<uint32_t>(static_cast<int64_t>(own.unixS) + diffMs / 1000);
  }

  char age[24] = "";
  if (rx.ageMs != INT32_MIN) {
    snprintf(age, sizeof(age), "&edad_ms=%ld", static_cast<long>(rx.ageMs));
  }
  char dist[20] = "";
  if (!std::isnan(rx.distM)) {
    snprintf(dist, sizeof(dist), "&dist_m=%.0f", static_cast<double>(rx.distM));
  }
  snprintf(r.extra, sizeof(r.extra), "&rssi=%.1f&snr=%.1f&per=%.1f&sats=%u&nodo=%u&valido=%u%s%s",
           static_cast<double>(rx.rssiDbm), static_cast<double>(rx.snrDb), static_cast<double>(rx.perPct),
           rx.last.numSv, rx.nodeId, rx.lastResult == static_cast<uint8_t>(RxResult::Ok) ? 1u : 0u, age, dist);
  appendCrossing(r);
  return true;
}
#endif

bool send(HTTPClient& http, WiFiClient& client, const Report& r) {
  http.begin(client, TRACCAR_HOST, TRACCAR_PORT, buildQuery(r));
  const int code = http.GET();
  http.end();
  if (code == HTTP_CODE_OK) {
    g_stats.traccarOk.fetch_add(1, std::memory_order_relaxed);
    g_stats.traccarLastOkMs.store(millis());
    return true;
  }
  g_stats.traccarFail.fetch_add(1, std::memory_order_relaxed);
  return false;
}

}  // namespace

void Telemetry::startTask() {
  const bool enabled = strlen(WIFI_SSID) > 0;
  g_stats.wifiEnabled.store(enabled);
  if (!enabled) {
    Serial.println("[TEL] WIFI_SSID vacío en secrets.h, telemetría desactivada");
    return;
  }
  Serial.printf("[TEL] Traccar %s:%d, dispositivo %s\n", TRACCAR_HOST, TRACCAR_PORT, kOwnId);
#if defined(PANDA_ROLE_CRUCE)
  Serial.printf("[TEL] Tren recibido por LoRa como %s\n", kTrainViaLoraId);
#endif
  xTaskCreatePinnedToCore(taskEntry, "telemetry", cfg::task::kStackTelemetry, this, cfg::task::kPrioTelemetry,
                          nullptr, cfg::task::kCoreAux);
}

void Telemetry::taskEntry(void* arg) {
  static_cast<Telemetry*>(arg)->taskLoop();
}

void Telemetry::taskLoop() {
  WiFi.mode(WIFI_STA);
  // Sin ahorro de energía del Wi-Fi: con el ahorro activo cada envío puede
  // demorar cientos de ms esperando que la radio despierte.
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t lastWifiBegin = millis();
  bool wasConnected = false;
#if defined(PANDA_ROLE_CRUCE)
  uint32_t lastOwnReport = 0;
#endif

  WiFiClient client;
  HTTPClient http;
  // Conexión TCP persistente entre envíos: se evita el handshake cada segundo.
  http.setReuse(true);
  http.setConnectTimeout(cfg::telemetry::kHttpTimeoutMs);
  http.setTimeout(cfg::telemetry::kHttpTimeoutMs);

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(cfg::telemetry::kPeriodMs));

    const bool connected = (WiFi.status() == WL_CONNECTED);
    g_stats.wifiConnected.store(connected);
    if (connected != wasConnected) {
      logNote(connected ? NoteCode::WifiUp : NoteCode::WifiDown);
      Serial.printf("[TEL] Wi-Fi %s\n", connected ? "conectado" : "desconectado");
      wasConnected = connected;
    }
    if (!connected) {
      // El iPhone apaga el hotspot si nadie está conectado un rato. Se reintenta
      // cada tanto por si el usuario lo vuelve a abrir.
      if (millis() - lastWifiBegin > cfg::telemetry::kWifiRetryMs) {
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiBegin = millis();
      }
      continue;
    }

    Report r{};
#if defined(PANDA_ROLE_CRUCE)
    // El cruce sube el tren recibido cada segundo y su propia posición cada 15 s.
    if (trainViaLoraReport(r)) {
      send(http, client, r);
    }
    if (millis() - lastOwnReport > cfg::telemetry::kCruceOwnPositionPeriodMs && ownReport(r)) {
      send(http, client, r);
      lastOwnReport = millis();
    }
#else
    if (ownReport(r)) {
      send(http, client, r);
    }
#endif
  }
}
