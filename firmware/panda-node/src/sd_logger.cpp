#include "sd_logger.h"

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "board_pins.h"
#include "config.h"
#include "crossing.h"
#include "radio_link.h"
#include "system_state.h"

SdLogger g_logger;

namespace {

// Archivo con buffer propio. Se escribe a la tarjeta en bloques grandes, que es
// lo que mejor rinde en una microSD, y se hace flush periódico.
class BufferedFile {
 public:
  bool open(const String& path, const char* header) {
    file_ = SD.open(path, FILE_WRITE);
    if (!file_) {
      return false;
    }
    if (buf_ == nullptr) {
      buf_ = static_cast<char*>(heap_caps_malloc(cfg::sdlog::kFileBuffer, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (buf_ == nullptr) {
        buf_ = static_cast<char*>(malloc(cfg::sdlog::kFileBuffer));
      }
    }
    len_ = 0;
    append(header);
    return buf_ != nullptr;
  }

  // Agrega texto con formato. Devuelve false si hubo error de escritura.
  bool appendf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf_ + len_, cfg::sdlog::kFileBuffer - len_, fmt, args);
    va_end(args);
    if (n < 0) {
      return true;
    }
    len_ += std::min(static_cast<size_t>(n), cfg::sdlog::kFileBuffer - len_ - 1);
    return len_ < cfg::sdlog::kWriteThreshold || drain();
  }

  bool append(const char* text) { return appendf("%s", text); }

  bool drain() {
    if (len_ == 0) {
      return true;
    }
    const size_t written = file_.write(reinterpret_cast<const uint8_t*>(buf_), len_);
    const bool ok = (written == len_);
    g_stats.logKBytes.fetch_add(static_cast<uint32_t>(written / 1024), std::memory_order_relaxed);
    len_ = 0;
    return ok;
  }

  bool flush() {
    const bool ok = drain();
    file_.flush();
    return ok;
  }

  void close() {
    if (file_) {
      drain();
      file_.close();
    }
    len_ = 0;
  }

 private:
  File file_;
  char* buf_ = nullptr;
  size_t len_ = 0;
};

BufferedFile s_gnss;
BufferedFile s_imu;
BufferedFile s_events;
BufferedFile s_link;      // tx.csv en el tren, rx.csv en el cruce
BufferedFile s_decision;  // decision.csv en el cruce
bool s_open = false;

// Formatea un valor en grados * 1e7 sin pasar por float, para no perder
// resolución (un float tiene ~0,4 m de resolución a 34° de latitud).
void formatE7(char* out, size_t size, int32_t v) {
  const char* sign = v < 0 ? "-" : "";
  const uint32_t a = static_cast<uint32_t>(v < 0 ? -static_cast<int64_t>(v) : v);
  snprintf(out, size, "%s%lu.%07lu", sign, static_cast<unsigned long>(a / 10000000UL),
           static_cast<unsigned long>(a % 10000000UL));
}

// Entero que puede faltar: INT32_MIN se escribe como campo vacío.
void formatOptionalInt(char* out, size_t size, int32_t v) {
  if (v == INT32_MIN) {
    out[0] = '\0';
  } else {
    snprintf(out, size, "%ld", static_cast<long>(v));
  }
}

// Busca el número de sesión más alto en la raíz y devuelve el siguiente.
uint32_t nextSessionNumber() {
  uint32_t maxN = 0;
  File root = SD.open("/");
  if (!root) {
    return 1;
  }
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    const char* name = f.name();
    // name() devuelve solo el nombre, sin la barra inicial.
    if (f.isDirectory() && strncmp(name, "PANDA_", 6) == 0) {
      const uint32_t n = static_cast<uint32_t>(strtoul(name + 6, nullptr, 10));
      maxN = std::max(maxN, n);
    }
    f.close();
  }
  root.close();
  return maxN + 1;
}

bool openSession() {
  const uint32_t n = nextSessionNumber();
  char dir[16];
  snprintf(dir, sizeof(dir), "/PANDA_%04lu", static_cast<unsigned long>(n));
  if (!SD.mkdir(dir)) {
    return false;
  }
  const String base(dir);
  bool ok = true;
  ok &= s_gnss.open(base + "/gnss.csv",
                    "t_us,itow_ms,unix_s,fix_type,fix_ok,num_sv,lat_deg,lon_deg,hmsl_m,speed_mps,head_deg,"
                    "hacc_m,sacc_mps,headacc_deg,pdop\n");
  ok &= s_imu.open(base + "/imu.csv", "t_us,ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps\n");
  ok &= s_events.open(base + "/events.csv", "t_us,tipo,n,valor\n");
#if defined(PANDA_ROLE_TREN)
  ok &= s_link.open(base + "/tx.csv",
                    "t_us,counter,itow_ms,tx_age_ms,air_us,status,flags,slot,perfil,tiempo,pot_dbm\n");
#elif defined(PANDA_ROLE_CRUCE)
  ok &= s_link.open(base + "/rx.csv",
                    "t_us,resultado,node_id,counter,itow_ms,age_ms,gap,rssi_dbm,snr_db,ferr_hz,flags,lat_deg,"
                    "lon_deg,speed_mps,head_deg,hacc_m,num_sv,dist_m,tiempo,perfil,origen\n");
  ok &= s_decision.open(base + "/decision.csv",
                        "t_us,estado,motivo,tren,fase,alerta,dist_m,vel_mps,acerc_mps,eta_cv_s,eta_min_s,edad_ms,"
                        "panda_libre,panda_ok,senal,via_ocupada,barrera_baja\n");
#endif
  if (ok) {
    g_stats.logSession.store(n);
    s_events.appendf("%lld,NOTA,%u,0\n", static_cast<long long>(esp_timer_get_time()),
                     static_cast<unsigned>(NoteCode::Boot));
    Serial.printf("[SD] Sesión %s abierta\n", dir);
  }
  return ok;
}

void closeSession() {
  s_gnss.close();
  s_imu.close();
  s_events.close();
  s_link.close();
  s_decision.close();
  s_open = false;
}

bool mountCard() {
  // El bus SPI compartido ya lo inicializó el setup(). SD.begin solo agrega la
  // tarjeta como dispositivo con su propio CS.
  // Hasta 6 archivos abiertos por sesión más el listado de la raíz.
  if (!SD.begin(pins::kSdCs, SPI, cfg::sdlog::kSdSpiHz, "/sd", 8)) {
    SD.end();
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    SD.end();
    return false;
  }
  Serial.printf("[SD] Tarjeta montada, %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}

bool writeRecord(const LogRecord& r) {
  switch (r.type) {
    case LogType::Gnss: {
      const GnssFix& f = r.gnss;
      char lat[16];
      char lon[16];
      formatE7(lat, sizeof(lat), f.latE7);
      formatE7(lon, sizeof(lon), f.lonE7);
      return s_gnss.appendf("%lld,%lu,%lu,%u,%u,%u,%s,%s,%.3f,%.3f,%.5f,%.3f,%.3f,%.5f,%.2f\n",
                            static_cast<long long>(f.tRxUs), static_cast<unsigned long>(f.itowMs),
                            static_cast<unsigned long>(f.unixS), f.fixType, f.fixOk ? 1u : 0u, f.numSv, lat, lon,
                            f.hMslMm / 1000.0, f.gSpeedMms / 1000.0, f.headMotE5 / 1e5, f.hAccMm / 1000.0,
                            f.sAccMms / 1000.0, f.headAccE5 / 1e5, f.pDopE2 / 100.0);
    }
    case LogType::Imu: {
      const ImuSample& s = r.imu;
      return s_imu.appendf("%lld,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n", static_cast<long long>(s.tUs),
                           static_cast<double>(s.ax), static_cast<double>(s.ay), static_cast<double>(s.az),
                           static_cast<double>(s.gx), static_cast<double>(s.gy), static_cast<double>(s.gz));
    }
    case LogType::Pps:
      return s_events.appendf("%lld,PPS,%lu,%lu\n", static_cast<long long>(r.ev.tUs),
                              static_cast<unsigned long>(r.ev.seq), static_cast<unsigned long>(r.ev.value));
    case LogType::Mark:
      return s_events.appendf("%lld,MARCA,%lu,%lu\n", static_cast<long long>(r.ev.tUs),
                              static_cast<unsigned long>(r.ev.seq), static_cast<unsigned long>(r.ev.value));
    case LogType::Note:
      return s_events.appendf("%lld,NOTA,%lu,%lu\n", static_cast<long long>(r.ev.tUs),
                              static_cast<unsigned long>(r.ev.seq), static_cast<unsigned long>(r.ev.value));
    case LogType::Tx: {
      const TxLog& t = r.tx;
      char age[12];
      formatOptionalInt(age, sizeof(age), t.txAgeMs);
      return s_link.appendf("%lld,%lu,%lu,%s,%lu,%d,%u,%u,%u,%u,%d\n", static_cast<long long>(t.tStartUs),
                            static_cast<unsigned long>(t.counter), static_cast<unsigned long>(t.itowMs), age,
                            static_cast<unsigned long>(t.airUs), t.status, t.flags, t.slot, t.profile, t.timeQ,
                            t.powerDbm);
    }
    case LogType::Rx: {
      const RxLog& x = r.rx;
      char age[12];
      char lat[16];
      char lon[16];
      char dist[16];
      formatOptionalInt(age, sizeof(age), x.ageMs);
      formatE7(lat, sizeof(lat), x.latE7);
      formatE7(lon, sizeof(lon), x.lonE7);
      if (std::isnan(x.distM)) {
        dist[0] = '\0';
      } else {
        snprintf(dist, sizeof(dist), "%.1f", static_cast<double>(x.distM));
      }
      return s_link.appendf("%lld,%s,%u,%lu,%lu,%s,%u,%.1f,%.2f,%.0f,%u,%s,%s,%.2f,%.2f,%.2f,%u,%s,%u,%u,%s\n",
                            static_cast<long long>(x.tEndUs), radiolink::rxResultName(x.result), x.nodeId,
                            static_cast<unsigned long>(x.counter), static_cast<unsigned long>(x.itowMs), age, x.gap,
                            static_cast<double>(x.rssiDbm), static_cast<double>(x.snrDb),
                            static_cast<double>(x.freqErrHz), x.flags, lat, lon, x.speedCms / 100.0,
                            x.headingCdeg / 100.0, x.hAccCm / 100.0, x.numSv, dist, x.timeQ, x.profile,
                            x.injected ? "USB" : "RADIO");
    }
    case LogType::Decision: {
      const DecisionLog& d = r.dec;
      char eta[12];
      if (std::isnan(d.etaCvS)) {
        eta[0] = '\0';
      } else {
        snprintf(eta, sizeof(eta), "%.2f", static_cast<double>(d.etaCvS));
      }
      char age[12];
      formatOptionalInt(age, sizeof(age), d.ageMs);
      return s_decision.appendf("%lld,%s,%s,%04X,%s,%u,%.1f,%.2f,%.2f,%s,%.2f,%s,%u,%u,%u,%u,%u\n",
                                static_cast<long long>(d.tUs), crossing::stateName(static_cast<CrossState>(d.state)),
                                crossing::reasonName(static_cast<CrossReason>(d.reason)), d.trainId,
                                crossing::phaseName(static_cast<TrainPhase>(d.phase)), d.alerting,
                                static_cast<double>(d.distM), static_cast<double>(d.speedMps),
                                static_cast<double>(d.closingMps), eta, static_cast<double>(d.etaMinS), age,
                                (d.outputs & 1) ? 1u : 0u, (d.outputs & 2) ? 1u : 0u, (d.outputs & 4) ? 1u : 0u,
                                (d.outputs & 8) ? 1u : 0u, (d.outputs & 16) ? 1u : 0u);
    }
  }
  return true;
}

}  // namespace

void SdLogger::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "sdlog", cfg::task::kStackLogger, this, cfg::task::kPrioLogger, nullptr,
                          cfg::task::kCoreAux);
}

void SdLogger::taskEntry(void* arg) {
  static_cast<SdLogger*>(arg)->taskLoop();
}

void SdLogger::taskLoop() {
  uint32_t lastMountTry = 0;
  uint32_t lastFlush = millis();
  bool mountTried = false;
  LogRecord rec;

  for (;;) {
    // Montaje (y remontaje si se sacó la tarjeta).
    if (!s_open && (!mountTried || millis() - lastMountTry > cfg::sdlog::kRemountPeriodMs)) {
      mountTried = true;
      lastMountTry = millis();
      if (mountCard() && openSession()) {
        s_open = true;
        g_stats.sdOnline.store(true);
      } else {
        closeSession();
        SD.end();
        g_stats.sdOnline.store(false);
      }
    }

    // Se vacía la cola siempre, haya tarjeta o no. Sin tarjeta los registros se
    // descartan, así los productores nunca ven la cola llena por esto.
    if (xQueueReceive(g_logQueue, &rec, pdMS_TO_TICKS(100)) == pdTRUE) {
      do {
        if (s_open) {
          if (writeRecord(rec)) {
            g_stats.logRecords.fetch_add(1, std::memory_order_relaxed);
          } else {
            Serial.println("[SD] Error de escritura, se cierra la sesión");
            closeSession();
            SD.end();
            g_stats.sdOnline.store(false);
          }
        }
      } while (xQueueReceive(g_logQueue, &rec, 0) == pdTRUE);
    }

    if (s_open && millis() - lastFlush > cfg::sdlog::kFlushPeriodMs) {
      lastFlush = millis();
      const bool ok = s_gnss.flush() && s_imu.flush() && s_events.flush() && s_link.flush() && s_decision.flush();
      if (!ok) {
        Serial.println("[SD] Error en flush, se cierra la sesión");
        closeSession();
        SD.end();
        g_stats.sdOnline.store(false);
      }
    }
  }
}
