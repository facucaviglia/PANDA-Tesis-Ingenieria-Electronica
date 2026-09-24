#include "gnss_manager.h"

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <esp_timer.h>

#include "board_pins.h"
#include "config.h"
#include "system_state.h"

GnssManager g_gnss;

static SFE_UBLOX_GNSS_SERIAL s_gnss;
static HardwareSerial& s_uart = Serial1;

// ---------------------------------------------------------------------------
// 1PPS
//
// La ISR solo toma el tiempo y cuenta. Todo lo demás (registro, estadística)
// lo hace la tarea GNSS, para que la ISR sea corta y determinista.
// ---------------------------------------------------------------------------
static portMUX_TYPE s_ppsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t s_ppsLastUs = 0;
static volatile uint32_t s_ppsCount = 0;

static void IRAM_ATTR onPpsIsr() {
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&s_ppsMux);
  s_ppsLastUs = now;
  s_ppsCount = s_ppsCount + 1;
  portEXIT_CRITICAL_ISR(&s_ppsMux);
}

// ---------------------------------------------------------------------------
// Anclas del reloj GPS local. Las escribe la tarea GNSS y las leen la radio y
// la telemetría, por eso van protegidas con un spinlock (son de 64 bits).
// ---------------------------------------------------------------------------
struct TimeAnchor {
  int64_t localUs = 0;  // Instante local de referencia
  int64_t towUs = 0;    // TOW en ese instante
  bool valid = false;
};
static portMUX_TYPE s_anchorMux = portMUX_INITIALIZER_UNLOCKED;
static TimeAnchor s_ppsAnchor;
static TimeAnchor s_coarseAnchor;
static TimeAnchor s_pcAnchor;

void setPcTime(int64_t towUs) {
  if (!cfg::sim::kAllowPcTime) {
    return;
  }
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&s_anchorMux);
  s_pcAnchor = {now, towUs, true};
  portEXIT_CRITICAL(&s_anchorMux);
}

int64_t towDiffUs(int64_t a, int64_t b) {
  int64_t d = (a - b) % kGpsWeekUs;
  if (d >= kGpsWeekUs / 2) d -= kGpsWeekUs;
  if (d < -kGpsWeekUs / 2) d += kGpsWeekUs;
  return d;
}

static int64_t towWrap(int64_t t) {
  t %= kGpsWeekUs;
  return t < 0 ? t + kGpsWeekUs : t;
}

TimeQuality gpsTowAt(int64_t tLocalUs, int64_t& towUs) {
  TimeAnchor pps;
  TimeAnchor coarse;
  TimeAnchor pc;
  portENTER_CRITICAL(&s_anchorMux);
  pps = s_ppsAnchor;
  coarse = s_coarseAnchor;
  pc = s_pcAnchor;
  portEXIT_CRITICAL(&s_anchorMux);

  if (pps.valid && (tLocalUs - pps.localUs) < static_cast<int64_t>(cfg::gnss::kPpsHoldoverMs) * 1000) {
    towUs = towWrap(pps.towUs + (tLocalUs - pps.localUs));
    return TimeQuality::Pps;
  }
  if (coarse.valid && (tLocalUs - coarse.localUs) < static_cast<int64_t>(cfg::gnss::kCoarseMaxAgeMs) * 1000) {
    towUs = towWrap(coarse.towUs + (tLocalUs - coarse.localUs));
    return TimeQuality::Coarse;
  }
  // El tiempo de la PC es el último recurso y solo para banco.
  if (pc.valid && (tLocalUs - pc.localUs) < static_cast<int64_t>(cfg::sim::kPcTimeHoldMs) * 1000) {
    towUs = towWrap(pc.towUs + (tLocalUs - pc.localUs));
    return TimeQuality::Pc;
  }
  return TimeQuality::None;
}

const char* timeQualityName(TimeQuality q) {
  switch (q) {
    case TimeQuality::Pps:
      return "PPS";
    case TimeQuality::Coarse:
      return "GRUESO";
    case TimeQuality::Pc:
      return "PC";
    default:
      return "NO";
  }
}

// ---------------------------------------------------------------------------
// Callback de NAV-PVT
//
// La llama la librería desde checkCallbacks(), dentro de la tarea GNSS. Copia
// los campos que usa PANDA a nuestra estructura, que no depende de la librería.
// ---------------------------------------------------------------------------
static int64_t s_lastFixUs = 0;

static void onNavPvt(UBX_NAV_PVT_data_t* pvt) {
  GnssFix fix{};
  fix.tRxUs = esp_timer_get_time();
  fix.itowMs = pvt->iTOW;
  fix.latE7 = pvt->lat;
  fix.lonE7 = pvt->lon;
  fix.hMslMm = pvt->hMSL;
  fix.gSpeedMms = pvt->gSpeed;
  fix.headMotE5 = pvt->headMot;
  fix.hAccMm = pvt->hAcc;
  fix.sAccMms = pvt->sAcc;
  fix.headAccE5 = pvt->headAcc;
  fix.pDopE2 = pvt->pDOP;
  fix.fixType = pvt->fixType;
  fix.numSv = pvt->numSV;

  // Un fix sirve para PANDA si el receptor lo marca dentro de sus máscaras de
  // DOP y precisión, es 2D o 3D (o combinado con DR), y la posición no está
  // marcada como inválida.
  const bool typeOk = (fix.fixType >= 2 && fix.fixType <= 4);
  fix.fixOk = pvt->flags.bits.gnssFixOK && typeOk && !pvt->flags3.bits.invalidLlh;

  fix.timeValid = pvt->valid.bits.validDate && pvt->valid.bits.validTime && pvt->valid.bits.fullyResolved;
  fix.unixS = fix.timeValid ? utcToUnix(pvt->year, pvt->month, pvt->day, pvt->hour, pvt->min, pvt->sec) : 0;

  xQueueOverwrite(g_latestFix, &fix);

  // Anclas de tiempo. El iTOW solo es confiable con fecha y hora resueltas.
  if (fix.timeValid) {
    int64_t ppsUs;
    portENTER_CRITICAL(&s_ppsMux);
    ppsUs = s_ppsLastUs;
    portEXIT_CRITICAL(&s_ppsMux);

    portENTER_CRITICAL(&s_anchorMux);
    s_coarseAnchor = {fix.tRxUs, static_cast<int64_t>(fix.itowMs) * 1000, true};
    // La solución de la época del segundo entero (iTOW múltiplo de 1000) llega
    // unos ms después del flanco del PPS que marcó ese mismo segundo. Si el
    // último flanco cae en esa ventana, se toma como ancla precisa.
    const int64_t sincePps = fix.tRxUs - ppsUs;
    if ((fix.itowMs % 1000) == 0 && ppsUs != 0 && sincePps > 0 && sincePps < 400000) {
      s_ppsAnchor = {ppsUs, static_cast<int64_t>(fix.itowMs) * 1000, true};
    }
    portEXIT_CRITICAL(&s_anchorMux);
  }

  LogRecord rec{};
  rec.type = LogType::Gnss;
  rec.gnss = fix;
  logPush(rec);

  s_lastFixUs = fix.tRxUs;
  g_stats.gnssFixes.fetch_add(1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Días desde la época civil (algoritmo de Howard Hinnant). Exacto para todo el
// calendario gregoriano y sin tablas.
// ---------------------------------------------------------------------------
uint32_t utcToUnix(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
  int32_t y = static_cast<int32_t>(year) - (month <= 2 ? 1 : 0);
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = static_cast<uint32_t>(y - era * 400);
  const uint32_t mp = (month + 9) % 12;
  const uint32_t doy = (153 * mp + 2) / 5 + day - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(doe) - 719468;
  return static_cast<uint32_t>(days * 86400 + hour * 3600 + min * 60 + sec);
}

// ---------------------------------------------------------------------------
// Configuración del receptor
// ---------------------------------------------------------------------------
bool GnssManager::begin() {
  // El pin de wakeup en alto evita que el módulo entre en modo de bajo consumo.
  pinMode(pins::kGnssWakeup, OUTPUT);
  digitalWrite(pins::kGnssWakeup, HIGH);

  pinMode(pins::kGnssPps, INPUT);
  attachInterrupt(digitalPinToInterrupt(pins::kGnssPps), onPpsIsr, RISING);

  // El buffer de RX se fija antes de begin(), después no se puede cambiar.
  s_uart.setRxBufferSize(cfg::gnss::kUartRxBuffer);

  return configure();
}

bool GnssManager::configure() {
  g_stats.gnssOnline.store(false);

  // 1) Encontrar al módulo probando velocidades.
  uint32_t found = 0;
  for (uint32_t baud : cfg::gnss::kProbeBauds) {
    s_uart.end();
    s_uart.begin(baud, SERIAL_8N1, pins::kGnssRx, pins::kGnssTx);
    if (s_gnss.begin(s_uart, 800)) {
      found = baud;
      break;
    }
  }
  if (found == 0) {
    Serial.println("[GNSS] No responde en ninguna velocidad");
    return false;
  }
  Serial.printf("[GNSS] Detectado a %lu baud\n", static_cast<unsigned long>(found));

  // 2) Subir la velocidad. El ACK del módulo sale ya a la velocidad nueva, así
  //    que el resultado de setSerialRate no es confiable. Se verifica después.
  if (found != cfg::gnss::kTargetBaud) {
    s_gnss.setSerialRate(cfg::gnss::kTargetBaud, COM_PORT_UART1, VAL_LAYER_RAM, 250);
    s_uart.flush();
    delay(50);
    s_uart.updateBaudRate(cfg::gnss::kTargetBaud);
    delay(50);
    if (!s_gnss.isConnected(1000)) {
      Serial.println("[GNSS] Se perdió al cambiar de velocidad");
      return false;
    }
  }
  g_stats.gnssBaud.store(cfg::gnss::kTargetBaud);

  // 3) Configuración de navegación, toda en la capa RAM del módulo. Si se corta
  //    la alimentación vuelve a fábrica y este código la repite. Así el módulo
  //    nunca queda en un estado que no conocemos.
  bool ok = true;
  ok &= s_gnss.setUART1Output(COM_TYPE_UBX, VAL_LAYER_RAM);
  ok &= s_gnss.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM);
  ok &= s_gnss.setNavigationFrequency(cfg::gnss::kNavRateHz, VAL_LAYER_RAM);
  ok &= s_gnss.setAutoPVTcallbackPtr(&onNavPvt, VAL_LAYER_RAM);
  if (!ok) {
    Serial.println("[GNSS] Alguna configuración no fue aceptada");
    return false;
  }

  Serial.printf("[GNSS] Configurado: UBX, %u Hz, automotriz, %lu baud\n", cfg::gnss::kNavRateHz,
                static_cast<unsigned long>(cfg::gnss::kTargetBaud));
  s_lastFixUs = esp_timer_get_time();
  g_stats.gnssOnline.store(true);
  logNote(NoteCode::GnssOnline, cfg::gnss::kTargetBaud);
  return true;
}

// ---------------------------------------------------------------------------
// Tarea GNSS (core 1)
// ---------------------------------------------------------------------------
void GnssManager::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "gnss", cfg::task::kStackGnss, this, cfg::task::kPrioGnss, nullptr,
                          cfg::task::kCoreSafety);
}

void GnssManager::taskEntry(void* arg) {
  static_cast<GnssManager*>(arg)->taskLoop();
}

void GnssManager::taskLoop() {
  uint32_t lastPps = 0;
  int64_t lastPpsUs = 0;
  uint32_t rateWindowFixes = g_stats.gnssFixes.load();
  int64_t rateWindowStart = esp_timer_get_time();
  int64_t lastReinit = esp_timer_get_time();

  for (;;) {
    // Procesa lo que haya en la UART y dispara onNavPvt por cada NAV-PVT.
    // Cada 2 ms entran como mucho 92 bytes a 460800 baud, muy por debajo del
    // buffer, así que ningún mensaje se pierde por esperar.
    if (g_stats.gnssOnline.load()) {
      s_gnss.checkUblox();
      s_gnss.checkCallbacks();
    }

    const int64_t now = esp_timer_get_time();

    // 1PPS: registrar cada flanco nuevo con el error respecto de 1 s exacto.
    uint32_t ppsCount;
    int64_t ppsUs;
    portENTER_CRITICAL(&s_ppsMux);
    ppsCount = s_ppsCount;
    ppsUs = s_ppsLastUs;
    portEXIT_CRITICAL(&s_ppsMux);
    if (ppsCount != lastPps) {
      const int64_t interval = (lastPpsUs != 0) ? (ppsUs - lastPpsUs) : 0;
      if (interval > 0) {
        g_stats.ppsErrorUs.store(static_cast<int32_t>(interval - 1000000));
      }
      LogRecord rec{};
      rec.type = LogType::Pps;
      rec.ev.tUs = ppsUs;
      rec.ev.seq = ppsCount;
      rec.ev.value = static_cast<uint32_t>(interval);
      logPush(rec);
      g_stats.ppsCount.store(ppsCount);
      lastPps = ppsCount;
      lastPpsUs = ppsUs;
    }

    // Tasa real de soluciones en ventanas de 1 s.
    if (now - rateWindowStart >= 1000000) {
      const uint32_t fixes = g_stats.gnssFixes.load();
      const uint32_t rateX10 = static_cast<uint32_t>((fixes - rateWindowFixes) * 10000000LL / (now - rateWindowStart));
      g_stats.gnssRateX10.store(rateX10);
      rateWindowFixes = fixes;
      rateWindowStart = now;
    }

    // Vigilancia: sin NAV-PVT por más de kTimeoutMs se considera caído (por
    // ejemplo, si el módulo se reinició y volvió a 9600 baud).
    if (g_stats.gnssOnline.load() && (now - s_lastFixUs) > cfg::gnss::kTimeoutMs * 1000LL) {
      Serial.println("[GNSS] Sin datos, se marca fuera de línea");
      g_stats.gnssOnline.store(false);
      logNote(NoteCode::GnssLost);
    }
    if (!g_stats.gnssOnline.load() && (now - lastReinit) > cfg::gnss::kReinitPeriodMs * 1000LL) {
      lastReinit = now;
      configure();
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
