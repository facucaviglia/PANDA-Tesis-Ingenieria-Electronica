#include "display_manager.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <climits>
#include <cmath>
#include <esp_timer.h>

#include "board_pins.h"
#include "button.h"
#include "config.h"
#include "crossing.h"
#include "gnss_manager.h"
#include "pmu_manager.h"
#include "radio_link.h"
#include "radio_manager.h"
#include "system_state.h"

DisplayManager g_display;

// I2C por hardware con los pines de la placa. U8g2 llama a Wire.begin(sda, scl)
// con estos pines, así nunca cae en los pines por defecto del ESP32-S3 (8 y 9),
// que en esta placa son la UART del GNSS.
static U8G2_SH1106_128X64_NONAME_F_HW_I2C s_u8g2(U8G2_R0, U8X8_PIN_NONE, pins::kI2cScl, pins::kI2cSda);

static constexpr uint8_t kLineH = 10;  // Alto de renglón con la fuente 6x10
static constexpr uint8_t kLines = 6;

bool DisplayManager::begin() {
  // Se sondea la dirección antes de inicializar para no escribir a ciegas.
  Wire.begin(pins::kI2cSda, pins::kI2cScl, cfg::ui::kI2cHz);
  Wire.beginTransmission(board::kOledI2cAddr);
  if (Wire.endTransmission() != 0) {
    Serial.println("[OLED] No responde en 0x3C, se sigue sin pantalla");
    online_ = false;
    return false;
  }
  s_u8g2.setBusClock(cfg::ui::kI2cHz);
  s_u8g2.begin();
  s_u8g2.setFont(u8g2_font_6x10_tf);
  s_u8g2.clearBuffer();
  s_u8g2.drawStr(0, kLineH, "PANDA " PANDA_ROLE_NAME);
  s_u8g2.sendBuffer();
  online_ = true;
  bootLine_ = 1;
  return true;
}

void DisplayManager::bootStep(const char* line) {
  Serial.printf("[BOOT] %s\n", line);
  if (!online_) {
    return;
  }
  if (bootLine_ >= kLines) {
    // Pantalla llena: se reinicia debajo del título.
    s_u8g2.clearBuffer();
    s_u8g2.drawStr(0, kLineH, "PANDA " PANDA_ROLE_NAME);
    bootLine_ = 1;
  }
  s_u8g2.drawStr(0, kLineH * (bootLine_ + 1), line);
  s_u8g2.sendBuffer();
  ++bootLine_;
}

void DisplayManager::showFatal(const char* subsystem, const char* detail) {
  if (!online_) {
    return;
  }
  s_u8g2.clearBuffer();
  s_u8g2.setFont(u8g2_font_helvB12_tf);
  s_u8g2.drawStr(0, 16, "FALLA");
  s_u8g2.drawStr(0, 34, subsystem);
  s_u8g2.setFont(u8g2_font_6x10_tf);
  s_u8g2.drawStr(0, 50, detail);
  s_u8g2.drawStr(0, 62, "Nodo NO operativo");
  s_u8g2.sendBuffer();
}

void DisplayManager::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "ui", cfg::task::kStackUi, this, cfg::task::kPrioUi, nullptr,
                          cfg::task::kCoreAux);
}

void DisplayManager::taskEntry(void* arg) {
  static_cast<DisplayManager*>(arg)->taskLoop();
}

// ---------------------------------------------------------------------------
// Utilidades de dibujo
// ---------------------------------------------------------------------------
static uint32_t s_lastMarkMs = 0;
static uint32_t s_lastMarkNum = 0;

static void drawLine(uint8_t row, const char* text) {
  s_u8g2.drawStr(0, kLineH * (row + 1) - 1, text);
}

// Renglón superior común: rol, microSD, Wi-Fi y batería. Si hubo una marca en
// los últimos 2 s se muestra invertido para confirmar la pulsación.
static void drawHeader() {
  char line[32];
  if (millis() - s_lastMarkMs < 2000 && s_lastMarkNum > 0) {
    s_u8g2.drawBox(0, 0, 128, kLineH + 1);
    s_u8g2.setDrawColor(0);
    snprintf(line, sizeof(line), "  MARCA #%lu", static_cast<unsigned long>(s_lastMarkNum));
    drawLine(0, line);
    s_u8g2.setDrawColor(1);
    return;
  }
  const int32_t batt = g_stats.battPercent.load();
  char battStr[12];
  if (batt >= 0) {
    snprintf(battStr, sizeof(battStr), "%ld%%", static_cast<long>(batt));
  } else {
    snprintf(battStr, sizeof(battStr), "USB");
  }
  snprintf(line, sizeof(line), "%s SD%s WF%s %s", PANDA_ROLE_SHORT, g_stats.sdOnline.load() ? "+" : "-",
           !g_stats.wifiEnabled.load() ? "x" : (g_stats.wifiConnected.load() ? "+" : "-"), battStr);
  drawLine(0, line);
}

#if !defined(PANDA_ROLE_CRUCE)
static void fixLine(char* line, size_t size, const GnssFix& fix, bool haveFix) {
  if (!g_stats.gnssOnline.load()) {
    snprintf(line, size, "GNSS FUERA DE LINEA");
  } else if (!haveFix || !fix.fixOk) {
    snprintf(line, size, "Sin fix  sv%u", haveFix ? fix.numSv : 0u);
  } else {
    snprintf(line, size, "%s sv%u hAcc %.1fm", fix.fixType == 3 ? "3D" : "2D", fix.numSv, fix.hAccMm / 1000.0);
  }
}
#endif


#if !defined(PANDA_ROLE_CRUCE)
static const char* traccarState() {
  if (!g_stats.wifiEnabled.load()) return "x";
  if (g_stats.traccarOk.load() > 0 && millis() - g_stats.traccarLastOkMs.load() < 3000) return "ok";
  if (g_stats.traccarFail.load() > 0) return "er";
  return "--";
}
#endif

// ---------------------------------------------------------------------------
// Pantallas por rol
// ---------------------------------------------------------------------------
#if defined(PANDA_ROLE_REGISTRADOR)
static void drawRole() {
  GnssFix fix{};
  const bool haveFix = xQueuePeek(g_latestFix, &fix, 0) == pdTRUE;
  char line[32];
  fixLine(line, sizeof(line), fix, haveFix);
  drawLine(1, line);
  if (haveFix && fix.fixOk) {
    snprintf(line, sizeof(line), "%.1f km/h  R %03ld", fix.gSpeedMms * 0.0036, static_cast<long>(fix.headMotE5 / 100000));
  } else {
    snprintf(line, sizeof(line), "--.- km/h");
  }
  drawLine(2, line);
  const uint32_t rate = g_stats.gnssRateX10.load();
  snprintf(line, sizeof(line), "GNSS %lu.%luHz PPS%+ldus", static_cast<unsigned long>(rate / 10),
           static_cast<unsigned long>(rate % 10), static_cast<long>(g_stats.ppsErrorUs.load()));
  drawLine(3, line);
  if (g_stats.imuOnline.load()) {
    snprintf(line, sizeof(line), "IMU %luHz %s %.2fg", static_cast<unsigned long>(g_stats.imuRateX10.load() / 10),
             g_stats.imuStill.load() ? "QUIETO" : "MOV", g_stats.imuAccNormMg.load() / 1000.0);
  } else {
    snprintf(line, sizeof(line), "IMU FUERA DE LINEA");
  }
  drawLine(4, line);
  snprintf(line, sizeof(line), "S%lu %luk d%lu T:%s", static_cast<unsigned long>(g_stats.logSession.load()),
           static_cast<unsigned long>(g_stats.logRecords.load() / 1000),
           static_cast<unsigned long>(g_stats.logDropped.load()), traccarState());
  drawLine(5, line);
}

#elif defined(PANDA_ROLE_TREN)
static void drawRole() {
  GnssFix fix{};
  const bool haveFix = xQueuePeek(g_latestFix, &fix, 0) == pdTRUE;
  radiolink::TxStatus tx{};
  const bool haveTx = radiolink::latestTx(tx);
  char line[32];

  fixLine(line, sizeof(line), fix, haveFix);
  drawLine(1, line);

  if (haveFix && fix.fixOk) {
    snprintf(line, sizeof(line), "%.1f km/h  R %03ld", fix.gSpeedMms * 0.0036, static_cast<long>(fix.headMotE5 / 100000));
  } else {
    snprintf(line, sizeof(line), "--.- km/h");
  }
  drawLine(2, line);

  if (haveTx) {
    if (tx.slot == 255) {
      snprintf(line, sizeof(line), "TX %lu SIN SINC", static_cast<unsigned long>(tx.txCount));
    } else {
      snprintf(line, sizeof(line), "TX %lu r%u/%u", static_cast<unsigned long>(tx.txCount), tx.slot, tx.slotCount);
    }
  } else {
    snprintf(line, sizeof(line), "TX --");
  }
  drawLine(3, line);

  // Perfil, potencia y antigüedad del fix al transmitir (E-6 del lado tren).
  const auto& prof = radiolink::activeProfile();
  if (haveTx && tx.lastTxAgeMs != INT32_MIN) {
    snprintf(line, sizeof(line), "SF%u/%.0f %ddBm e%ld", prof.sf, static_cast<double>(prof.bwKHz),
             radiolink::activePowerDbm(), static_cast<long>(tx.lastTxAgeMs));
  } else {
    snprintf(line, sizeof(line), "SF%u/%.0f %ddBm e--", prof.sf, static_cast<double>(prof.bwKHz),
             radiolink::activePowerDbm());
  }
  drawLine(4, line);

  snprintf(line, sizeof(line), "ID %04lX S%lu d%lu T:%s", static_cast<unsigned long>(g_stats.nodeId.load()),
           static_cast<unsigned long>(g_stats.logSession.load()), static_cast<unsigned long>(g_stats.logDropped.load()),
           traccarState());
  drawLine(5, line);
}

#elif defined(PANDA_ROLE_CRUCE)
static void drawRole() {
  CrossingStatus cs{};
  const bool haveCs = crossing::latest(cs);
  radiolink::RxStatus rx{};
  const bool haveRx = radiolink::latestRx(rx);
  char line[32];

  // Estado en grande: es lo que vería el peatón (NO SEGURO) o el operador.
  const CrossState state = haveCs ? cs.state : CrossState::Iniciando;
  s_u8g2.setFont(u8g2_font_helvB14_tf);
  const char* big = crossing::stateName(state);
  if (state == CrossState::NoSeguro) {
    // Invertido para que se distinga de lejos.
    s_u8g2.drawBox(0, 12, 128, 19);
    s_u8g2.setDrawColor(0);
    s_u8g2.drawStr((128 - s_u8g2.getStrWidth(big)) / 2, 28, big);
    s_u8g2.setDrawColor(1);
  } else {
    s_u8g2.drawStr((128 - s_u8g2.getStrWidth(big)) / 2, 28, big);
  }
  s_u8g2.setFont(u8g2_font_6x10_tf);

  // Motivo.
  if (haveCs && cs.reasonTrain != 0) {
    snprintf(line, sizeof(line), "%s %04X", crossing::reasonName(cs.reason), cs.reasonTrain);
  } else {
    snprintf(line, sizeof(line), "%s", haveCs ? crossing::reasonName(cs.reason) : "arrancando");
  }
  s_u8g2.drawStr(0, 41, line);

  // Tren principal: distancia, velocidad y ETA mínimo. Sin tren, el enlace.
  if (haveCs && cs.haveTrain && !std::isnan(cs.distM)) {
    if (!std::isnan(cs.etaMinS) && cs.etaMinS < 999.0f) {
      snprintf(line, sizeof(line), "%.0fm %.0fkm/h e%.0fs", static_cast<double>(cs.distM),
               static_cast<double>(cs.speedMps) * 3.6, static_cast<double>(cs.etaMinS));
    } else {
      snprintf(line, sizeof(line), "%.0fm %.0fkm/h", static_cast<double>(cs.distM), static_cast<double>(cs.speedMps) * 3.6);
    }
  } else if (haveRx && rx.lastAuthUs != 0) {
    snprintf(line, sizeof(line), "RSSI %.0f ok%lu", static_cast<double>(rx.rssiDbm), static_cast<unsigned long>(rx.ok));
  } else {
    snprintf(line, sizeof(line), "sin beacons");
  }
  s_u8g2.drawStr(0, 52, line);

  // Salidas: barrera de referencia, circuito de vía y PANDA operativo.
  if (haveCs) {
    snprintf(line, sizeof(line), "bar %s via %s%s P%s", cs.barrierDown ? "BAJA" : "alta",
             cs.trackOccupied ? "OCU" : "lib", cs.trackEnabled ? "" : "*", cs.pandaOk ? "+" : "-");
  } else {
    snprintf(line, sizeof(line), "--");
  }
  s_u8g2.drawStr(0, 63, line);
}
#endif

void DisplayManager::drawStatus() {
  s_u8g2.clearBuffer();
  s_u8g2.setFont(u8g2_font_6x10_tf);
  drawHeader();
  drawRole();
  s_u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// Consola serie
// ---------------------------------------------------------------------------
static void printStatusLine() {
  GnssFix fix{};
  const bool haveFix = xQueuePeek(g_latestFix, &fix, 0) == pdTRUE;
  const uint32_t rate = g_stats.gnssRateX10.load();
  int64_t tow = 0;
  const TimeQuality tq = gpsTowAt(esp_timer_get_time(), tow);
  Serial.printf("[%6lus] GNSS %s fix%u sv%u %lu.%luHz hAcc%.1fm v%.1fm/s t:%s | PPS %lu err%ldus | ",
                static_cast<unsigned long>(millis() / 1000), g_stats.gnssOnline.load() ? "on" : "OFF",
                haveFix ? fix.fixType : 0u, haveFix ? fix.numSv : 0u, static_cast<unsigned long>(rate / 10),
                static_cast<unsigned long>(rate % 10), haveFix ? fix.hAccMm / 1000.0 : 0.0,
                haveFix ? fix.gSpeedMms / 1000.0 : 0.0, timeQualityName(tq),
                static_cast<unsigned long>(g_stats.ppsCount.load()), static_cast<long>(g_stats.ppsErrorUs.load()));
#if defined(PANDA_ROLE_TREN)
  radiolink::TxStatus tx{};
  if (radiolink::latestTx(tx)) {
    Serial.printf("TX %lu ranura %u/%u edad %ldms aire %luus | ", static_cast<unsigned long>(tx.txCount), tx.slot,
                  tx.slotCount, static_cast<long>(tx.lastTxAgeMs == INT32_MIN ? -1 : tx.lastTxAgeMs),
                  static_cast<unsigned long>(tx.lastAirUs));
  }
#elif defined(PANDA_ROLE_CRUCE)
  CrossingStatus cs{};
  if (crossing::latest(cs)) {
    Serial.printf("CRUCE %s (%s) trenes %u", crossing::stateName(cs.state), crossing::reasonName(cs.reason),
                  cs.trainCount);
    if (cs.haveTrain && !std::isnan(cs.distM)) {
      Serial.printf(" [%04X %s d%.0fm v%.1fm/s acerc%.1f eta%.1fs min%.1fs]", cs.trainId, crossing::phaseName(cs.phase),
                    static_cast<double>(cs.distM), static_cast<double>(cs.speedMps),
                    static_cast<double>(cs.closingMps), static_cast<double>(cs.etaCvS),
                    static_cast<double>(cs.etaMinS));
    }
    Serial.printf(" bar %s via %s | ", cs.barrierDown ? "BAJA" : "alta", cs.trackOccupied ? "OCUPADA" : "libre");
  }
  radiolink::RxStatus rx{};
  if (radiolink::latestRx(rx)) {
    Serial.printf("RX ok%lu sint%lu crc%lu mac%lu rep%lu vie%lu | RSSI %.1f SNR %.1f edad %ldms PER %.1f%% dist %.0fm | ",
                  static_cast<unsigned long>(rx.ok), static_cast<unsigned long>(rx.unverified),
                  static_cast<unsigned long>(rx.crcError), static_cast<unsigned long>(rx.badTag),
                  static_cast<unsigned long>(rx.replay), static_cast<unsigned long>(rx.stale),
                  static_cast<double>(rx.rssiDbm), static_cast<double>(rx.snrDb),
                  static_cast<long>(rx.ageMs == INT32_MIN ? -1 : rx.ageMs), static_cast<double>(rx.perPct),
                  std::isnan(rx.distM) ? -1.0 : static_cast<double>(rx.distM));
  }
#endif
  Serial.printf("IMU %lu.%luHz %s | SD %s s%lu rec%lu drop%lu | WiFi %s TRC ok%lu f%lu | BAT %ld%%%s\n",
                static_cast<unsigned long>(g_stats.imuRateX10.load() / 10),
                static_cast<unsigned long>(g_stats.imuRateX10.load() % 10), g_stats.imuStill.load() ? "QUIETO" : "MOV",
                g_stats.sdOnline.load() ? "ok" : "--", static_cast<unsigned long>(g_stats.logSession.load()),
                static_cast<unsigned long>(g_stats.logRecords.load()),
                static_cast<unsigned long>(g_stats.logDropped.load()),
                !g_stats.wifiEnabled.load() ? "off" : (g_stats.wifiConnected.load() ? "ok" : "--"),
                static_cast<unsigned long>(g_stats.traccarOk.load()),
                static_cast<unsigned long>(g_stats.traccarFail.load()), static_cast<long>(g_stats.battPercent.load()),
                g_stats.vbusIn.load() ? " USB" : "");
}

void DisplayManager::taskLoop() {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastStatus = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(cfg::ui::kRefreshPeriodMs));

    // Botón. Las marcas llevan el tiempo exacto tomado en la interrupción.
    uint32_t markNum = 0;
    int64_t markUs = 0;
    for (button::Event ev = button::poll(markNum, markUs); ev != button::Event::None;
         ev = button::poll(markNum, markUs)) {
      if (ev == button::Event::Mark) {
        LogRecord rec{};
        rec.type = LogType::Mark;
        rec.ev.tUs = markUs;
        rec.ev.seq = markNum;
        GnssFix fix{};
        rec.ev.value = (xQueuePeek(g_latestFix, &fix, 0) == pdTRUE) ? fix.itowMs : 0;
        logPush(rec);
        g_stats.marks.store(markNum);
        s_lastMarkMs = millis();
        s_lastMarkNum = markNum;
        Serial.printf("[MARCA] #%lu\n", static_cast<unsigned long>(markNum));
      } else if (ev == button::Event::LongPress) {
#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
        radiolink::requestNextProfile();
#endif
      }
    }

    // Una vez por segundo: batería (I2C de la PMU) y resumen por consola.
    if (millis() - lastStatus >= cfg::ui::kStatusPeriodMs) {
      lastStatus = millis();
      const BatteryStatus b = g_pmu.readBattery();
      g_stats.battPercent.store(b.percent);
      g_stats.battMv.store(b.battMv);
      g_stats.vbusIn.store(b.vbusIn);
      g_stats.charging.store(b.charging);
      printStatusLine();
    }

    if (online_) {
      drawStatus();
    }
  }
}
