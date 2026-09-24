#include "pmu_manager.h"

#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

#include "board_pins.h"

PmuManager g_pmu;

static XPowersPMU s_pmu;

// Los métodos genéricos por canal (enablePowerOutput, setPowerChannelVoltage)
// son públicos en la interfaz común de XPowersLib y protegidos en la clase
// concreta, por eso se usan a través de esta referencia.
static XPowersLibInterface& s_rails = s_pmu;

// Mapa de rieles del T-Beam Supreme (LoRaBoards.cpp de LilyGo):
//   DCDC1  3,3 V  ESP32-S3. Arranca encendido y NUNCA se toca.
//   ALDO1  3,3 V  Sensores I2C (QMC6310, BME280)
//   ALDO2  3,3 V  Sensores (IMU QMI8658)
//   ALDO3  3,3 V  Radio SX1262
//   ALDO4  3,3 V  GNSS MAX-M10S
//   BLDO1  3,3 V  microSD
//   BLDO2  3,3 V  microSD
//   DCDC3..5      Conector M.2 externo, no se usa. Se deja como está.
static constexpr uint16_t kRail3V3 = 3300;

bool PmuManager::begin() {
  // XPowersLib inicializa Wire1 con los pines que le pasamos.
  if (!s_pmu.begin(Wire1, AXP2101_SLAVE_ADDRESS, pins::kPmuSda, pins::kPmuScl)) {
    online_ = false;
    return false;
  }

  // Protección del riel del micro: si algún código intentara apagarlo, la
  // librería lo rechaza.
  s_rails.setProtectedChannel(XPOWERS_DCDC1);

  // Solo en arranque en frío se apagan y reencienden los rieles de sensores y
  // microSD. LilyGo lo hace para liberar el bus si algún periférico quedó
  // trabado sosteniendo SDA o MISO de un arranque anterior.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    s_rails.disablePowerOutput(XPOWERS_ALDO1);
    s_rails.disablePowerOutput(XPOWERS_ALDO2);
    s_rails.disablePowerOutput(XPOWERS_BLDO1);
    delay(250);  // Solo en setup: tiempo de descarga de los rieles
  }

  // GNSS y radio: son los dos rieles críticos del nodo.
  s_rails.setPowerChannelVoltage(XPOWERS_ALDO4, kRail3V3);
  s_rails.enablePowerOutput(XPOWERS_ALDO4);
  s_rails.setPowerChannelVoltage(XPOWERS_ALDO3, kRail3V3);
  s_rails.enablePowerOutput(XPOWERS_ALDO3);

  // Sensores e IMU.
  s_rails.setPowerChannelVoltage(XPOWERS_ALDO1, kRail3V3);
  s_rails.enablePowerOutput(XPOWERS_ALDO1);
  s_rails.setPowerChannelVoltage(XPOWERS_ALDO2, kRail3V3);
  s_rails.enablePowerOutput(XPOWERS_ALDO2);

  // microSD.
  s_rails.setPowerChannelVoltage(XPOWERS_BLDO1, kRail3V3);
  s_rails.enablePowerOutput(XPOWERS_BLDO1);
  s_rails.setPowerChannelVoltage(XPOWERS_BLDO2, kRail3V3);
  s_rails.enablePowerOutput(XPOWERS_BLDO2);

  // Canales que no se usan en esta placa.
  s_rails.disablePowerOutput(XPOWERS_DCDC2);
  s_rails.disablePowerOutput(XPOWERS_DLDO1);
  s_rails.disablePowerOutput(XPOWERS_DLDO2);

  // Medición de batería, VBUS y sistema por el ADC de la PMU.
  s_pmu.enableBattDetection();
  s_pmu.enableBattVoltageMeasure();
  s_pmu.enableVbusVoltageMeasure();
  s_pmu.enableSystemVoltageMeasure();

  // Cargador para una 18650: 4,2 V de fin de carga y 500 mA, un valor
  // conservador que cualquier celda soporta y que el USB entrega sin problema.
  s_pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
  s_pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

  // No usamos interrupciones de la PMU todavía.
  s_pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  s_pmu.clearIrqStatus();

  s_pmu.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

  // Verificación: si los rieles críticos no quedaron encendidos, el GNSS o la
  // radio no van a responder y el diagnóstico sería engañoso.
  online_ = s_rails.isPowerChannelEnable(XPOWERS_ALDO3) && s_rails.isPowerChannelEnable(XPOWERS_ALDO4);
  return online_;
}

void PmuManager::setLed(LedPattern pattern) {
  if (!online_) {
    return;
  }
  switch (pattern) {
    case LedPattern::ChargeControlled:
      s_pmu.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
      break;
    case LedPattern::Off:
      s_pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);
      break;
    case LedPattern::On:
      s_pmu.setChargingLedMode(XPOWERS_CHG_LED_ON);
      break;
    case LedPattern::BlinkSlow:
      s_pmu.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
      break;
    case LedPattern::BlinkFast:
      s_pmu.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
      break;
  }
}

BatteryStatus PmuManager::readBattery() {
  BatteryStatus st{};
  if (!online_) {
    st.percent = -1;
    return st;
  }
  st.batteryPresent = s_pmu.isBatteryConnect();
  st.vbusIn = s_pmu.isVbusIn();
  st.charging = s_pmu.isCharging();
  st.percent = st.batteryPresent ? s_pmu.getBatteryPercent() : -1;
  st.battMv = st.batteryPresent ? s_pmu.getBattVoltage() : 0;
  st.vbusMv = s_pmu.getVbusVoltage();
  st.sysMv = s_pmu.getSystemVoltage();
  return st;
}
