#include "radio_manager.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <esp_timer.h>

#include "board_pins.h"
#include "system_state.h"

RadioManager g_radio;

// La radio tiene su propio periférico SPI (HSPI) para no compartir bus con la
// microSD. Una escritura lenta de la tarjeta nunca puede demorar un beacon.
static SPIClass s_radioSpi(HSPI);
static SX1262 s_radio = new Module(pins::kRadioCs, pins::kRadioDio1, pins::kRadioRst, pins::kRadioBusy, s_radioSpi,
                                   SPISettings(8000000, MSBFIRST, SPI_MODE0));

static TaskHandle_t s_irqTask = nullptr;
static volatile int64_t s_irqUs = 0;

// DIO1 se activa con TX_DONE en transmisión y con RX_DONE, CRC_ERR o timeout
// en recepción. La ISR solo toma el tiempo y avisa: leer el chip por SPI desde
// una ISR bloquearía demasiado.
static void IRAM_ATTR onDio1() {
  s_irqUs = esp_timer_get_time();
  if (s_irqTask != nullptr) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(s_irqTask, kNotifyRadioIrq, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

int16_t RadioManager::begin(const cfg::radio::LoraProfile& profile, int8_t powerDbm) {
  s_radioSpi.begin(pins::kRadioSck, pins::kRadioMiso, pins::kRadioMosi, pins::kRadioCs);

  // Parámetros LoRa:
  //  - TCXO a 1,8 V por DIO3: sin esto el chip no arranca el oscilador.
  //  - Regulador DC-DC interno (useRegulatorLDO = false): menos consumo.
  int16_t st = s_radio.begin(cfg::radio::kFrequencyMHz, profile.bwKHz, profile.sf, profile.cr, cfg::radio::kSyncWord,
                             powerDbm, cfg::radio::kPreambleSymbols, board::kRadioTcxoVoltage, false);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO] begin() falló, código %d\n", st);
    return st;
  }

  // El switch de antena TX/RX de la placa lo maneja DIO2 del SX1262.
  st = s_radio.setDio2AsRfSwitch(true);
  if (st != RADIOLIB_ERR_NONE) return st;

  // 140 mA es el límite necesario para llegar a 22 dBm sin que el PA se recorte.
  st = s_radio.setCurrentLimit(cfg::radio::kCurrentLimitMa);
  if (st != RADIOLIB_ERR_NONE) return st;

  // CRC de 2 bytes sobre el payload.
  st = s_radio.setCRC(2);
  if (st != RADIOLIB_ERR_NONE) return st;

  // Header implícito: el beacon siempre mide lo mismo, el receptor ya conoce el
  // largo, la tasa de código y el CRC. Ahorra unos 1,3 ms por paquete.
  st = s_radio.implicitHeader(cfg::radio::kBeaconLength);
  if (st != RADIOLIB_ERR_NONE) return st;

  s_radio.setDio1Action(onDio1);

  configured_ = true;
  g_stats.radioOnline.store(true);
  Serial.printf("[RADIO] SX1262 OK, %.1f MHz, %s, %d dBm, ToA beacon %lu us\n",
                static_cast<double>(cfg::radio::kFrequencyMHz), profile.name, powerDbm,
                static_cast<unsigned long>(timeOnAirUs(cfg::radio::kBeaconLength)));
  return RADIOLIB_ERR_NONE;
}

int16_t RadioManager::applyProfile(const cfg::radio::LoraProfile& profile) {
  int16_t st = s_radio.standby();
  if (st != RADIOLIB_ERR_NONE) return st;
  st = s_radio.setBandwidth(profile.bwKHz);
  if (st != RADIOLIB_ERR_NONE) return st;
  st = s_radio.setSpreadingFactor(profile.sf);
  if (st != RADIOLIB_ERR_NONE) return st;
  st = s_radio.setCodingRate(profile.cr);
  if (st != RADIOLIB_ERR_NONE) return st;
  // La tasa de código y el largo viajan en el header, que no existe en modo
  // implícito: hay que volver a fijarlos en ambos extremos.
  return s_radio.implicitHeader(cfg::radio::kBeaconLength);
}

int16_t RadioManager::setPower(int8_t dbm) {
  return s_radio.setOutputPower(dbm);
}

int16_t RadioManager::setFrequency(float mhz) {
  return s_radio.setFrequency(mhz);
}

void RadioManager::setIrqTask(TaskHandle_t task) {
  s_irqTask = task;
}

int64_t RadioManager::lastIrqUs() const {
  return s_irqUs;
}

int16_t RadioManager::standbyXosc() {
  return s_radio.standby(RADIOLIB_SX126X_STANDBY_XOSC);
}

int16_t RadioManager::startTransmit(const uint8_t* data, size_t len) {
  return s_radio.startTransmit(data, len);
}

int16_t RadioManager::finishTransmit() {
  return s_radio.finishTransmit();
}

int16_t RadioManager::startReceive() {
  return s_radio.startReceive();
}

int16_t RadioManager::readPacket(uint8_t* buf, size_t len, PacketInfo& info) {
  const int16_t st = s_radio.readData(buf, len);
  info.rssiDbm = s_radio.getRSSI(true);
  info.snrDb = s_radio.getSNR();
  info.freqErrHz = s_radio.getFrequencyError();
  return st;
}

float RadioManager::channelRssi() {
  return s_radio.getRSSI(false);
}

void RadioManager::sleep() {
  if (configured_) {
    s_radio.sleep(true);
  }
}

uint32_t RadioManager::timeOnAirUs(size_t len) {
  return configured_ ? static_cast<uint32_t>(s_radio.getTimeOnAir(len)) : 0;
}
