#pragma once
// =============================================================================
// Pinout real del LILYGO T-Beam Supreme (V3), variante 915 MHz con SX1262 y
// u-blox MAX-M10S.
//
// Fuentes cruzadas (verificado el 24-sep-2026):
//  - utilities.h y LoRaBoards.cpp del repo oficial Xinyuan-LilyGO/LilyGo-LoRa-Series
//  - variants/esp32s3/tbeam-s3-core/variant.h de Meshtastic
// La wiki de LilyGo muestra un ejemplo con pines de otra placa, no usarla.
//
// Ojo con el GPIO 42: es el SDA del bus de la PMU. La placa NO tiene un LED de
// usuario en ningún GPIO. El único LED controlable es el de carga de la AXP2101.
// =============================================================================

#include <cstdint>

namespace pins {

// --- Bus I2C 0 (Wire): OLED SH1106, magnetómetro QMC6310, BME280 ------------
constexpr int kI2cSda = 17;
constexpr int kI2cScl = 18;

// --- Bus I2C 1 (Wire1): PMU AXP2101 y RTC PCF8563 ---------------------------
constexpr int kPmuSda = 42;
constexpr int kPmuScl = 41;
constexpr int kPmuIrq = 40;

// --- GNSS u-blox MAX-M10S (UART1 del ESP32-S3) ------------------------------
constexpr int kGnssRx     = 9;  // RX del ESP32, recibe el TX del módulo
constexpr int kGnssTx     = 8;  // TX del ESP32, va al RX del módulo
constexpr int kGnssPps    = 6;  // Pulso por segundo alineado a tiempo GPS
constexpr int kGnssWakeup = 7;  // Se mantiene en alto para que el módulo no duerma

// --- Radio Semtech SX1262 (bus SPI dedicado, periférico HSPI) ---------------
constexpr int kRadioSck  = 12;
constexpr int kRadioMiso = 13;
constexpr int kRadioMosi = 11;
constexpr int kRadioCs   = 10;
constexpr int kRadioRst  = 5;
constexpr int kRadioDio1 = 1;
constexpr int kRadioBusy = 4;

// --- Bus SPI compartido (periférico FSPI): microSD e IMU QMI8658 ------------
constexpr int kSpiSck  = 36;
constexpr int kSpiMiso = 37;
constexpr int kSpiMosi = 35;
constexpr int kSdCs    = 47;
constexpr int kImuCs   = 34;
constexpr int kImuInt  = 33;

// --- Otros ------------------------------------------------------------------
constexpr int kButton = 0;   // Botón BOOT, activo en bajo, con pull-up en placa
constexpr int kRtcInt = 14;

// --- Entradas y salidas del nodo cruce (header derecho de la placa) ---------
// GPIO libres del header según el pinout oficial de LilyGo: 2, 3, 21, 38, 39,
// 45, 46 y 48. Se evitan 45 y 46 porque son pines de arranque del ESP32-S3:
// un relé o un pull-up externo en ellos puede impedir que la placa arranque.
//
// Todas las salidas se cablean con un pull-down externo (10 kΩ a GND) y
// manejan el relé o el LED a través de un transistor u optoacoplador. Así,
// durante un reinicio, sin alimentación o con el micro colgado, la salida
// queda en bajo, que es el estado seguro de cada una.
constexpr int kOutPandaLibre = 21;  // Relé "PANDA ve vía libre". Desenergizado = pedido de cierre
constexpr int kOutPandaOk = 38;     // Relé "PANDA operativo". Desenergizado = ignorar PANDA
constexpr int kOutPedestrian = 39;  // Señal peatonal "CRUCE NO SEGURO" (LED rojo)
constexpr int kOutBuzzer = 48;      // Aviso sonoro (PWM hacia un buzzer o amplificador)

// Circuito de vía, por optoacoplador. Opto conduciendo = pin en bajo = vía
// LIBRE. Opto abierto o cable cortado = pin en alto por el pull-up = vía
// OCUPADA. Así una falla del cableado nunca se lee como vía libre.
constexpr int kInTrackCircuit = 2;

}  // namespace pins

namespace board {

// El SX1262 de esta placa usa un TCXO alimentado desde DIO3 a 1,8 V, y el
// switch de antena TX/RX lo maneja el propio chip con DIO2.
constexpr float kRadioTcxoVoltage = 1.8f;

constexpr uint8_t kOledI2cAddr = 0x3C;

}  // namespace board
