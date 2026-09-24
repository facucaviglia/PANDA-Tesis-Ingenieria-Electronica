#pragma once
// =============================================================================
// Radio LoRa SX1262: capa fina sobre RadioLib.
//
// Solo la usa la tarea de enlace (core 1). Ninguna otra tarea toca la radio, así
// que no necesita mutex. La interrupción DIO1 (TX_DONE o RX_DONE) guarda el
// instante exacto y despierta a la tarea dueña con una notificación.
//
// IMPORTANTE: nunca transmitir con el SX1262 sin antena conectada.
// =============================================================================

#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

// Bit de notificación que usa la ISR de la radio.
constexpr uint32_t kNotifyRadioIrq = 1u << 0;

struct PacketInfo {
  float rssiDbm;
  float snrDb;
  float freqErrHz;
};

class RadioManager {
 public:
  // Inicializa el chip con un perfil y una potencia. Devuelve el código de
  // RadioLib (0 = RADIOLIB_ERR_NONE).
  int16_t begin(const cfg::radio::LoraProfile& profile, int8_t powerDbm);

  // Cambia SF, BW y CR sin reinicializar el chip.
  int16_t applyProfile(const cfg::radio::LoraProfile& profile);
  int16_t setPower(int8_t dbm);
  int16_t setFrequency(float mhz);

  // Tarea que recibe la notificación de DIO1.
  void setIrqTask(TaskHandle_t task);
  int64_t lastIrqUs() const;

  // Espera en standby con el oscilador de cristal encendido: el transmisor
  // arranca en microsegundos, sin los ~5 ms de arranque del TCXO.
  int16_t standbyXosc();

  int16_t startTransmit(const uint8_t* data, size_t len);
  int16_t finishTransmit();

  // Recepción continua: el chip queda escuchando después de cada paquete, no
  // hace falta volver a llamarla.
  int16_t startReceive();

  // Lee el paquete recibido. Devuelve el código de RadioLib (CRC incluido).
  int16_t readPacket(uint8_t* buf, size_t len, PacketInfo& info);

  // RSSI instantáneo del canal, para el barrido. Requiere estar en recepción.
  float channelRssi();

  void sleep();

  uint32_t timeOnAirUs(size_t len);

 private:
  bool configured_ = false;
};

extern RadioManager g_radio;
