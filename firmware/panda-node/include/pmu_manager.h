#pragma once
// =============================================================================
// PMU AXP2101: rieles de alimentación, batería y LED de estado.
//
// En el T-Beam Supreme el GNSS, la radio, la IMU y la microSD cuelgan de
// reguladores de la PMU. Si la PMU no se inicializa, nada de eso tiene tensión.
// =============================================================================

#include <cstdint>

enum class LedPattern : uint8_t {
  ChargeControlled,  // El LED lo maneja el cargador (uso normal)
  Off,
  On,
  BlinkSlow,         // 1 Hz: advertencia, algo no crítico falló
  BlinkFast,         // 4 Hz: falla fatal, el nodo no está operativo
};

struct BatteryStatus {
  bool batteryPresent;
  bool vbusIn;
  bool charging;
  int percent;       // -1 si no hay batería
  uint16_t battMv;
  uint16_t vbusMv;
  uint16_t sysMv;
};

class PmuManager {
 public:
  // Inicializa el bus I2C 1, verifica que sea una AXP2101 y enciende los rieles
  // del GNSS, la radio, los sensores y la microSD. Devuelve false si la PMU no
  // responde o si algún riel crítico no quedó encendido.
  bool begin();

  // El patrón lo genera la propia PMU por hardware. Sigue titilando aunque el
  // ESP32 se cuelgue, lo que lo hace útil como indicación de falla.
  void setLed(LedPattern pattern);

  BatteryStatus readBattery();

  bool online() const { return online_; }

 private:
  bool online_ = false;
};

extern PmuManager g_pmu;
