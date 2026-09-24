#pragma once
// =============================================================================
// IMU QMI8658 (6 ejes) por el bus SPI compartido con la microSD.
//
// Rol en PANDA: no es para navegar. Sirve para detectar si el tren está quieto
// o en marcha, verificar que el GNSS no esté mintiendo (plausibilidad) y cubrir
// huecos cortos del GNSS. En la fase 1 solo se registra y se calcula un
// detector de quietud simple para ajustar después con datos reales.
//
// Se usa la FIFO interna del sensor: aunque la microSD retenga el bus SPI
// durante una escritura lenta, el sensor sigue acumulando muestras y no se
// pierde ninguna.
// =============================================================================

class ImuManager {
 public:
  // Requiere que el bus SPI compartido ya esté inicializado.
  bool begin();
  void startTask();

 private:
  static void taskEntry(void* arg);
  void taskLoop();
};

extern ImuManager g_imu;
