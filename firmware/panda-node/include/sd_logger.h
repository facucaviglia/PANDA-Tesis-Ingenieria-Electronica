#pragma once
// =============================================================================
// Registrador en microSD (caja negra, E-13).
//
// Cada encendido abre una sesión nueva en una carpeta PANDA_NNNN con tres CSV:
//   gnss.csv    un renglón por NAV-PVT (10 Hz)
//   imu.csv     un renglón por muestra de la IMU (~224 Hz)
//   events.csv  1PPS, marcas del botón y eventos del sistema
//
// Todos comparten la base de tiempo t_us (microsegundos desde el arranque del
// ESP32), así se pueden alinear entre sí. gnss.csv además trae el tiempo GPS
// (itow_ms) y UTC (unix_s) para alinear con el otro nodo o con un video.
//
// Si no hay tarjeta el sistema funciona igual. Se reintenta montar cada 5 s,
// así que se puede insertar con la placa encendida.
// =============================================================================

class SdLogger {
 public:
  void startTask();

 private:
  static void taskEntry(void* arg);
  void taskLoop();
};

extern SdLogger g_logger;
