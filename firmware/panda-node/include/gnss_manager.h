#pragma once
// =============================================================================
// GNSS u-blox MAX-M10S por UBX binario.
//
// Se configura en cada arranque (solo en RAM del módulo, nunca en su flash):
//   - UART a 460800 baud
//   - Solo protocolo UBX de salida, sin NMEA
//   - 10 Hz de navegación
//   - Modelo dinámico automotriz
//   - NAV-PVT automático en cada época
//
// El 1PPS se captura por interrupción para medir el reloj y, en la fase 2,
// alinear las ranuras TDMA de la radio.
// =============================================================================

#include <cstdint>

class GnssManager {
 public:
  // Busca el módulo, lo configura y deja la UART lista. Bloquea unos segundos
  // como máximo. Se llama desde setup().
  bool begin();

  // Crea la tarea que procesa la UART, publica los fix y vigila la salud del
  // receptor. Si begin() falló, la tarea reintenta periódicamente.
  void startTask();

 private:
  static void taskEntry(void* arg);
  void taskLoop();
  bool configure();
};

extern GnssManager g_gnss;

// Conversión de fecha UTC a segundos Unix sin depender de la zona horaria.
uint32_t utcToUnix(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

// -----------------------------------------------------------------------------
// Reloj GPS local
//
// Relaciona el reloj del ESP32 (esp_timer, µs desde el arranque) con el tiempo
// GPS de la semana (TOW). Los dos nodos comparten así una misma escala de
// tiempo, que se usa para ubicar las ranuras TDMA y para medir en el cruce la
// antigüedad exacta de cada beacon.
//
//   Pps     el ancla es un flanco del 1PPS, error de microsegundos
//   Coarse  el ancla es la llegada del último NAV-PVT, que llega 30 a 50 ms
//           después de su época: sirve para mostrar, no para el TDMA fino
//   Pc      tiempo dado por la PC (simulador de banco), solo si no hay GNSS
//   None    no hay tiempo GPS
// -----------------------------------------------------------------------------
enum class TimeQuality : uint8_t { None = 0, Coarse = 1, Pps = 2, Pc = 3 };

constexpr int64_t kGpsWeekUs = 604800LL * 1000000LL;

// TOW en µs que corresponde al instante local tLocalUs.
TimeQuality gpsTowAt(int64_t tLocalUs, int64_t& towUs);

// a - b entre dos TOW, contemplando el cambio de semana. Resultado en
// [-semana/2, semana/2).
int64_t towDiffUs(int64_t a, int64_t b);

const char* timeQualityName(TimeQuality q);

// Ancla de tiempo que manda el simulador de la PC (comando T). Solo se usa si
// no hay tiempo del GNSS y si cfg::sim::kAllowPcTime está habilitado.
void setPcTime(int64_t towUs);
