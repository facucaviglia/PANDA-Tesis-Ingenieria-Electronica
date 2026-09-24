#pragma once
// =============================================================================
// Estado compartido entre tareas.
//
// Reglas de concurrencia:
//  - Los productores del core 1 (GNSS, IMU) nunca se bloquean. Publican con
//    colas de timeout cero y, si la cola está llena, cuentan la pérdida.
//  - El último fix se publica en una cola de un solo elemento (xQueueOverwrite)
//    y los lectores lo copian con xQueuePeek. Es atómico y no necesita mutex.
//  - Los contadores de estado son std::atomic de 32 bits, que en el Xtensa del
//    ESP32-S3 son libres de locks.
// =============================================================================

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Solución de navegación del GNSS, tomada del UBX NAV-PVT.
struct GnssFix {
  int64_t tRxUs;       // esp_timer_get_time() al terminar de recibir el mensaje
  uint32_t itowMs;     // Tiempo GPS de la semana de la época de navegación
  uint32_t unixS;      // Tiempo UTC en segundos Unix, 0 si la fecha no es válida
  int32_t latE7;       // Grados * 1e7
  int32_t lonE7;       // Grados * 1e7
  int32_t hMslMm;      // Altura sobre el nivel medio del mar
  int32_t gSpeedMms;   // Velocidad sobre el suelo
  int32_t headMotE5;   // Rumbo del movimiento, grados * 1e5
  uint32_t hAccMm;     // Precisión horizontal estimada
  uint32_t sAccMms;    // Precisión de velocidad estimada
  uint32_t headAccE5;  // Precisión de rumbo estimada
  uint16_t pDopE2;     // PDOP * 100
  uint8_t fixType;     // 0 sin fix, 2 2D, 3 3D, 4 GNSS+DR, 5 solo tiempo
  uint8_t numSv;       // Satélites usados en la solución
  bool fixOk;          // Fix 2D/3D válido dentro de las máscaras del receptor
  bool timeValid;      // Fecha y hora UTC resueltas
};

// Una muestra de la IMU en unidades físicas.
struct ImuSample {
  int64_t tUs;         // Tiempo estimado de la muestra (ver imu_manager.cpp)
  float ax, ay, az;    // m/s²
  float gx, gy, gz;    // °/s
};

// Un beacon transmitido por el nodo tren.
struct TxLog {
  int64_t tStartUs;    // Instante local en que se ordenó la transmisión
  uint32_t counter;
  uint32_t itowMs;     // iTOW del fix que viaja en el beacon
  int32_t txAgeMs;     // Antigüedad del fix al transmitir. INT32_MIN si no hay tiempo
  uint32_t airUs;      // Duración medida de la transmisión (hasta TX_DONE)
  int16_t status;      // Código de RadioLib
  uint8_t flags;       // BeaconFlag
  uint8_t slot;        // Ranura TDMA usada, 255 si transmitió sin sincronía
  uint8_t profile;
  uint8_t timeQ;       // TimeQuality del tren al transmitir
  int8_t powerDbm;
};

// Resultado de la recepción de un paquete en el nodo cruce.
enum class RxResult : uint8_t {
  Ok = 0,        // Autenticado, nuevo y fresco: dato válido
  Unverified,    // Autenticado, pero sin tiempo GPS en alguna punta: solo métrica
  CrcError,
  BadFormat,
  BadTag,        // Falló el CMAC: no viene de un nodo con la clave
  Replay,        // Contador repetido o viejo
  Stale,         // Más viejo que la ventana de 2 s
  Future,        // Tiempo del beacon adelantado respecto del cruce
};

struct RxLog {
  int64_t tEndUs;      // Instante local del fin de recepción (IRQ)
  uint32_t counter;
  uint32_t itowMs;
  int32_t ageMs;       // Antigüedad del dato al recibirlo. INT32_MIN si no se sabe
  int32_t latE7;
  int32_t lonE7;
  float rssiDbm;
  float snrDb;
  float freqErrHz;
  float distM;         // Distancia tren a cruce, NAN si falta algún fix
  uint16_t nodeId;
  uint16_t speedCms;
  uint16_t headingCdeg;
  uint16_t hAccCm;
  uint16_t gap;        // Beacons perdidos entre este y el anterior del mismo tren
  uint8_t result;      // RxResult
  uint8_t flags;
  uint8_t numSv;
  uint8_t timeQ;       // TimeQuality del cruce
  uint8_t profile;
  uint8_t injected;    // 1 si vino del simulador por USB
};

// Una evaluación de la lógica del cruce para un tren (fase 3). Se registra
// cada vez que llega un beacon válido de ese tren y en cada cambio de estado.
struct DecisionLog {
  int64_t tUs;
  float distM;         // Distancia tren a cruce, proyectada al instante actual
  float speedMps;
  float closingMps;    // Velocidad de acercamiento al cruce (negativa si se aleja)
  float etaCvS;        // ETA a velocidad constante (NAN si no se acerca)
  float etaMinS;       // ETA mínimo suponiendo aceleración máxima
  int32_t ageMs;       // Antigüedad del último dato válido
  uint16_t trainId;
  uint8_t state;       // Estado del cruce (CrossState)
  uint8_t reason;      // Motivo principal (CrossReason)
  uint8_t phase;       // Fase de este tren (TrainPhase)
  uint8_t alerting;    // Este tren sostiene el NO SEGURO
  uint8_t outputs;     // Bits: 1 PANDA libre, 2 PANDA operativo, 4 señal peatonal, 8 vía ocupada, 16 barrera baja
};

enum class LogType : uint8_t {
  Gnss = 0,
  Imu,
  Pps,    // Flanco del 1PPS. value = intervalo con el anterior en µs
  Mark,   // Marca manual del botón. seq = número de marca
  Note,   // Evento del sistema. value = código de nota
  Tx,
  Rx,
  Decision,
};

enum class NoteCode : uint32_t {
  Boot = 1,
  GnssOnline = 2,
  GnssLost = 3,
  ImuOnline = 4,
  WifiUp = 5,
  WifiDown = 6,
  ProfileChange = 7,   // value = índice del perfil
  PowerChange = 8,     // value = dBm
  CounterReserve = 9,  // value = base del bloque reservado
  StateChange = 10,    // value = estado << 8 | motivo
  TrainNew = 11,       // value = id del tren
  TrainForgotten = 12, // value = id del tren
  Passage = 13,        // value = id << 16 | distancia mínima en m
  TrackChange = 14,    // value = 1 ocupada, 0 libre
  TrackWithoutPanda = 15, // La vía se ocupó con PANDA diciendo vía libre. value = id del tren más cercano o 0
  SilentRelease = 16,  // value = id << 8 | 1 por tiempo, 2 por circuito de vía
  RefSaved = 17,       // value = cantidad de fixes promediados
  WatchdogTrip = 18,   // La tarea de decisión dejó de refrescar las salidas
};

struct LogEvent {
  int64_t tUs;
  uint32_t seq;
  uint32_t value;
};

struct LogRecord {
  LogType type;
  union {
    GnssFix gnss;
    ImuSample imu;
    LogEvent ev;
    TxLog tx;
    RxLog rx;
    DecisionLog dec;
  };
};

// Contadores y banderas de estado. Los leen la pantalla, la consola y la
// telemetría. Los escribe cada módulo dueño del dato.
struct SystemStats {
  std::atomic<bool> pmuOnline{false};
  std::atomic<bool> gnssOnline{false};
  std::atomic<bool> imuOnline{false};
  std::atomic<bool> radioOnline{false};
  std::atomic<bool> sdOnline{false};
  std::atomic<bool> wifiEnabled{false};
  std::atomic<bool> wifiConnected{false};

  std::atomic<uint32_t> gnssBaud{0};
  std::atomic<uint32_t> gnssFixes{0};
  std::atomic<uint32_t> gnssRateX10{0};   // Hz * 10
  std::atomic<uint32_t> ppsCount{0};
  std::atomic<int32_t> ppsErrorUs{0};     // Intervalo medido menos 1 s

  std::atomic<uint32_t> imuSamples{0};
  std::atomic<uint32_t> imuRateX10{0};    // Hz * 10
  std::atomic<bool> imuStill{false};
  std::atomic<uint32_t> imuAccNormMg{0};  // |a| promedio en mg

  std::atomic<uint32_t> logSession{0};
  std::atomic<uint32_t> logRecords{0};
  std::atomic<uint32_t> logDropped{0};
  std::atomic<uint32_t> logKBytes{0};

  std::atomic<uint32_t> traccarOk{0};
  std::atomic<uint32_t> traccarFail{0};
  std::atomic<uint32_t> traccarLastOkMs{0};

  std::atomic<uint32_t> marks{0};

  std::atomic<uint32_t> nodeId{0};
  std::atomic<uint32_t> profile{0};
  std::atomic<int32_t> txPowerDbm{0};
  std::atomic<uint32_t> linkAliveMs{0};  // Última vuelta de la tarea de enlace

  std::atomic<int32_t> battPercent{-1};
  std::atomic<uint32_t> battMv{0};
  std::atomic<bool> vbusIn{false};
  std::atomic<bool> charging{false};
};

extern SystemStats g_stats;

// Último fix publicado (cola de un elemento).
extern QueueHandle_t g_latestFix;

// Cola de registros hacia la microSD.
extern QueueHandle_t g_logQueue;

// Crea las colas globales. Se llama una vez al principio del setup().
bool systemStateInit();

// Encola un registro sin bloquear. Si no hay lugar, cuenta la pérdida.
bool logPush(const LogRecord& rec);

// Atajos para eventos.
void logNote(NoteCode code, uint32_t value = 0);
