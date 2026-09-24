#pragma once
// =============================================================================
// Enlace de seguridad por LoRa entre nodo tren y nodo cruce.
//
// Una sola tarea por nodo es dueña de la radio (core 1, la prioridad más alta
// del sistema):
//   - TREN:  en cada trama de 100 ms espera su ranura TDMA, arma el beacon con
//            el fix más reciente, lo cifra, lo autentica y lo transmite.
//   - CRUCE: escucha en recepción continua. Por cada paquete verifica CRC,
//            CMAC, contador y frescura, mide RSSI, SNR y antigüedad, y publica
//            el estado del enlace.
//
// Las demás tareas (pantalla, telemetría, consola) piden cambios de perfil,
// de potencia o un barrido de canales por notificación y leen el estado por
// colas de un elemento. Nunca tocan la radio directamente.
// =============================================================================

#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "beacon.h"
#include "config.h"

namespace radiolink {

// Estado del transmisor (nodo tren).
struct TxStatus {
  uint32_t txCount;
  uint32_t counter;      // Próximo contador a usar
  int32_t lastTxAgeMs;   // Antigüedad del fix en el último beacon. INT32_MIN si no se sabe
  uint32_t lastAirUs;    // Duración medida del último beacon en el aire
  int16_t lastStatus;    // Código de RadioLib de la última transmisión
  uint8_t timeQ;         // TimeQuality al transmitir
  uint8_t slot;          // 255 = sin sincronía, transmite con el reloj local
  uint8_t slotCount;
  uint32_t slotUs;
  uint32_t airUs;        // Tiempo en el aire teórico del perfil activo
};

// Estado del receptor (nodo cruce).
struct RxStatus {
  bool haveTrain;        // Se recibió al menos un beacon autenticado
  uint16_t nodeId;       // Tren del último beacon autenticado
  int64_t lastValidUs;   // Último beacon válido (autenticado, nuevo y fresco)
  int64_t lastAuthUs;    // Último beacon autenticado, válido o no
  BeaconData last;       // Contenido del último beacon autenticado
  int32_t ageMs;         // Antigüedad de ese beacon. INT32_MIN si no se sabe
  float rssiDbm;
  float snrDb;
  float freqErrHz;
  float perPct;          // Pérdida de beacons en la última ventana de 5 s
  float distM;           // Distancia tren a cruce, NAN si falta algún fix
  uint8_t lastResult;    // RxResult del último paquete
  uint8_t timeQ;         // TimeQuality del cruce
  uint32_t ok;
  uint32_t unverified;
  uint32_t crcError;
  uint32_t badTag;
  uint32_t replay;
  uint32_t stale;
  uint32_t other;
};

// Beacon autenticado que el enlace le pasa a la lógica de decisión del cruce.
// Solo pasan los que superaron el CMAC y el contador: los demás no prueban
// nada sobre la presencia de un tren.
struct AuthBeacon {
  BeaconData data;
  int64_t tEndUs;      // Instante local de recepción
  int32_t ageMs;       // Antigüedad verificada, INT32_MIN si no se pudo verificar
  uint8_t result;      // RxResult: Ok, Unverified, Stale o Future
  bool injected;       // Vino del simulador por USB, no por radio
};

// Cola de beacons autenticados hacia la tarea de decisión (solo en el cruce).
QueueHandle_t authBeaconQueue();

// Simulador de banco: entrega un paquete de 35 bytes como si hubiera llegado
// por radio. Pasa por la misma validación completa. Devuelve false si la
// inyección está deshabilitada o la cola está llena.
bool injectPacket(const uint8_t* pkt, size_t len, float rssiDbm, float snrDb);

// Inicializa la radio con el perfil y la potencia guardados. Se llama desde
// setup(), después de persist::begin(). Devuelve false si el chip no responde.
bool begin();

// Crea la tarea de enlace del rol compilado (tren o cruce).
void startTask();

// Pedidos desde otras tareas. Se atienden en la tarea de enlace entre paquetes.
void requestNextProfile();
void requestTogglePower();
void requestScan();

bool latestTx(TxStatus& out);
bool latestRx(RxStatus& out);

const cfg::radio::LoraProfile& activeProfile();
uint8_t activeProfileIndex();
int8_t activePowerDbm();

const char* rxResultName(uint8_t result);

}  // namespace radiolink
