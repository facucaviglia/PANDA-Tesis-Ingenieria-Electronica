#pragma once
// =============================================================================
// Lógica de decisión del nodo cruce (fase 3).
//
// Una tarea en el core 1 corre a 20 Hz. En cada vuelta:
//   1. Incorpora los beacons autenticados que dejó el enlace.
//   2. Para cada tren proyecta su posición al instante actual con la
//      antigüedad medida del dato (x + v·edad) y calcula la distancia al
//      cruce, la velocidad de acercamiento y dos ETA:
//        - a velocidad constante:   ETA_cv  = d / v_acerc
//        - mínimo (peor caso):      el menor tiempo en que el tren podría
//          llegar acelerando al máximo con un perfil de tracción real
//          (aceleración constante, después potencia constante, con tope en
//          la velocidad de la línea).
//      El ETA mínimo es el que decide. Así un tren detenido cerca o uno que
//      arranca nunca toman desprevenido al cruce.
//   3. Resuelve el estado del cruce con cierre en OR y apertura en AND (ver
//      cfg::crossing en config.h) y maneja las salidas físicas.
//
// Invariante: ninguna falla de PANDA puede apagar la señal ni dar vía libre.
// Todo lo que PANDA no puede asegurar es NO SEGURO, y si el nodo mismo no está
// sano se declara no operativo para que la barrera vuelva a su comportamiento
// actual (solo circuito de vía).
//
// La distancia es en línea recta. En una vía curva es menor que la distancia
// sobre la vía, así que el ETA sale más corto y la alerta más temprana: el
// error va siempre del lado seguro.
// =============================================================================

#include <cstdint>

enum class CrossState : uint8_t {
  Iniciando = 0,
  Apagado,     // Nada que avisar. La señal no afirma nada: no hay verde
  NoSeguro,    // Señal peatonal encendida y pedido de cierre de PANDA
  Falla,       // El nodo no puede cumplir su función: PANDA no operativo
};

enum class CrossReason : uint8_t {
  Ninguno = 0,
  TrenAproxima,     // ETA mínimo por debajo del umbral
  TrenEnZona,       // Tren dentro del radio de ocupación
  SinDatos,         // Tren en aproximación que dejó de mandar datos válidos
  SinPosicion,      // Tren que se escucha pero sin fix
  NoVerificable,    // Tren que se escucha pero sin tiempo para verificar frescura
  ViaOcupada,       // Circuito de vía ocupado
  FallaRadio,
  FallaTiempo,      // El cruce no tiene tiempo GPS: no puede validar nada
  FallaReferencia,  // No hay posición del cruce
  Arranque,
};

enum class TrainPhase : uint8_t {
  Lejos = 0,
  Aproxima,
  EnZona,
  Alejandose,
  SinDatos,
};

enum class RefSource : uint8_t { Ninguna = 0, Gnss, Guardada, Config, Pc };

struct CrossingStatus {
  CrossState state;
  CrossReason reason;
  uint16_t reasonTrain;     // Tren que causa el motivo, 0 si no aplica
  int64_t stateSinceUs;

  // Salidas y entradas
  bool pandaLibre;
  bool pandaOk;
  bool pedestrian;
  bool barrierDown;         // Controlador de barrera de referencia (simulado)
  bool trackOccupied;
  bool trackEnabled;        // Entrada física habilitada (si no, se simula)
  bool muted;

  // Referencia del cruce
  RefSource refSource;
  int32_t refLatE7;
  int32_t refLonE7;
  uint32_t refSamples;

  // Tren principal (el más crítico)
  uint8_t trainCount;
  bool haveTrain;
  uint16_t trainId;
  TrainPhase phase;
  float distM;
  float speedMps;
  float closingMps;
  float etaCvS;
  float etaMinS;
  int32_t ageMs;
  uint32_t silentMs;

  // Contadores para validación
  uint32_t passages;          // Pasos detectados (mínimo de distancia)
  uint32_t trackWithoutPanda; // Vía ocupada con PANDA diciendo libre
  uint32_t silentReleases;
  uint32_t watchdogTrips;
};

namespace crossing {

// Configura las salidas en estado seguro. Se llama al principio del setup().
void beginIo();

// Crea la tarea de decisión. Requiere el enlace ya iniciado.
void startTask();

bool latest(CrossingStatus& out);

// Pedidos desde la consola o el simulador (los atiende la tarea de decisión).
void requestSaveRef();
void requestClearRef();
void setPcRef(int32_t latE7, int32_t lonE7);
void toggleSimulatedTrack();
void toggleMute();

const char* stateName(CrossState s);
const char* reasonName(CrossReason r);
const char* phaseName(TrainPhase p);
const char* refSourceName(RefSource s);

}  // namespace crossing
