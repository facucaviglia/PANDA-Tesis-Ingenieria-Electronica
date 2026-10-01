#pragma once
// =============================================================================
// Entradas y salidas físicas del nodo cruce, y maqueta de barrera.
//
// Salidas de PANDA (activas en alto, con pull-down externo, ver board_pins.h):
//   PANDA libre      Contacto cerrado solo cuando PANDA ve vía libre. Sin
//                    energía, en reinicio o con el micro colgado queda
//                    abierto: eso es un PEDIDO DE CIERRE.
//   PANDA operativo  Contacto cerrado solo cuando el nodo está sano. Si cae, el
//                    controlador de barrera ignora a PANDA y vuelve al
//                    comportamiento actual (solo circuito de vía).
//   Señal peatonal   LED rojo "CRUCE NO SEGURO". Nunca hay verde.
//   OTRO TREN        LED del Anexo XII: hay más de un tren en peligro.
//   Sonido           Un toque por segundo mientras el cruce está NO SEGURO,
//                    dos por segundo con OTRO TREN.
//
// Entrada:
//   Circuito de vía  Optoacoplador. Conduciendo = LIBRE. Abierto o cable
//                    cortado = OCUPADA. Sin la entrada cableada se simula.
//
// Controlador de barrera (maqueta, cfg::crossing::kBarrierOnBoard). Es un
// módulo aparte que solo ve los dos contactos y el circuito de vía, igual que
// el controlador real, con la regla acordada:
//   con PANDA operativo:   baja si PANDA pide cierre
//   sin PANDA operativo:   baja si el circuito de vía está ocupado (como hoy)
// Secuencia del Anexo XII: fonoluminosa (luces alternadas cada 0,5 s y
// campana), bajada del brazo, brazo abajo hasta que se levanta el pedido, y
// subida con las señales apagadas. Si el pedido vuelve mientras sube, baja de
// inmediato con las señales encendidas.
//
// Vigilancia: la tarea de decisión (core 1) refresca las salidas cada 50 ms.
// Un timer que corre en el core 0 verifica ese refresco: si pasan más de
// 300 ms sin él, abre los dos contactos (pedido de cierre y PANDA no
// operativo) y enciende la señal peatonal. El mismo timer corre la maqueta y
// el sonido. Es una primera barrera hasta sumar el watchdog externo TPL5010.
// =============================================================================

#include <cstdint>

namespace crossio {

struct Outputs {
  bool pandaLibre;
  bool pandaOk;
  bool pedestrian;
  bool sound;        // Aviso sonoro de PANDA
  bool otherTrain;   // Más de un tren en peligro
  bool muted;        // Silencia todo el sonido (banco)
};

enum class BarrierPhase : uint8_t {
  Arriba = 0,   // Brazo vertical, señales apagadas
  Fono,         // Fonoluminosa: luces y campana antes de bajar
  Bajando,
  Abajo,
  Subiendo,
};

// Configura los pines en estado seguro y arranca el timer de vigilancia,
// maqueta y sonido. Se llama al principio del setup(), antes que nada.
void begin();

// Aplica las salidas y refresca el latido para el vigilante.
void apply(const Outputs& out);

// Circuito de vía: lectura cruda de la entrada física (true = OCUPADA), o el
// valor simulado si la entrada no está habilitada en config.h.
bool readTrackOccupied();
bool trackOccupied();
void setSimulatedTrack(bool occupied);

// Estado de la maqueta. Con la barrera externa devuelve Arriba.
BarrierPhase barrierPhase();
const char* barrierPhaseName(BarrierPhase p);

// Cantidad de veces que el vigilante tuvo que intervenir.
uint32_t watchdogTrips();

}  // namespace crossio
