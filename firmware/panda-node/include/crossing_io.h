#pragma once
// =============================================================================
// Entradas y salidas físicas del nodo cruce.
//
// Salidas (todas activas en alto, con pull-down externo, ver board_pins.h):
//   PANDA libre      Relé energizado solo cuando PANDA ve vía libre. Sin
//                    energía, en reinicio o con el micro colgado queda
//                    desenergizado: eso es un PEDIDO DE CIERRE.
//   PANDA operativo  Relé energizado solo cuando el nodo está sano. Si cae, el
//                    controlador de barrera ignora a PANDA y vuelve al
//                    comportamiento actual (solo circuito de vía).
//   Señal peatonal   LED rojo "CRUCE NO SEGURO". Nunca hay verde.
//   Sonido           Un toque por segundo mientras el cruce está NO SEGURO.
//
// Entrada:
//   Circuito de vía  Optoacoplador. Conduciendo = LIBRE. Abierto o cable
//                    cortado = OCUPADA.
//
// Vigilancia: la tarea de decisión (core 1) refresca las salidas cada 50 ms.
// Un timer que corre en el core 0 verifica ese refresco: si pasan más de
// 300 ms sin él, fuerza PANDA libre y PANDA operativo a cero y enciende la
// señal peatonal. Es una primera barrera hasta sumar el watchdog externo
// TPL5010 (acción del DFMEA contra el bloqueo del micro).
// =============================================================================

#include <cstdint>

namespace crossio {

struct Outputs {
  bool pandaLibre;
  bool pandaOk;
  bool pedestrian;
  bool sound;
};

// Configura los pines en estado seguro y arranca el vigilante. Se llama al
// principio del setup(), antes que cualquier otra cosa.
void begin();

// Aplica las salidas y refresca el latido para el vigilante.
void apply(const Outputs& out);

// Lectura cruda de la entrada del circuito de vía: true = OCUPADA.
bool readTrackOccupied();

// Cantidad de veces que el vigilante tuvo que intervenir.
uint32_t watchdogTrips();

}  // namespace crossio
