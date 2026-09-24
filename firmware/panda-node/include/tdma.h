#pragma once
// =============================================================================
// Plan TDMA: ranuras dentro de la trama de 100 ms alineada al tiempo GPS.
//
// Todos los nodos tren calculan el mismo plan a partir del tiempo en el aire
// del perfil activo, así que no hace falta coordinarlos por radio. Cada tren
// toma la ranura (nodeId módulo cantidad de ranuras).
//
//   trama n:  |<------------------------- 100 ms ------------------------->|
//             época GNSS
//             |--- offset inicial ---|ranura 0|ranura 1|ranura 2|...
//
// Complejidad: O(1) por beacon. No hay colas ni listas, solo aritmética.
// =============================================================================

#include <cstdint>

namespace tdma {

struct SlotPlan {
  uint32_t airUs;       // Tiempo en el aire del beacon con el perfil activo
  uint32_t slotUs;      // Aire + guarda, redondeado al milisegundo
  uint8_t slotCount;    // Ranuras que entran en la trama
  uint8_t mySlot;       // Ranura de este nodo
  uint32_t myOffsetUs;  // Inicio de mi ranura respecto del inicio de la trama
};

SlotPlan makePlan(uint32_t airUs, uint16_t nodeId);

// Próximo inicio de mi ranura, en TOW (µs), que sea posterior a towNowUs más un
// margen. El margen deja tiempo para despertar y preparar el paquete.
int64_t nextSlotTow(const SlotPlan& plan, int64_t towNowUs, int64_t marginUs);

}  // namespace tdma
