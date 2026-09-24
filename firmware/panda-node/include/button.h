#pragma once
// =============================================================================
// Botón BOOT (GPIO 0).
//
//  - Pulsación corta: MARCA numerada en events.csv, con el tiempo exacto del
//    momento en que se apretó (tomado en la interrupción). Sirve para anotar
//    "estación X" o el instante real en que el tren pasa el cruce.
//  - Pulsación larga (1,5 s): pasa al siguiente perfil de radio.
// =============================================================================

#include <cstdint>

namespace button {

enum class Event : uint8_t { None, Mark, LongPress };

void begin();

// Devuelve el próximo evento pendiente. Para Mark entrega el número de marca y
// el instante en que se apretó el botón.
Event poll(uint32_t& markNumber, int64_t& markUs);

// Genera una marca desde software (comando "m" de la consola).
void injectMark();

}  // namespace button
