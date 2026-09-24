#pragma once
// =============================================================================
// Datos que sobreviven a un reinicio, guardados en la NVS de la flash.
//
//  - Perfil de radio elegido y potencia (banco o campo).
//  - Contador del beacon: se reserva en bloques para que nunca se repita,
//    aunque el nodo se reinicie. Si se repitiera, se repetiría el nonce del
//    cifrado AES-CTR y el receptor lo tomaría como repetición.
//
// Escribir la flash congela la caché unos milisegundos en los dos núcleos. Por
// eso se escribe solo al arrancar y una vez cada 65536 beacons, siempre
// después de una transmisión y nunca antes.
// =============================================================================

#include <cstdint>

namespace persist {

void begin();

uint8_t loadProfile();
void saveProfile(uint8_t index);

bool loadFieldPower();
void saveFieldPower(bool field);

// Devuelve el primer valor del bloque reservado y deja guardado el siguiente.
uint32_t reserveCounterBlock();

// Posición de referencia del cruce medida en el lugar (grados * 1e7).
bool loadCrossingRef(int32_t& latE7, int32_t& lonE7);
void saveCrossingRef(int32_t latE7, int32_t lonE7);
void clearCrossingRef();

// Identificador del nodo: PANDA_NODE_ID si está definido, si no sale de la MAC.
uint16_t nodeId();

}  // namespace persist
