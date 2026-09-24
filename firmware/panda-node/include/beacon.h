#pragma once
// =============================================================================
// Beacon del nodo tren: 35 bytes, cifrado y autenticado.
//
// Formato en el aire (little-endian, igual al orden de memoria del ESP32):
//
//   off  largo  campo          en claro  descripción
//   0    1      verTipo        sí        versión (4 bits altos) y tipo de nodo
//   1    2      nodeId         sí        identificador del tren
//   3    4      counter        sí        contador anti-repetición, nunca se repite
//   7    1      flags          no        ver BeaconFlag
//   8    4      itowMs         no        tiempo GPS de la semana del fix
//   12   4      latE7          no        latitud en grados * 1e7
//   16   4      lonE7          no        longitud en grados * 1e7
//   20   2      speedCms       no        velocidad en cm/s
//   22   2      headingCdeg    no        rumbo en centésimas de grado
//   24   2      hAccCm         no        precisión horizontal en cm
//   26   1      numSv          no        satélites usados
//   27   8      tag            sí        CMAC truncado sobre los bytes 0 a 26
//
// Cabecera en claro: el receptor necesita el nodeId y el contador para armar
// el nonce y para descartar repeticiones antes de descifrar.
// Esquema cifrar-y-después-autenticar: el CMAC cubre la cabecera y el texto
// cifrado. El receptor verifica el CMAC primero y solo entonces descifra.
// El nonce de CTR es (verTipo, nodeId, counter), único mientras el contador no
// se repita, y eso lo garantiza persist::reserveCounterBlock().
// =============================================================================

#include <cstddef>
#include <cstdint>

#include "crypto.h"

namespace BeaconFlag {
constexpr uint8_t kFixOk = 1 << 0;       // Fix 2D/3D válido
constexpr uint8_t kTimeSync = 1 << 1;    // El tren tiene tiempo GPS (itow confiable)
constexpr uint8_t kTimePps = 1 << 2;     // ... y está disciplinado por el 1PPS
constexpr uint8_t kStill = 1 << 3;       // La IMU ve al tren quieto
constexpr uint8_t kImuOk = 1 << 4;       // La IMU funciona
constexpr uint8_t kPlausChecked = 1 << 5;  // Fase 4: se verificó GNSS contra IMU
constexpr uint8_t kEstimated = 1 << 6;   // Fase 4: posición proyectada sin GNSS
}  // namespace BeaconFlag

// Contenido del beacon en unidades de trabajo.
struct BeaconData {
  uint16_t nodeId;
  uint32_t counter;
  uint8_t flags;
  uint32_t itowMs;
  int32_t latE7;
  int32_t lonE7;
  uint16_t speedCms;
  uint16_t headingCdeg;
  uint16_t hAccCm;
  uint8_t numSv;
};

// Representación exacta en el aire.
struct __attribute__((packed)) BeaconWire {
  uint8_t verType;
  uint16_t nodeId;
  uint32_t counter;
  // Desde acá, cifrado
  uint8_t flags;
  uint32_t itowMs;
  int32_t latE7;
  int32_t lonE7;
  uint16_t speedCms;
  uint16_t headingCdeg;
  uint16_t hAccCm;
  uint8_t numSv;
  // Etiqueta
  uint8_t tag[8];
};
static_assert(sizeof(BeaconWire) == 35, "El beacon tiene que medir 35 bytes");

enum class BeaconOpen : uint8_t {
  Ok = 0,
  BadLength,
  BadVersion,
  BadTag,
};

class BeaconCodec {
 public:
  BeaconCodec(const uint8_t encKey[16], const uint8_t macKey[16]);

  // Arma, cifra y autentica. out tiene que tener 35 bytes.
  void seal(const BeaconData& in, uint8_t* out);

  // Verifica la etiqueta y, si es válida, descifra.
  BeaconOpen open(const uint8_t* in, size_t len, BeaconData& out);

 private:
  void makeNonce(const BeaconWire& w, uint8_t nonce[crypto::kBlock]);
  crypto::Aes128 enc_;
  crypto::Aes128 mac_;
};

constexpr size_t kBeaconCipherOffset = 7;
constexpr size_t kBeaconCipherLen = 20;
constexpr size_t kBeaconTagOffset = 27;
constexpr size_t kBeaconTagLen = 8;

// Arma un beacon de referencia y lo compara byte a byte con el que genera
// tools/verify_crypto.py en la PC. Garantiza que el firmware y el
// decodificador de referencia coinciden en claves, formato y algoritmos.
bool beaconSelfTest();
