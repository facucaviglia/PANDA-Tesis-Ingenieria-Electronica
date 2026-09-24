#pragma once
// =============================================================================
// Primitivas criptográficas del beacon, sobre el AES-128 por hardware del
// ESP32-S3 (vía mbedTLS en modo ECB, bloque a bloque).
//
//  - AES-128-CTR para cifrar el contenido del beacon.
//  - AES-CMAC (RFC 4493) para autenticarlo. El mbedTLS del core Arduino no
//    trae CMAC compilado, así que se implementa acá sobre el mismo AES.
//
// Se validan con los vectores oficiales en cada arranque (selfTest). Si fallan,
// el nodo no arranca: un CMAC mal implementado aceptaría beacons falsos.
// =============================================================================

#include <cstddef>
#include <cstdint>

namespace crypto {

constexpr size_t kBlock = 16;

// Contexto AES con la clave ya expandida, para no repetirlo en cada beacon.
class Aes128 {
 public:
  Aes128();
  ~Aes128();
  Aes128(const Aes128&) = delete;
  Aes128& operator=(const Aes128&) = delete;

  void setKey(const uint8_t key[kBlock]);
  void encryptBlock(const uint8_t in[kBlock], uint8_t out[kBlock]);

 private:
  void* ctx_;  // mbedtls_aes_context, oculto para no exponer mbedTLS en el header
};

// Cifra o descifra (es la misma operación) con AES-CTR. El bloque contador
// inicial es counterBlock y se incrementa como entero de 128 bits big-endian.
void ctrXor(Aes128& aes, const uint8_t counterBlock[kBlock], uint8_t* data, size_t len);

// AES-CMAC según RFC 4493. Devuelve la etiqueta completa de 16 bytes.
void cmac(Aes128& aes, const uint8_t* msg, size_t len, uint8_t tag[kBlock]);

// Comparación en tiempo constante, para no filtrar por tiempo cuántos bytes de
// la etiqueta coincidieron.
bool equalConstTime(const uint8_t* a, const uint8_t* b, size_t len);

// Vectores de RFC 4493 (CMAC) y NIST SP 800-38A F.5.1 (CTR).
bool selfTest();

}  // namespace crypto
