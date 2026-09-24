#include "crypto.h"

#include <cstring>
#include <mbedtls/aes.h>

namespace crypto {

Aes128::Aes128() {
  auto* c = new mbedtls_aes_context;
  mbedtls_aes_init(c);
  ctx_ = c;
}

Aes128::~Aes128() {
  auto* c = static_cast<mbedtls_aes_context*>(ctx_);
  mbedtls_aes_free(c);
  delete c;
}

void Aes128::setKey(const uint8_t key[kBlock]) {
  mbedtls_aes_setkey_enc(static_cast<mbedtls_aes_context*>(ctx_), key, 128);
}

void Aes128::encryptBlock(const uint8_t in[kBlock], uint8_t out[kBlock]) {
  mbedtls_aes_crypt_ecb(static_cast<mbedtls_aes_context*>(ctx_), MBEDTLS_AES_ENCRYPT, in, out);
}

// ---------------------------------------------------------------------------
// CTR
// ---------------------------------------------------------------------------
static void incrementBlock(uint8_t block[kBlock]) {
  for (int i = kBlock - 1; i >= 0; --i) {
    if (++block[i] != 0) {
      break;
    }
  }
}

void ctrXor(Aes128& aes, const uint8_t counterBlock[kBlock], uint8_t* data, size_t len) {
  uint8_t ctr[kBlock];
  uint8_t stream[kBlock];
  memcpy(ctr, counterBlock, kBlock);
  for (size_t off = 0; off < len; off += kBlock) {
    aes.encryptBlock(ctr, stream);
    const size_t n = (len - off < kBlock) ? (len - off) : kBlock;
    for (size_t i = 0; i < n; ++i) {
      data[off + i] ^= stream[i];
    }
    incrementBlock(ctr);
  }
}

// ---------------------------------------------------------------------------
// CMAC (RFC 4493)
//
// 1. L = AES(K, 0). Las subclaves K1 y K2 salen de desplazar L un bit a la
//    izquierda en GF(2^128), aplicando la constante Rb = 0x87 si se cae un 1.
// 2. Se procesa el mensaje como CBC-MAC. El último bloque se combina con K1 si
//    está completo o se rellena con 10...0 y se combina con K2 si no.
// ---------------------------------------------------------------------------
static void leftShiftOne(const uint8_t in[kBlock], uint8_t out[kBlock]) {
  uint8_t carry = 0;
  for (int i = kBlock - 1; i >= 0; --i) {
    out[i] = static_cast<uint8_t>((in[i] << 1) | carry);
    carry = (in[i] & 0x80) ? 1 : 0;
  }
}

static void deriveSubkeys(Aes128& aes, uint8_t k1[kBlock], uint8_t k2[kBlock]) {
  static constexpr uint8_t kRb = 0x87;
  uint8_t zero[kBlock] = {0};
  uint8_t l[kBlock];
  aes.encryptBlock(zero, l);

  leftShiftOne(l, k1);
  if (l[0] & 0x80) {
    k1[kBlock - 1] ^= kRb;
  }
  leftShiftOne(k1, k2);
  if (k1[0] & 0x80) {
    k2[kBlock - 1] ^= kRb;
  }
}

void cmac(Aes128& aes, const uint8_t* msg, size_t len, uint8_t tag[kBlock]) {
  uint8_t k1[kBlock];
  uint8_t k2[kBlock];
  deriveSubkeys(aes, k1, k2);

  size_t nBlocks = (len + kBlock - 1) / kBlock;
  bool lastComplete;
  if (nBlocks == 0) {
    nBlocks = 1;
    lastComplete = false;
  } else {
    lastComplete = (len % kBlock) == 0;
  }

  uint8_t last[kBlock];
  const size_t lastOff = (nBlocks - 1) * kBlock;
  if (lastComplete) {
    for (size_t i = 0; i < kBlock; ++i) {
      last[i] = msg[lastOff + i] ^ k1[i];
    }
  } else {
    const size_t rem = len - lastOff;
    for (size_t i = 0; i < kBlock; ++i) {
      uint8_t b;
      if (i < rem) {
        b = msg[lastOff + i];
      } else if (i == rem) {
        b = 0x80;
      } else {
        b = 0x00;
      }
      last[i] = b ^ k2[i];
    }
  }

  uint8_t x[kBlock] = {0};
  uint8_t y[kBlock];
  for (size_t b = 0; b + 1 < nBlocks; ++b) {
    for (size_t i = 0; i < kBlock; ++i) {
      y[i] = x[i] ^ msg[b * kBlock + i];
    }
    aes.encryptBlock(y, x);
  }
  for (size_t i = 0; i < kBlock; ++i) {
    y[i] = x[i] ^ last[i];
  }
  aes.encryptBlock(y, tag);
}

bool equalConstTime(const uint8_t* a, const uint8_t* b, size_t len) {
  uint8_t diff = 0;
  for (size_t i = 0; i < len; ++i) {
    diff |= a[i] ^ b[i];
  }
  return diff == 0;
}

// ---------------------------------------------------------------------------
// Vectores de prueba oficiales
// ---------------------------------------------------------------------------
bool selfTest() {
  static constexpr uint8_t kKey[kBlock] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                                           0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
  static constexpr uint8_t kMsg[kBlock] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
                                           0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a};
  // RFC 4493, ejemplo 1: mensaje vacío.
  static constexpr uint8_t kTagEmpty[kBlock] = {0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28,
                                                0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75, 0x67, 0x46};
  // RFC 4493, ejemplo 2: mensaje de 16 bytes.
  static constexpr uint8_t kTag16[kBlock] = {0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
                                             0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c};
  // NIST SP 800-38A F.5.1, primer bloque de CTR-AES128.
  static constexpr uint8_t kCtr0[kBlock] = {0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
                                            0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff};
  static constexpr uint8_t kCtrOut[kBlock] = {0x87, 0x4d, 0x61, 0x91, 0xb6, 0x20, 0xe3, 0x26,
                                              0x1b, 0xef, 0x68, 0x64, 0x99, 0x0d, 0xb6, 0xce};

  Aes128 aes;
  aes.setKey(kKey);
  uint8_t tag[kBlock];

  cmac(aes, nullptr, 0, tag);
  if (!equalConstTime(tag, kTagEmpty, kBlock)) return false;

  cmac(aes, kMsg, sizeof(kMsg), tag);
  if (!equalConstTime(tag, kTag16, kBlock)) return false;

  uint8_t buf[kBlock];
  memcpy(buf, kMsg, kBlock);
  ctrXor(aes, kCtr0, buf, kBlock);
  if (!equalConstTime(buf, kCtrOut, kBlock)) return false;

  // Ida y vuelta: descifrar tiene que devolver el original.
  ctrXor(aes, kCtr0, buf, kBlock);
  return equalConstTime(buf, kMsg, kBlock);
}

}  // namespace crypto
