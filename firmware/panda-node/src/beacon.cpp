#include "beacon.h"

#include <cstring>

#include "config.h"

static_assert(offsetof(BeaconWire, flags) == kBeaconCipherOffset, "offset de cifrado");
static_assert(offsetof(BeaconWire, tag) == kBeaconTagOffset, "offset de etiqueta");
static_assert(kBeaconCipherOffset + kBeaconCipherLen == kBeaconTagOffset, "largo cifrado");
static_assert(sizeof(BeaconWire) == cfg::radio::kBeaconLength, "largo configurado en la radio");

static constexpr uint8_t kVerType = static_cast<uint8_t>((cfg::beacon::kVersion << 4) | cfg::beacon::kNodeTypeTren);

BeaconCodec::BeaconCodec(const uint8_t encKey[16], const uint8_t macKey[16]) {
  enc_.setKey(encKey);
  mac_.setKey(macKey);
}

void BeaconCodec::makeNonce(const BeaconWire& w, uint8_t nonce[crypto::kBlock]) {
  // Los primeros 7 bytes identifican al paquete. Los 9 restantes arrancan en
  // cero y hacen de contador de bloque de CTR (el beacon usa 2 bloques).
  memset(nonce, 0, crypto::kBlock);
  memcpy(nonce, &w, kBeaconCipherOffset);
}

void BeaconCodec::seal(const BeaconData& in, uint8_t* out) {
  BeaconWire w{};
  w.verType = kVerType;
  w.nodeId = in.nodeId;
  w.counter = in.counter;
  w.flags = in.flags;
  w.itowMs = in.itowMs;
  w.latE7 = in.latE7;
  w.lonE7 = in.lonE7;
  w.speedCms = in.speedCms;
  w.headingCdeg = in.headingCdeg;
  w.hAccCm = in.hAccCm;
  w.numSv = in.numSv;

  uint8_t* raw = reinterpret_cast<uint8_t*>(&w);
  uint8_t nonce[crypto::kBlock];
  makeNonce(w, nonce);
  crypto::ctrXor(enc_, nonce, raw + kBeaconCipherOffset, kBeaconCipherLen);

  uint8_t tag[crypto::kBlock];
  crypto::cmac(mac_, raw, kBeaconTagOffset, tag);
  memcpy(w.tag, tag, kBeaconTagLen);

  memcpy(out, &w, sizeof(w));
}

BeaconOpen BeaconCodec::open(const uint8_t* in, size_t len, BeaconData& out) {
  if (len != sizeof(BeaconWire)) {
    return BeaconOpen::BadLength;
  }
  BeaconWire w;
  memcpy(&w, in, sizeof(w));
  if (w.verType != kVerType) {
    return BeaconOpen::BadVersion;
  }

  // Primero la autenticación, sobre lo que llegó tal cual.
  uint8_t tag[crypto::kBlock];
  crypto::cmac(mac_, in, kBeaconTagOffset, tag);
  if (!crypto::equalConstTime(tag, w.tag, kBeaconTagLen)) {
    return BeaconOpen::BadTag;
  }

  uint8_t* raw = reinterpret_cast<uint8_t*>(&w);
  uint8_t nonce[crypto::kBlock];
  makeNonce(w, nonce);
  crypto::ctrXor(enc_, nonce, raw + kBeaconCipherOffset, kBeaconCipherLen);

  out.nodeId = w.nodeId;
  out.counter = w.counter;
  out.flags = w.flags;
  out.itowMs = w.itowMs;
  out.latE7 = w.latE7;
  out.lonE7 = w.lonE7;
  out.speedCms = w.speedCms;
  out.headingCdeg = w.headingCdeg;
  out.hAccCm = w.hAccCm;
  out.numSv = w.numSv;
  return BeaconOpen::Ok;
}

bool beaconSelfTest() {
  static constexpr uint8_t kExpected[35] = {
      0x11, 0xa2, 0x3f, 0x40, 0xe2, 0x01, 0x00, 0xc2, 0x49, 0x7d, 0x78, 0x5b, 0x4d, 0x9c, 0x6f, 0x75, 0xb9, 0xca,
      0x14, 0xc1, 0x45, 0x93, 0x9a, 0xda, 0x4b, 0x07, 0xe7, 0x69, 0x01, 0x30, 0x03, 0xbe, 0x9e, 0x3f, 0x3d};
  BeaconData d{};
  d.nodeId = 0x3FA2;
  d.counter = 123456;
  d.flags = 0x07;
  d.itowMs = 345600100;
  d.latE7 = -346032145;
  d.lonE7 = -585012345;
  d.speedCms = 1523;
  d.headingCdeg = 27350;
  d.hAccCm = 180;
  d.numSv = 14;

  BeaconCodec codec(cfg::beacon::kEncKey, cfg::beacon::kMacKey);
  uint8_t pkt[35];
  codec.seal(d, pkt);
  if (memcmp(pkt, kExpected, sizeof(pkt)) != 0) {
    return false;
  }
  BeaconData back{};
  if (codec.open(pkt, sizeof(pkt), back) != BeaconOpen::Ok) {
    return false;
  }
  // Un bit alterado tiene que ser rechazado.
  pkt[12] ^= 0x01;
  return codec.open(pkt, sizeof(pkt), back) == BeaconOpen::BadTag && back.counter == d.counter &&
         back.latE7 == d.latE7;
}
