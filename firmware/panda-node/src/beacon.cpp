#include "beacon.h"

#include <algorithm>
#include <climits>
#include <cstring>

#include "config.h"

static_assert(offsetof(BeaconWire, flags) == kBeaconCipherOffset, "offset de cifrado");
static_assert(offsetof(BeaconWire, tag) == kBeaconTagOffset, "offset de etiqueta");
static_assert(kBeaconCipherOffset + kBeaconCipherLen == kBeaconTagOffset, "largo cifrado");
static_assert(sizeof(BeaconWire) == cfg::radio::kBeaconLength, "largo configurado en la radio");

static constexpr int8_t kAccelWireUnknown = INT8_MIN;

// Mismas conversiones que hacc_to_wire() y accel_to_wire() de
// tools/verify_crypto.py, en aritmética entera para que coincidan bit a bit.
static uint8_t hAccToWire(uint16_t cm) {
  return static_cast<uint8_t>(std::min<uint32_t>(255u, (static_cast<uint32_t>(cm) + 9u) / 10u));
}

static int8_t accelToWire(int16_t cms2) {
  if (cms2 == kAccelUnknown) return kAccelWireUnknown;
  const int32_t a = cms2;
  const int32_t w = a >= 0 ? (a + 1) / 2 : -((-a + 1) / 2);
  return static_cast<int8_t>(std::max<int32_t>(-127, std::min<int32_t>(127, w)));
}

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
  w.hAccDm = hAccToWire(in.hAccCm);
  w.accel2Cms2 = accelToWire(in.accelCms2);
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
  out.hAccCm = static_cast<uint16_t>(w.hAccDm * 10u);
  out.accelCms2 = w.accel2Cms2 == kAccelWireUnknown ? kAccelUnknown : static_cast<int16_t>(w.accel2Cms2 * 2);
  out.numSv = w.numSv;
  return BeaconOpen::Ok;
}

bool beaconSelfTest() {
  static constexpr uint8_t kExpected[35] = {
      0x21, 0xa2, 0x3f, 0x40, 0xe2, 0x01, 0x00, 0x2e, 0x23, 0xfa, 0x88, 0xb0, 0xaa, 0xef, 0x58, 0xcf, 0x2e, 0x7a,
      0xed, 0x10, 0x65, 0xf4, 0x16, 0x53, 0x4e, 0xc8, 0xaf, 0x50, 0x9e, 0x0f, 0xfb, 0xde, 0x39, 0x31, 0x7b};
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
  d.accelCms2 = -36;

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
  const bool roundTrip = back.counter == d.counter && back.latE7 == d.latE7 && back.hAccCm == 180 &&
                         back.accelCms2 == -36;
  return codec.open(pkt, sizeof(pkt), back) == BeaconOpen::BadTag && roundTrip;
}
