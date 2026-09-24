#include "persist.h"

#include <Arduino.h>
#include <Preferences.h>

#include "config.h"

namespace persist {

static Preferences s_prefs;
static constexpr const char* kNamespace = "panda";

void begin() {
  s_prefs.begin(kNamespace, false);
}

uint8_t loadProfile() {
  const uint8_t p = s_prefs.getUChar("profile", cfg::radio::kDefaultProfile);
  return p < cfg::radio::kProfileCount ? p : cfg::radio::kDefaultProfile;
}

void saveProfile(uint8_t index) {
  s_prefs.putUChar("profile", index);
}

bool loadFieldPower() {
  return s_prefs.getBool("fieldpwr", false);
}

void saveFieldPower(bool field) {
  s_prefs.putBool("fieldpwr", field);
}

uint32_t reserveCounterBlock() {
  // Arranca en 1: el receptor usa 0 como "nunca visto".
  const uint32_t base = s_prefs.getUInt("ctr", 1);
  s_prefs.putUInt("ctr", base + cfg::beacon::kCounterBlock);
  return base;
}

bool loadCrossingRef(int32_t& latE7, int32_t& lonE7) {
  if (!s_prefs.isKey("ref_lat") || !s_prefs.isKey("ref_lon")) {
    return false;
  }
  latE7 = s_prefs.getInt("ref_lat", 0);
  lonE7 = s_prefs.getInt("ref_lon", 0);
  return latE7 != 0 || lonE7 != 0;
}

void saveCrossingRef(int32_t latE7, int32_t lonE7) {
  s_prefs.putInt("ref_lat", latE7);
  s_prefs.putInt("ref_lon", lonE7);
}

void clearCrossingRef() {
  s_prefs.remove("ref_lat");
  s_prefs.remove("ref_lon");
}

uint16_t nodeId() {
#if defined(PANDA_NODE_ID)
  return static_cast<uint16_t>(PANDA_NODE_ID);
#else
  // Los 48 bits de la MAC plegados a 16. No es único garantizado, pero entre
  // dos o tres placas la chance de choque es despreciable. Se evita el 0.
  const uint64_t mac = ESP.getEfuseMac();
  uint16_t id = static_cast<uint16_t>(mac ^ (mac >> 16) ^ (mac >> 32));
  return id == 0 ? 1 : id;
#endif
}

}  // namespace persist
