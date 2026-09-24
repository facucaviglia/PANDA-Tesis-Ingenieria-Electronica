#pragma once
// =============================================================================
// Geodesia mínima para PANDA.
//
// Haversine sobre la esfera de radio medio terrestre. A las distancias de
// PANDA (unos km) el error frente al elipsoide WGS84 es menor al 0,5 %, muy
// por debajo del error del GNSS. Se calcula en double: en float el error de
// redondeo sería de metros.
// =============================================================================

#include <cmath>
#include <cstdint>

namespace geo {

constexpr double kEarthRadiusM = 6371008.8;
constexpr double kDegToRad = M_PI / 180.0;

inline double haversineM(int32_t lat1E7, int32_t lon1E7, int32_t lat2E7, int32_t lon2E7) {
  const double lat1 = lat1E7 * 1e-7 * kDegToRad;
  const double lat2 = lat2E7 * 1e-7 * kDegToRad;
  const double dLat = lat2 - lat1;
  const double dLon = (lon2E7 - lon1E7) * 1e-7 * kDegToRad;
  const double s1 = std::sin(dLat / 2.0);
  const double s2 = std::sin(dLon / 2.0);
  const double a = s1 * s1 + std::cos(lat1) * std::cos(lat2) * s2 * s2;
  return 2.0 * kEarthRadiusM * std::asin(std::sqrt(std::fmin(1.0, a)));
}

}  // namespace geo
