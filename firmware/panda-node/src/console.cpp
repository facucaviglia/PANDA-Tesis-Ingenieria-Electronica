#include "console.h"

#include <Arduino.h>
#include <cstdlib>
#include <cstring>

#include "button.h"
#include "config.h"
#include "crossing.h"
#include "gnss_manager.h"
#include "radio_link.h"
#include "radio_manager.h"
#include "system_state.h"

namespace console {

static void printHelp() {
  Serial.println();
  Serial.println("Comandos (una letra y Enter):");
  Serial.println("  h  esta ayuda");
  Serial.println("  i  información del nodo y del enlace");
  Serial.println("  m  marca (igual que el botón)");
#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
  Serial.println("  p  siguiente perfil de radio (cambiarlo en los DOS nodos)");
  Serial.println("  s  barrido de canales (piso de ruido)");
#endif
#if defined(PANDA_ROLE_TREN)
  Serial.println("  w  alternar potencia BANCO / CAMPO");
#endif
#if defined(PANDA_ROLE_CRUCE)
  Serial.println("  c  guardar la posición del cruce (promedio del GNSS propio)");
  Serial.println("  C  borrar la posición guardada del cruce");
  Serial.println("  v  simular el circuito de vía (alterna LIBRE / OCUPADA)");
  Serial.println("  x  silenciar o activar el aviso sonoro");
#endif
  Serial.println();
}

static void printInfo() {
  Serial.println();
  Serial.printf("PANDA %s fw %s\n", PANDA_ROLE_NAME, PANDA_FW_VERSION);
#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
  const auto& p = radiolink::activeProfile();
  Serial.printf("Radio: %.1f MHz, %s, %d dBm, aire del beacon %lu us\n", static_cast<double>(cfg::radio::kFrequencyMHz),
                p.name, radiolink::activePowerDbm(),
                static_cast<unsigned long>(g_radio.timeOnAirUs(cfg::radio::kBeaconLength)));
#endif
#if defined(PANDA_ROLE_TREN)
  radiolink::TxStatus tx{};
  if (radiolink::latestTx(tx)) {
    Serial.printf("Nodo %04lX, ranura %u de %u, %lu us por ranura, próximo contador %lu\n",
                  static_cast<unsigned long>(g_stats.nodeId.load()), tx.slot, tx.slotCount,
                  static_cast<unsigned long>(tx.slotUs), static_cast<unsigned long>(tx.counter));
  }
#endif
#if defined(PANDA_ROLE_CRUCE)
  CrossingStatus cs{};
  if (crossing::latest(cs)) {
    Serial.printf("Cruce: %s (%s), referencia %s %.7f, %.7f (%lu fixes promediados)\n", crossing::stateName(cs.state),
                  crossing::reasonName(cs.reason), crossing::refSourceName(cs.refSource), cs.refLatE7 / 1e7,
                  cs.refLonE7 / 1e7, static_cast<unsigned long>(cs.refSamples));
    Serial.printf("Circuito de vía: %s (%s). Pasos %lu, vía sin PANDA %lu, liberados %lu, watchdog %lu\n",
                  cs.trackOccupied ? "OCUPADA" : "LIBRE", cs.trackEnabled ? "entrada física" : "simulado",
                  static_cast<unsigned long>(cs.passages), static_cast<unsigned long>(cs.trackWithoutPanda),
                  static_cast<unsigned long>(cs.silentReleases), static_cast<unsigned long>(cs.watchdogTrips));
    Serial.printf("Umbrales: ETA %.0f s, a_max %.1f m/s2, ocupación %.0f m, E-9 %lu ms\n",
                  static_cast<double>(cfg::crossing::kAlertEtaS), static_cast<double>(cfg::crossing::kMaxAccelMps2),
                  static_cast<double>(cfg::crossing::kOccupiedRadiusM),
                  static_cast<unsigned long>(cfg::beacon::kLinkTimeoutMs));
  }
#endif
  Serial.printf("GNSS a %lu baud, PSRAM libre %lu KB\n", static_cast<unsigned long>(g_stats.gnssBaud.load()),
                static_cast<unsigned long>(ESP.getFreePsram() / 1024));
  Serial.println();
}

// Comandos de una letra.
static void handleLetter(char c) {
  switch (c) {
    case 'h':
    case '?':
      printHelp();
      break;
    case 'i':
      printInfo();
      break;
    case 'm':
      button::injectMark();
      break;
#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
    case 'p':
      radiolink::requestNextProfile();
      break;
    case 's':
      radiolink::requestScan();
      break;
#endif
#if defined(PANDA_ROLE_TREN)
    case 'w':
      radiolink::requestTogglePower();
      break;
#endif
#if defined(PANDA_ROLE_CRUCE)
    case 'c':
      crossing::requestSaveRef();
      break;
    case 'C':
      crossing::requestClearRef();
      break;
    case 'v':
      crossing::toggleSimulatedTrack();
      break;
    case 'x':
      crossing::toggleMute();
      break;
#endif
    default:
      break;
  }
}

#if defined(PANDA_ROLE_CRUCE)
static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
#endif

// Comandos del simulador. Devuelve false si la línea no es un comando válido.
static bool handleSimLine(char* line) {
  char* save = nullptr;
  const char* cmd = strtok_r(line, " ", &save);
  if (cmd == nullptr || cmd[1] != '\0') {
    return false;
  }
  switch (cmd[0]) {
    case 'T': {
      const char* a = strtok_r(nullptr, " ", &save);
      if (a == nullptr) return false;
      setPcTime(static_cast<int64_t>(strtoll(a, nullptr, 10)) * 1000);
      return true;
    }
#if defined(PANDA_ROLE_CRUCE)
    case 'R': {
      const char* a = strtok_r(nullptr, " ", &save);
      const char* b = strtok_r(nullptr, " ", &save);
      if (a == nullptr || b == nullptr) return false;
      crossing::setPcRef(static_cast<int32_t>(strtol(a, nullptr, 10)), static_cast<int32_t>(strtol(b, nullptr, 10)));
      return true;
    }
    case 'B': {
      const char* hex = strtok_r(nullptr, " ", &save);
      if (hex == nullptr || strlen(hex) != 2 * cfg::radio::kBeaconLength) return false;
      uint8_t pkt[cfg::radio::kBeaconLength];
      for (size_t i = 0; i < sizeof(pkt); ++i) {
        const int hi = hexNibble(hex[2 * i]);
        const int lo = hexNibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        pkt[i] = static_cast<uint8_t>((hi << 4) | lo);
      }
      const char* r = strtok_r(nullptr, " ", &save);
      const char* s = strtok_r(nullptr, " ", &save);
      const float rssi = r ? strtof(r, nullptr) : -80.0f;
      const float snr = s ? strtof(s, nullptr) : 10.0f;
      radiolink::injectPacket(pkt, sizeof(pkt), rssi, snr);
      return true;
    }
#endif
    default:
      return false;
  }
}

static void consoleTask(void*) {
  char line[128];
  size_t len = 0;
  for (;;) {
    while (Serial.available() > 0) {
      const int c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line[len] = '\0';
        if (len == 1) {
          handleLetter(line[0]);
        } else if (len > 1 && cfg::sim::kAllowInjection) {
          handleSimLine(line);
        }
        len = 0;
        continue;
      }
      if (len < sizeof(line) - 1) {
        line[len++] = static_cast<char>(c);
      } else {
        len = 0;  // Línea demasiado larga: se descarta
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void startTask() {
  xTaskCreatePinnedToCore(consoleTask, "console", 4096, nullptr, 4, nullptr, cfg::task::kCoreAux);
}

}  // namespace console
