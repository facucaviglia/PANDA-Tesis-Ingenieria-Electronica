// =============================================================================
// PANDA · Paso a Nivel Dinámico y Activo
// Firmware de nodo para LILYGO T-Beam Supreme (ESP32-S3, SX1262, MAX-M10S)
//
// Roles (se eligen al compilar, ver platformio.ini):
//   REGISTRADOR  GNSS 10 Hz + IMU a microSD y posición en vivo a Traccar.
//   TREN         Lo mismo más un beacon LoRa cifrado por trama de 100 ms, en su
//                ranura TDMA alineada al tiempo GPS.
//   CRUCE        Recibe y valida beacons (CRC, CMAC, contador, frescura), decide
//                el estado del cruce (NO SEGURO / APAGADO / FALLA) con el ETA de
//                cada tren y el circuito de vía, maneja las salidas físicas y
//                sube a Traccar el tren tal como llegó por radio.
//
// Arquitectura de tareas (FreeRTOS):
//   Core 1 (lazo de seguridad)   link (prio 22), decision (21), gnss (20), imu (18)
//   Core 0 (auxiliar)            sdlog (8), telemetry (5), console (4), ui (3), Wi-Fi
// Ninguna tarea encuesta millis(): cada una se bloquea en su período o en su
// cola, así el planificador reparte el CPU de forma determinista. El loop() de
// Arduino no se usa y se elimina.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>

#include "beacon.h"
#include "board_pins.h"
#include "button.h"
#include "config.h"
#include "console.h"
#include "crossing.h"
#include "crypto.h"
#include "display_manager.h"
#include "gnss_manager.h"
#include "imu_manager.h"
#include "persist.h"
#include "pmu_manager.h"
#include "radio_link.h"
#include "radio_manager.h"
#include "sd_logger.h"
#include "system_state.h"
#include "telemetry.h"

// -----------------------------------------------------------------------------
// Falla fatal
//
// La PMU hace titilar su LED a 4 Hz por hardware, así la indicación sigue
// aunque el ESP32 quede colgado. Si la que falló es la propia PMU no hay LED,
// y queda la pantalla y la consola.
// En el cruce, "nodo no operativo" equivale a SIN DATOS, que se trata como NO
// SEGURO. En la fase 3 además se va a forzar la salida física a ese estado.
// -----------------------------------------------------------------------------
[[noreturn]] static void fatalError(const char* subsystem, const char* detail) {
  g_pmu.setLed(LedPattern::BlinkFast);
  g_display.showFatal(subsystem, detail);
  for (;;) {
    Serial.printf("[FATAL] %s: %s. Nodo NO operativo.\n", subsystem, detail);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Todos los dispositivos SPI con su CS en alto antes de tocar cualquier bus.
// Si el CS de la IMU quedara flotando mientras se inicializa la microSD, la IMU
// interpretaría la secuencia de arranque de la tarjeta como comandos propios.
static void deselectAllSpiDevices() {
  for (int cs : {pins::kSdCs, pins::kImuCs, pins::kRadioCs}) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
  }
}

void setup() {
#if defined(PANDA_ROLE_CRUCE)
  // --- Salidas del cruce en estado seguro, antes que cualquier otra cosa ------
  crossing::beginIo();
#endif

  // --- Consola USB -----------------------------------------------------------
  // Buffer de recepción amplio: el simulador de la PC manda hasta 20 líneas por
  // segundo.
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  // Sin PC conectada el USB no debe frenar a nadie: timeout de TX en cero.
  Serial.setTxTimeoutMs(0);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" PANDA nodo " PANDA_ROLE_NAME "  fw " PANDA_FW_VERSION);
  Serial.println(" T-Beam Supreme / SX1262 / MAX-M10S");
  Serial.println(" Escribí h y Enter para ver los comandos");
  Serial.println("==============================================");

  deselectAllSpiDevices();
  persist::begin();

  if (!systemStateInit()) {
    fatalError("MEMORIA", "No se crearon las colas");
  }
  Serial.printf("[BOOT] PSRAM %s, %lu KB libres\n", psramFound() ? "OK" : "NO",
                static_cast<unsigned long>(ESP.getFreePsram() / 1024));

  // --- 1. PMU: sin ella no hay tensión en GNSS, radio, IMU ni microSD --------
  if (!g_pmu.begin()) {
    g_display.begin();
    fatalError("PMU", "AXP2101 no responde");
  }
  g_stats.pmuOnline.store(true);

  // --- 2. Pantalla -----------------------------------------------------------
  g_display.begin();
  g_display.bootStep("PMU ok");

#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
  // --- Criptografía: si los vectores oficiales no dan, el CMAC no es confiable
  if (!crypto::selfTest()) {
    fatalError("CRIPTO", "Fallan vectores AES");
  }
  if (!beaconSelfTest()) {
    fatalError("CRIPTO", "Beacon distinto a la PC");
  }
  g_display.bootStep("AES/CMAC ok");
#endif

  // --- 3. GNSS ---------------------------------------------------------------
  // No es fatal: la tarea reintenta cada 10 s. En el tren, sin GNSS el beacon
  // sale marcado sin fix y el cruce lo trata como no válido.
  g_display.bootStep(g_gnss.begin() ? "GNSS ok 10Hz" : "GNSS FALLA (reintenta)");

  // --- 4. Bus SPI compartido e IMU -------------------------------------------
  SPI.begin(pins::kSpiSck, pins::kSpiMiso, pins::kSpiMosi);
#if !defined(PANDA_ROLE_CRUCE)
  // El cruce no se mueve: no se lee la IMU para no llenar la microSD de nada.
  g_display.bootStep(g_imu.begin() ? "IMU ok" : "IMU FALLA");
#endif

  // --- 5. Radio --------------------------------------------------------------
#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
  // Sin radio el tren y el cruce no tienen razón de ser: falla fatal.
  if (!radiolink::begin()) {
    fatalError("RADIO", "SX1262 no responde");
  }
  g_display.bootStep(radiolink::activeProfile().name);
#else
  const int16_t radioStatus = g_radio.begin(cfg::radio::kProfiles[cfg::radio::kDefaultProfile],
                                            cfg::radio::kTxPowerBenchDbm);
  if (radioStatus != 0) {
    // El registrador no usa la radio. Se avisa pero se sigue registrando.
    g_display.bootStep("RADIO FALLA (aviso)");
    g_pmu.setLed(LedPattern::BlinkSlow);
  } else {
    g_display.bootStep("RADIO ok (dormida)");
    g_radio.sleep();
  }
#endif

  // --- 6. Tareas ---------------------------------------------------------------
  button::begin();
  logNote(NoteCode::Boot);

  g_logger.startTask();
  g_telemetry.startTask();
  g_gnss.startTask();
#if !defined(PANDA_ROLE_CRUCE)
  g_imu.startTask();
#endif
#if defined(PANDA_ROLE_TREN) || defined(PANDA_ROLE_CRUCE)
  radiolink::startTask();
#endif
#if defined(PANDA_ROLE_CRUCE)
  crossing::startTask();
#endif
  console::startTask();

  g_display.bootStep("Listo");
  g_display.startTask();
  Serial.println("[BOOT] Sistema en marcha");
}

void loop() {
  // Todo el trabajo lo hacen las tareas. El loopTask de Arduino se elimina para
  // no ocupar el core 1 con un bucle vacío.
  vTaskDelete(nullptr);
}
