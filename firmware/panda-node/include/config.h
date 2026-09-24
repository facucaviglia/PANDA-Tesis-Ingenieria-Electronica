#pragma once
// =============================================================================
// PANDA · Parámetros globales del nodo
//
// Todo lo que se ajusta en banco o en campo vive acá. Los pines están en
// board_pins.h y las credenciales en secrets.h.
// =============================================================================

#include <cstddef>
#include <cstdint>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets_example.h"
#endif

// -----------------------------------------------------------------------------
// Rol del nodo. Lo define el entorno de platformio.ini, no se toca a mano.
// -----------------------------------------------------------------------------
#if defined(PANDA_ROLE_REGISTRADOR)
#define PANDA_ROLE_NAME "REGISTRADOR"
#define PANDA_ROLE_SHORT "REG"
#elif defined(PANDA_ROLE_TREN)
#define PANDA_ROLE_NAME "TREN"
#define PANDA_ROLE_SHORT "TREN"
#elif defined(PANDA_ROLE_CRUCE)
#define PANDA_ROLE_NAME "CRUCE"
#define PANDA_ROLE_SHORT "CRUCE"
#else
#error "Falta definir el rol: compilar con -e registrador, -e tren o -e cruce"
#endif

#define PANDA_FW_VERSION "0.3.0-fase3"

namespace cfg {

// -----------------------------------------------------------------------------
// GNSS u-blox MAX-M10S
// -----------------------------------------------------------------------------
namespace gnss {
// 10 Hz: una solución cada 100 ms. Es lo que fija la antigüedad del dato en el
// cruce, porque el beacon sale una vez por época.
constexpr uint8_t kNavRateHz = 10;

// A 460800 baud un NAV-PVT (100 bytes) tarda 2,2 ms en llegar. A 9600 baud
// tardaría 104 ms y no entraría en la época.
constexpr uint32_t kTargetBaud = 460800;

// Velocidades que se prueban al arrancar. Primero la final, porque si solo se
// reinició el ESP32 el módulo sigue configurado. Después la de fábrica.
constexpr uint32_t kProbeBauds[] = {kTargetBaud, 9600, 38400, 115200};

// Buffer de recepción de la UART. A 460800 baud llegan 46 bytes por ms, así que
// 4 KB cubren casi 90 ms sin leer.
constexpr size_t kUartRxBuffer = 4096;

// Sin NAV-PVT durante este tiempo se considera que el GNSS se cayó y se
// intenta reconfigurar.
constexpr uint32_t kTimeoutMs = 3000;
constexpr uint32_t kReinitPeriodMs = 10000;

// Reloj GPS local. El flanco del 1PPS marca el segundo GPS exacto con error de
// microsegundos. Entre flancos se interpola con el reloj del ESP32, que deriva
// menos de 20 ppm (20 µs por segundo). Pasado este tiempo sin PPS se deja de
// confiar en la referencia.
constexpr uint32_t kPpsHoldoverMs = 10000;

// Si no hay PPS se usa la hora del último NAV-PVT, que llega unos 30 a 50 ms
// después de su época. Es una referencia gruesa y se marca como tal.
constexpr uint32_t kCoarseMaxAgeMs = 2000;
}  // namespace gnss

// -----------------------------------------------------------------------------
// IMU QMI8658
// -----------------------------------------------------------------------------
namespace imu {
// Con acelerómetro y giróscopo activos, la tasa la fija el giróscopo: 224,2 Hz.
constexpr float kAccelOdrHz = 250.0f;
constexpr float kGyroOdrHz  = 224.2f;

// Se lee la FIFO cada 20 ms (unas 4 o 5 muestras). La FIFO guarda 128, o sea
// 570 ms, suficiente para aguantar una escritura lenta de la microSD en el bus
// SPI compartido sin perder muestras.
constexpr uint32_t kReadPeriodMs = 20;
constexpr uint16_t kFifoCapacity = 128;

// Descarte inicial de lecturas de FIFO mientras se estabiliza el sensor.
constexpr uint8_t kWarmupReads = 8;

// Detector de "quieto" (versión inicial, se ajusta en la fase 4 con datos del
// viaje 1). Ventana de 1 s: desvío de |a| y media de |w| por debajo del umbral.
constexpr float kStillAccStdMps2 = 0.08f;
constexpr float kStillGyroMeanDps = 1.5f;
}  // namespace imu

// -----------------------------------------------------------------------------
// Radio LoRa SX1262
// -----------------------------------------------------------------------------
namespace radio {
struct LoraProfile {
  const char* name;
  float bwKHz;
  uint8_t sf;
  uint8_t cr;  // Denominador de la tasa de código: 5 = 4/5, 8 = 4/8
};

// Perfiles a comparar en campo. El 0 es el de referencia. Se cambia con una
// pulsación larga del botón o con el comando "p" por la consola, y queda
// guardado en la flash. Los dos nodos tienen que usar el mismo.
//   SF7/500 CR4/5: 18 ms de aire, ~-117 dBm, 5 ranuras por trama de 100 ms
//   SF6/500 CR4/5: 10 ms, ~-114 dBm, 8 ranuras
//   SF7/250 CR4/5: 36 ms, ~-121 dBm, 2 ranuras
//   SF7/500 CR4/8: 26 ms, corrige errores por desvanecimiento rápido, 3 ranuras
constexpr LoraProfile kProfiles[] = {
    {"SF7/500 CR4/5", 500.0f, 7, 5},
    {"SF6/500 CR4/5", 500.0f, 6, 5},
    {"SF7/250 CR4/5", 250.0f, 7, 5},
    {"SF7/500 CR4/8", 500.0f, 7, 8},
};
constexpr uint8_t kProfileCount = sizeof(kProfiles) / sizeof(kProfiles[0]);
constexpr uint8_t kDefaultProfile = 0;

// Canal único de 500 kHz dentro de 915 a 928 MHz, lejos de la parte baja.
// Para elegir otro, correr el barrido ("s" por consola) en el lugar del ensayo.
constexpr float kFrequencyMHz = 925.0f;

// Canales candidatos del barrido: centros cada 500 kHz de 915,5 a 927,5 MHz.
constexpr float kScanFirstMHz = 915.5f;
constexpr float kScanStepMHz = 0.5f;
constexpr uint8_t kScanChannels = 25;
constexpr uint32_t kScanDwellMs = 120;

// Sync word privada: el chip descarta sin despertar al MCU los paquetes de
// LoRaWAN (0x34) y de otras redes. No da seguridad, eso lo hace el CMAC.
constexpr uint8_t kSyncWord = 0x12;
constexpr uint16_t kPreambleSymbols = 8;

// Potencia. En banco se usa poca para no saturar al receptor que está al lado.
// El máximo del SX1262 es 22 dBm y requiere 140 mA de límite de corriente.
// Se alterna entre las dos con el comando "w" por consola y queda guardada.
constexpr int8_t kTxPowerBenchDbm = 2;
constexpr int8_t kTxPowerFieldDbm = 22;
constexpr float kCurrentLimitMa = 140.0f;

// Largo fijo del beacon. Con header implícito el receptor ya lo conoce.
constexpr uint8_t kBeaconLength = 35;
}  // namespace radio

// -----------------------------------------------------------------------------
// TDMA: trama de 100 ms alineada al tiempo GPS
//
// Cada época GNSS (cada 100 ms) abre una trama. La ranura k empieza en
//   inicio_trama + kFirstSlotOffsetUs + k * largo_ranura   (módulo 100 ms)
// El desplazamiento inicial deja llegar el NAV-PVT de esa época antes de la
// primera ranura, así el beacon lleva el fix más fresco posible. El largo de
// ranura es el tiempo en el aire del perfil más la guarda.
// -----------------------------------------------------------------------------
namespace tdma {
constexpr uint32_t kFrameUs = 100000;
constexpr uint32_t kFirstSlotOffsetUs = 50000;
// La guarda cubre el error del reloj GPS local (µs), la latencia del timer y
// del SPI (~0,3 ms) y el arranque del transmisor. 2 ms sobra.
constexpr uint32_t kGuardUs = 2000;
constexpr uint8_t kMaxSlots = 8;

// Sin tiempo GPS el tren no puede respetar su ranura. En banco, bajo techo, es
// normal no tener fix. Con esto en true el tren igual transmite a 10 Hz con su
// reloj local (sin ranura) y marca el beacon como "sin tiempo". El cruce nunca
// toma esos beacons como datos válidos, solo como medición de enlace.
constexpr bool kAllowUnsyncedTx = true;
}  // namespace tdma

// -----------------------------------------------------------------------------
// Beacon y seguridad (R-29, E-22)
// -----------------------------------------------------------------------------
namespace beacon {
constexpr uint8_t kVersion = 1;
constexpr uint8_t kNodeTypeTren = 1;

// Ventana de frescura en el receptor. Un beacon con más de 2 s de antigüedad
// se descarta aunque la autenticación sea correcta, así una grabación
// reinyectada no puede mostrar una posición vieja del tren. Se tolera un poco
// de antigüedad negativa por diferencias de reloj entre nodos.
constexpr int32_t kMaxAgeMs = 2000;
constexpr int32_t kMinAgeMs = -500;

// E-9: sin beacon válido por más de 1 s el cruce pasa a "sin datos".
constexpr uint32_t kLinkTimeoutMs = 1000;

// El contador anti-repetición nunca puede repetirse, ni entre reinicios: el
// nonce del cifrado sale de él. Se reservan bloques en la flash (NVS) y se
// escribe una vez cada 65536 beacons (1,8 h a 10 Hz).
constexpr uint32_t kCounterBlock = 65536;

// Claves AES-128 de PROTOTIPO. Están en el repositorio a la vista, así que no
// dan seguridad real: sirven para probar la cadena completa. En el producto
// cada nodo tendría su clave provisionada de forma segura. Se usan claves
// distintas para cifrar y para autenticar.
constexpr uint8_t kEncKey[16] = {0x50, 0x41, 0x4e, 0x44, 0x41, 0x2d, 0x45, 0x4e,
                                 0x43, 0x2d, 0x4b, 0x45, 0x59, 0x2d, 0x30, 0x31};
constexpr uint8_t kMacKey[16] = {0x50, 0x41, 0x4e, 0x44, 0x41, 0x2d, 0x4d, 0x41,
                                 0x43, 0x2d, 0x4b, 0x45, 0x59, 0x2d, 0x30, 0x31};
}  // namespace beacon

// -----------------------------------------------------------------------------
// Lógica de decisión del nodo cruce (fase 3)
//
// El cruce muestra NO SEGURO si se cumple CUALQUIERA de estas condiciones
// (cierre en OR), y solo se apaga cuando no se cumple NINGUNA (apertura en AND):
//   1. Un tren podría llegar al cruce en menos de kAlertEtaS (ETA mínimo).
//   2. Un tren está a menos de kOccupiedRadiusM: puede estar sobre el cruce.
//   3. Un tren en aproximación o en zona dejó de mandar datos válidos (E-9).
//   4. Un tren se escucha pero sin posición o sin tiempo verificable.
//   5. El circuito de vía está ocupado.
//   6. El propio nodo está en falla (sin radio, sin tiempo GPS, sin referencia).
// -----------------------------------------------------------------------------
namespace crossing {
// Período de la tarea de decisión. 20 Hz, el doble que los beacons.
constexpr uint32_t kPeriodMs = 50;

// E-8b: umbral de alerta. 12 s de la Tabla I del SETOP (barrera baja antes de
// que llegue el tren) + 5 s de preaviso (8.6.6) + 8 s de margen para latencias
// y para la maniobra de la barrera.
constexpr float kAlertEtaS = 25.0f;

// ETA mínimo: se supone que el tren puede acelerar a esta tasa desde la
// velocidad medida. Un tren detenido cerca se considera que puede arrancar, y
// uno que acelera no toma desprevenido al cruce. 1,0 m/s² cubre a las
// formaciones eléctricas del AMBA. Con el perfil de tracción de abajo, un tren
// detenido a menos de 290 m mantiene el cruce en NO SEGURO, y la alerta de un
// tren a 100 km/h arranca a 792 m (28,5 s antes de que llegue).
constexpr float kMaxAccelMps2 = 1.0f;

// Perfil de tracción máximo que se le supone al tren para el ETA mínimo:
//   hasta kAccelKneeMps     aceleración constante kMaxAccelMps2
//   luego, hasta la máxima  potencia constante: a = a0 * vk / v
//   kLineMaxSpeedMps        velocidad máxima de la línea, no la supera
// Sin la parte de potencia constante, a 100 km/h se supondría 1 m/s² y la
// alerta se adelantaría unos 11 s de más, que es justo la ineficiencia que
// PANDA quiere reducir. 11,1 m/s = 40 km/h. 33,3 m/s = 120 km/h.
constexpr float kAccelKneeMps = 11.1f;
constexpr float kLineMaxSpeedMps = 33.3f;

// Radio de ocupación: la antena GNSS está en un solo coche y el resto de la
// formación puede estar sobre el cruce. Con un tren de hasta kMaxTrainLengthM,
// cualquier tren más cerca que eso más un margen se considera sobre el cruce.
// Para cargas largas hay que subirlo (la protección de fondo es el circuito de vía).
constexpr float kMaxTrainLengthM = 250.0f;
constexpr float kOccupiedMarginM = 30.0f;
constexpr float kOccupiedRadiusM = kMaxTrainLengthM + kOccupiedMarginM;

// Velocidad de acercamiento mínima para considerar que un tren se aproxima.
// Por debajo es ruido del GNSS.
constexpr float kMinClosingMps = 0.5f;

// Para apagar la alerta de un tren, tiene que cumplirse "fuera de peligro"
// durante este tiempo seguido. Evita parpadeos por ruido del GNSS.
constexpr uint32_t kClearHoldMs = 3000;

// Un tren que no se escucha hace más de esto se olvida si lo último que se
// supo es que se alejaba o estaba lejos. Si estaba en aproximación queda en
// SIN DATOS hasta que vuelva el enlace, pase por el circuito de vía o venza
// kSilentReleaseMs (se registra como LIBERADO_POR_TIEMPO).
constexpr uint32_t kForgetMs = 10000;
constexpr uint32_t kSilentReleaseMs = 120000;

// Más allá de esta distancia un tren no es relevante para este cruce aunque
// se acerque (queda registrado pero no alerta ni genera SIN DATOS).
constexpr float kRelevantRadiusM = 5000.0f;

// El nodo necesita su propio tiempo GPS para validar beacons. Sin tiempo por
// más de esto, el nodo pasa a FALLA.
constexpr uint32_t kOwnTimeLossMs = 5000;

// Referencia del cruce. Si no hay una guardada (comando "c" en la consola) se
// usa el promedio del GNSS propio de los últimos kRefAvgWindow fixes válidos.
// También se puede fijar acá con coordenadas medidas (grados * 1e7, 0 = no).
constexpr int32_t kFixedRefLatE7 = 0;
constexpr int32_t kFixedRefLonE7 = 0;
constexpr uint32_t kRefAvgWindow = 600;  // 60 s a 10 Hz

// Circuito de vía. En false, el nodo trabaja sin esa entrada (queda como "no
// conectado") y se puede simular con el comando "v" de la consola. Pasar a
// true recién con el optoacoplador cableado: con la entrada al aire se lee
// vía OCUPADA, que es lo seguro.
constexpr bool kTrackCircuitEnabled = false;
constexpr uint32_t kTrackDebounceMs = 50;

// Vigilancia de la tarea de decisión: si no refresca las salidas en este
// tiempo, un timer independiente en el otro núcleo las lleva a estado seguro.
constexpr uint32_t kHeartbeatTimeoutMs = 300;

// Aviso sonoro: un toque por segundo mientras el cruce está NO SEGURO por un
// tren o por el circuito de vía (SETOP 8.6.7 usa el mismo ritmo en la campana).
constexpr uint32_t kBeepHz = 2500;
constexpr uint32_t kBeepOnMs = 200;
constexpr uint32_t kBeepPeriodMs = 1000;
}  // namespace crossing

// -----------------------------------------------------------------------------
// Simulador de trenes por USB (banco, fase 3)
//
// Permite probar la lógica del cruce con una sola placa: tools/sim_trenes.py
// manda beacons reales (cifrados y autenticados) por la consola serie y el
// nodo los procesa por el mismo camino que los que llegan por radio.
// También puede dar el tiempo GPS desde la PC cuando el nodo no tiene fix
// bajo techo. Todo lo simulado queda marcado en los registros.
// ESTO ES SOLO PARA BANCO: en un nodo instalado tiene que estar en false.
// -----------------------------------------------------------------------------
namespace sim {
constexpr bool kAllowInjection = true;
constexpr bool kAllowPcTime = true;
constexpr uint32_t kPcTimeHoldMs = 5000;
}  // namespace sim

// -----------------------------------------------------------------------------
// Registro en microSD
// -----------------------------------------------------------------------------
namespace sdlog {
// SPI compartido con la IMU. 20 MHz es un compromiso seguro para tarjetas
// genéricas con cables cortos de placa.
constexpr uint32_t kSdSpiHz = 20000000;

// Cola entre las tareas que producen datos (core 1) y el registrador (core 0).
// Vive en PSRAM: 2048 registros son unos 9 s de IMU, sobra para cualquier
// demora de la tarjeta.
constexpr uint32_t kQueueDepth = 2048;

// Cada archivo acumula en RAM y escribe en bloques grandes. Se hace flush cada
// 2 s para acotar lo que se pierde si se corta la alimentación.
constexpr size_t kFileBuffer = 16384;
constexpr size_t kWriteThreshold = 12288;
constexpr uint32_t kFlushPeriodMs = 2000;
constexpr uint32_t kRemountPeriodMs = 5000;
}  // namespace sdlog

// -----------------------------------------------------------------------------
// Telemetría (capa de inteligencia, fuera del lazo de seguridad)
// -----------------------------------------------------------------------------
namespace telemetry {
// Una posición por segundo alcanza para ver el tren en el mapa. El dato
// completo a 10 Hz queda en la microSD.
constexpr uint32_t kPeriodMs = 1000;
constexpr uint32_t kHttpTimeoutMs = 2000;
constexpr uint32_t kWifiRetryMs = 10000;
// No se sube un dato más viejo que esto.
constexpr uint32_t kMaxFixAgeMs = 2000;
// El cruce sube su propia posición solo cada tanto, porque no se mueve.
constexpr uint32_t kCruceOwnPositionPeriodMs = 15000;
}  // namespace telemetry

// -----------------------------------------------------------------------------
// Interfaz: OLED, botón y consola serie
// -----------------------------------------------------------------------------
namespace ui {
// 5 Hz de refresco. Cada refresco del SH1106 por I2C a 400 kHz tarda ~25 ms y
// corre en el core 0, así que nunca demora al lazo de seguridad.
constexpr uint32_t kRefreshPeriodMs = 200;
constexpr uint32_t kI2cHz = 400000;
constexpr uint32_t kButtonDebounceUs = 50000;
// Pulsación corta: marca. Pulsación larga: cambio de perfil de radio.
constexpr uint32_t kLongPressUs = 1500000;
constexpr uint32_t kStatusPeriodMs = 1000;
}  // namespace ui

// -----------------------------------------------------------------------------
// Tareas de FreeRTOS: núcleo y prioridad
//
// Core 1 = lazo de seguridad (radio, decisión, GNSS, IMU).
// Core 0 = todo lo demás (microSD, Wi-Fi, pantalla). El stack de Wi-Fi del
// ESP32 ya corre en el core 0, así queda separado del lazo crítico (R-31).
// -----------------------------------------------------------------------------
namespace task {
constexpr int kCoreSafety = 1;
constexpr int kCoreAux = 0;

constexpr unsigned kPrioRadio = 22;
constexpr unsigned kPrioDecision = 21;
constexpr unsigned kPrioGnss = 20;
constexpr unsigned kPrioImu = 18;
constexpr unsigned kPrioLogger = 8;
constexpr unsigned kPrioTelemetry = 5;
constexpr unsigned kPrioUi = 3;

constexpr uint32_t kStackRadio = 6144;
constexpr uint32_t kStackDecision = 8192;
constexpr uint32_t kStackGnss = 6144;
constexpr uint32_t kStackImu = 6144;
constexpr uint32_t kStackLogger = 8192;
constexpr uint32_t kStackTelemetry = 8192;
constexpr uint32_t kStackUi = 6144;
}  // namespace task

}  // namespace cfg
