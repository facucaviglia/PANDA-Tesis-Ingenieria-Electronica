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

#define PANDA_FW_VERSION "0.4.1-panda-principal"

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
// Versión 2: precisión en dm y aceleración (ver beacon.h). Un nodo con otra
// versión se rechaza como formato inválido.
constexpr uint8_t kVersion = 2;
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
// Lógica de decisión del nodo cruce
//
// PANDA es el sistema PRINCIPAL del cruce (decisión del 1-oct-2026) y el
// circuito de vía queda de respaldo. El cruce muestra NO SEGURO y pide el
// cierre si se cumple CUALQUIERA de estas condiciones:
//   1. Un tren podría llegar al cruce en menos de kAlertEtaS aunque acelerara
//      al máximo que permite su material rodante (ETA de peor caso).
//   2. Un tren está a menos de kOccupiedRadiusM: puede estar sobre el cruce.
//   3. Un tren en aproximación o en zona dejó de mandar datos válidos (E-9).
//   4. Un tren se escucha sin posición, sin tiempo verificable o con datos
//      inconsistentes (plausibilidad).
//   5. El circuito de vía está ocupado y PANDA no sigue al tren que lo ocupó
//      (un tren sin nodo, o uno con el nodo caído): cierra como siempre.
//   6. El propio nodo está en falla: se declara no operativo y la barrera
//      vuelve a seguir solo al circuito de vía.
// Una vez pedido el cierre, se libera recién cuando ningún tren que se acerca
// puede llegar en kReleaseEtaS (equivalente del sector de aproximación).
// Lo nuevo frente al sistema actual: si el circuito se ocupa por un tren que
// PANDA sigue y ese tren todavía no puede llegar en kAlertEtaS, la barrera
// queda alta. Ahí está la ganancia para los trenes que no vienen a la máxima.
//
// Fuente de los tiempos: Anexo XII de ADIF, "Barreras automáticas en pasos a
// nivel y anuncios en pasos peatonales" (pliego de la Línea Roca), puntos 19
// y 20, que a su vez toman la Tabla I del SETOP 7/81.
// -----------------------------------------------------------------------------
namespace crossing {
// Período de la tarea de decisión. 20 Hz, el doble que los beacons.
constexpr uint32_t kPeriodMs = 50;

// --- Ciclo de la barrera según ADIF (Anexo XII, punto 20) -------------------
// Fonoluminosa: luces y campana antes de que empiece a bajar el brazo.
constexpr float kFonoluminosaS = 7.0f;
// Bajada del brazo: entre 5 y 10 s según el pliego, se calcula con 10 s.
constexpr float kArmDownS = 10.0f;
// Despejamiento: brazo ya abajo hasta que llega el tren. 12 s si la
// separación entre rieles extremos es de hasta 5 m (vía simple), 14 s entre 5
// y 10 m (vía doble, el caso típico del AMBA) y 16 s entre 10 y 15 m.
constexpr float kClearanceS = 14.0f;
constexpr float kBarrierCycleS = kFonoluminosaS + kArmDownS + kClearanceS;  // 31 s

// Sector de aproximación (Anexo XII, puntos 19 y 20): subida del brazo (lo
// mínimo que permita el mecanismo) más 5 s de espera desde que el brazo llega
// arriba hasta que se puede reiniciar el ciclo. La barrera no sube si hay
// otro tren en ese sector. En el prototipo la subida es la de la maqueta. En
// el producto va la subida medida del mecanismo instalado.
constexpr float kArmUpS = 3.0f;
constexpr float kApproachWaitS = 5.0f;

// --- Paso peatonal sin barrera (Anexo XII, punto 4.2) ------------------------
// Si el cruce es solo peatonal, la señal se enciende t_sem = t_p + 3 s antes
// del tren, con t_p = d_p / 0,7 m/s (velocidad del peatón de la Ley 22.431).
// Durante t_p el rojo es intermitente cada medio segundo y después queda fijo.
constexpr bool kHasBarrier = true;
constexpr float kPedestrianPathM = 12.0f;   // Distancia entre líneas de detención
constexpr float kPedestrianSpeedMps = 0.7f;
constexpr float kPedestrianCrossS = kPedestrianPathM / kPedestrianSpeedMps;  // t_p = 17,1 s
constexpr float kPedestrianWarnS = kPedestrianCrossS + 3.0f;                  // t_sem = 20,1 s

// Margen propio de PANDA: período de decisión, salida y reacción del
// controlador. La antigüedad del dato ya se compensa proyectando la posición.
constexpr float kLatencyMarginS = 1.0f;

// E-8b: umbral de decisión. Con barrera, 31 + 1 = 32 s.
constexpr float kAlertEtaS = (kHasBarrier ? kBarrierCycleS : kPedestrianWarnS) + kLatencyMarginS;

// Umbral para LIBERAR, equivalente en tiempo al sector de aproximación.
// Mientras PANDA pide el cierre, un tren que se acerca lo sigue sosteniendo
// hasta que su ETA de peor caso supere 32 + 3 + 5 = 40 s. Así, cuando PANDA
// libera, ningún tren que se acerca puede volver a pedir el cierre antes de
// que el brazo suba y pasen los 5 s de espera. Solo se aplica a trenes que se
// acercan: uno detenido no está "en aproximación", y si arranca, el umbral de
// cierre de 32 s le sigue dando el ciclo completo (SETOP 8.6.13).
constexpr float kReleaseEtaS = kAlertEtaS + (kHasBarrier ? kArmUpS + kApproachWaitS : 0.0f);

// --- Perfil de tracción de peor caso de la flota (línea piloto: Roca) -------
// El ETA de peor caso supone que el tren acelera al máximo desde la velocidad
// medida:
//   hasta kAccelKneeMps     aceleración constante kMaxAccelMps2
//   luego, hasta la máxima  potencia constante: a = a0 * vk / v
//   kLineMaxSpeedMps        velocidad máxima, no la supera
// Datos del CSR del Roca: aceleración publicada "superior a 0,8 m/s²", se toma
// 1,0 m/s² como cota hasta medirla en el viaje. La potencia publicada de la
// familia CSR es inconsistente (190 kW por motor da 3040 kW en 6 coches, la
// ficha dice 2160 kW). Con 3040 kW y 270 t vacío, P / (m a0) = 11,3 m/s: se
// toma el codo en 11,1 m/s (40 km/h), del lado seguro. Velocidad máxima de
// diseño 120 km/h. Con este perfil un tren detenido es peligro hasta 448 m y
// uno a 120 km/h se avisa 32 s antes (1066 m), la norma más 1 s de margen.
// kLineMaxSpeedMps es la máxima del tramo del cruce. Si el tramo tiene un
// límite menor que 120 km/h y el ATS lo hace cumplir, conviene poner ese
// límite: la ganancia crece (a 80 km/h pasa de 5 s a 11 s con tope de 90).
constexpr float kMaxAccelMps2 = 1.0f;
constexpr float kAccelKneeMps = 11.1f;
constexpr float kLineMaxSpeedMps = 33.3f;

// Requisito de instalación: el nodo tren va en la CABINA DELANTERA, en el
// sentido de marcha. Así la posición del beacon es la del frente del tren (a
// pocos metros, cubiertos por el margen) y el ETA no se atrasa. En el producto
// van dos nodos por formación, uno por cabina (trabajo futuro).
//
// Radio de ocupación: con el nodo en el frente, después del paso el resto de
// la formación sigue sobre el cruce. Roca: hasta 8 coches de 25,8 m = 206 m.
// Los trenes de carga, más largos, no tienen nodo: los cubre el circuito.
constexpr float kMaxTrainLengthM = 210.0f;
constexpr float kOccupiedMarginM = 30.0f;
constexpr float kOccupiedRadiusM = kMaxTrainLengthM + kOccupiedMarginM;

// --- Circuito de vía y atribución -------------------------------------------
// El sector de operación de ADIF mide el ciclo completo a la velocidad del
// tren más rápido, que el pliego fija en 120 km/h: 31 s × 33,3 m/s = 1033 m.
// Cuando el circuito se ocupa, PANDA lo da por "explicado" solo si en ese
// instante sigue un tren con dato fresco y coherente que se acerca y está a
// kTrackCircuitDistM ± kAttributionTolM, del lado del circuito. Si no, es un
// tren que PANDA no ve y se cierra como siempre. Sirve además de verificación
// cruzada de la posición que manda el tren.
constexpr float kDesignSpeedMps = 120.0f / 3.6f;
constexpr float kTrackCircuitDistM = kBarrierCycleS * kDesignSpeedMps;
constexpr float kAttributionTolM = 100.0f;
// Lado del circuito: rumbo desde el cruce hacia la junta de aproximación, en
// grados. Negativo = no se verifica el lado (banco). Se puede fijar en RAM con
// el comando "Q distancia rumbo" del simulador.
constexpr float kTrackCircuitBearingDeg = -1.0f;
constexpr float kSideTolDeg = 45.0f;

// Circuito de vía físico. En false, el nodo trabaja sin esa entrada y se
// simula con el comando "v" de la consola. Pasar a true recién con la entrada
// cableada: con la entrada al aire se lee vía OCUPADA, que es lo seguro.
constexpr bool kTrackCircuitEnabled = false;
constexpr uint32_t kTrackDebounceMs = 50;

// --- Plausibilidad del dato del tren (acción del DFMEA, modo RPN 120) -------
// Cada beacon válido se compara con el último dato CONFIABLE del mismo tren
// (el ancla), no con el anterior: así un salto del GNSS que después se queda
// corrido no vuelve a parecer coherente. La distancia recorrida desde el ancla
// tiene que estar entre lo que el tren recorre frenando de emergencia y lo que
// recorre acelerando al máximo (perfil de arriba), con una tolerancia que
// crece con la precisión informada. También se rechaza un cambio de velocidad
// o una aceleración informada imposibles. Mientras el dato no vuelve a entrar
// en esa envolvente, el ancla no se mueve y el cruce queda en NO SEGURO, y
// sigue así kPlausHoldMs después. Frenado de emergencia del CSR: 1,2 m/s².
constexpr float kPlausPosTolM = 10.0f;
constexpr float kPlausHAccFactor = 3.0f;
constexpr float kPlausBrakeMps2 = 1.2f;
constexpr float kPlausAccelMps2 = 2.2f;
constexpr float kPlausSpeedTolMps = 0.5f;
// Un ancla más vieja que esto (silencio largo) se reemplaza sin comparar. El
// silencio lo cubre la regla de SIN DATOS.
constexpr float kPlausAnchorMaxAgeS = 30.0f;
constexpr uint32_t kPlausHoldMs = 2000;

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

// Vigilancia de la tarea de decisión: si no refresca las salidas en este
// tiempo, un timer independiente en el otro núcleo las lleva a estado seguro.
constexpr uint32_t kHeartbeatTimeoutMs = 300;

// Aviso sonoro de PANDA: un toque por segundo mientras el cruce está NO
// SEGURO (dentro de los 60 a 240 golpes por minuto del Anexo XII, punto 4.2).
// Con OTRO TREN el ritmo se duplica (el Anexo pide de 1,5 a 2 veces). La
// campana de la maqueta da siempre un toque por segundo (SETOP 8.6.7).
//
// Tono: el Anexo XII (5.5) pide para la campana alguna frecuencia de la
// quinta octava de la IRAM 4036, preferentemente "sol". Con La3 = 440 Hz
// (convención franco-belga, la usual en castellano) sol5 es 1568 Hz; con la
// convención científica (La4 = 440 Hz) sería 784 Hz. A VERIFICAR con la
// tabla II de la IRAM 4036. 1568 Hz además queda más cerca de la resonancia
// de un piezo pasivo, que a 784 Hz suena bastante más bajo.
// En la maqueta hay un solo buzzer para la campana y el aviso de PANDA, así
// que comparten tono. El Anexo no fija tono para el aviso peatonal.
constexpr uint32_t kBeepHz = 1568;
constexpr uint32_t kBeepOnMs = 200;
constexpr uint32_t kBeepPeriodMs = 1000;
constexpr uint32_t kBeepPeriodOtherTrainMs = 500;

// Nivel reducido (Anexo XII 5.5, obligatorio en el pliego; en el SETOP 8.6.7
// es optativo para la comuna): con el brazo horizontal la campana baja de
// 95 a 60 dB. Con el buzzer pasivo se baja el ciclo de trabajo del PWM (512 de
// 1023 es el nivel pleno). El valor que da 60 dB hay que ajustarlo con el
// sonómetro. Un buzzer activo no se puede atenuar: suena siempre pleno.
constexpr uint32_t kBellLowDuty = 24;

// Señales intermitentes del Anexo XII (rojo peatonal durante t_p y OTRO
// TREN): encendido y apagado de medio segundo cada uno.
constexpr uint32_t kFlashHalfMs = 500;

// --- Monitoreo de alarmas (Anexo XII, punto 22) ------------------------------
// g) "brazo de barrera levantado con circuito de vía ocupado". Con PANDA
// principal eso es lo normal para un tren lento atribuido, así que la alarma
// se redefine como "el brazo no sigue la regla del controlador": hay pedido
// de cierre (de PANDA o, sin PANDA operativo, del circuito) y el brazo está
// arriba o subiendo por más de este tiempo. Necesita la posición del brazo:
// en el prototipo la da la maqueta, en el producto la detección de posición
// que el punto 22 e) ya exige.
constexpr uint32_t kAlarmBarrierMs = 1000;
// f) circuito de vía ocupado por más de 10 minutos. No cambia con PANDA.
constexpr uint32_t kAlarmTrackLongMs = 600000;

// Tipo de buzzer en GPIO 48. Pasivo (sin oscilador interno): se maneja con PWM
// a kBeepHz. Activo (trae su oscilador, suena con tensión continua, el más
// común en las casas de electrónica): se maneja con nivel alto y bajo. Una
// sirena de 12 V se maneja igual que uno activo, a través de un transistor.
constexpr bool kBuzzerIsActive = false;

// --- Maqueta de barrera en la misma placa ------------------------------------
// En el prototipo el controlador de la barrera corre en la misma T-Beam del
// cruce, como un módulo aparte que solo ve los dos contactos de PANDA y el
// circuito de vía, igual que el controlador real. En true, los GPIO 21, 38 y
// 3 manejan el servo y las dos luces alternadas. En false, el 21 y el 38 son
// los relés "PANDA libre" y "PANDA operativo" hacia un controlador externo.
constexpr bool kBarrierOnBoard = true;
// Tiempos de la maqueta. La bajada está dentro de los 5 a 10 s del pliego. La
// subida es la que usa PANDA para el sector de aproximación (kArmUpS).
constexpr float kModelArmDownS = 6.0f;
constexpr float kModelArmUpS = kArmUpS;
constexpr uint32_t kServoUpUs = 1000;     // Brazo vertical (abierto)
constexpr uint32_t kServoDownUs = 2000;   // Brazo horizontal (cerrado)
constexpr uint32_t kBarrierLightHalfMs = 500;  // SETOP 8.6.5: alternan cada 0,5 s

// TODO(tpl5010): watchdog externo TPL5010 (acción del DFMEA para el bloqueo del
// micro). Queda para después de probar las dos placas. Plan:
//   - Pin DONE del TPL5010 en el GPIO 46 (pin de arranque: no ponerle pull-up).
//   - Salida RESET del TPL5010 al pin EN del ESP32.
//   - Pulso en DONE desde crossio::apply(), o sea solo si la tarea de decisión
//     está viva y refrescando las salidas.
// constexpr bool kExternalWdtEnabled = false;
// constexpr int kExternalWdtDonePin = 46;
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
// TODO(traccar): telemetría APAGADA hasta probar el enlace entre las dos
// placas. Para activarla: completar WIFI_SSID y WIFI_PASSWORD en secrets.h,
// dar de alta los dispositivos en Traccar y pasar esto a true. Apagada, el
// nodo ni enciende el Wi-Fi. Todo lo demás (radio, decisión, pantalla,
// consola, microSD) funciona igual.
constexpr bool kEnabled = false;

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
