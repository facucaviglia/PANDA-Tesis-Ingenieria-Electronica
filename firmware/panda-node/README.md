# PANDA · Firmware de nodo

Firmware para el **LILYGO T-Beam Supreme (V3)** de 915 MHz con SX1262 y u-blox MAX-M10S.
Un solo código con varios roles, elegidos al compilar.

| Fase | Rol | Estado |
|---|---|---|
| 1 | `registrador`: GNSS 10 Hz + IMU a microSD, posición en vivo a Traccar | Lista |
| 2 | `tren` y `cruce`: enlace LoRa, beacon de 35 bytes cifrado y autenticado, TDMA | Lista |
| 3 | `cruce`: lógica de decisión, ETA, estados, circuito de vía, salidas, simulador | **Lista** |
| 4 | IMU: plausibilidad del GNSS, quieto o en marcha, huecos cortos | Pendiente |

## 1. Qué hace falta

- VS Code con la extensión **PlatformIO IDE**.
- Cable USB-C de datos (algunos cables solo cargan).
- microSD de 4 a 32 GB en **FAT32** por placa (opcional, sin tarjeta no se graba pero el resto anda).
- Para el enlace: **dos placas, cada una con su antena de 915 MHz conectada**.

> **Nunca encender un nodo `tren` sin antena.** Transmite desde que arranca y el SX1262 se puede dañar.

## 2. Configurar el hotspot y Traccar

1. Abrí `include/secrets.h` (no se sube a git) y completá `WIFI_SSID` y `WIFI_PASSWORD` con los datos del hotspot. En iPhone el nombre es el del teléfono (Ajustes > General > Información > Nombre).
2. En el iPhone: Ajustes > Compartir Internet > activar **Permitir a otros conectarse** y **Maximizar compatibilidad**. Sin esto emite en 5 GHz y el ESP32 no lo ve.
3. El iPhone apaga el hotspot si nadie se conecta durante un rato. Dejá abierta la pantalla de Compartir Internet hasta que la placa se conecte. Si se corta, la placa reintenta sola cada 10 s.
4. En <https://demo.traccar.org> creá una cuenta y da de alta los dispositivos que vayas a usar, con estos **identificadores**:

| Identificador | Qué muestra | Lo sube |
|---|---|---|
| `panda-itba-tren` | El tren según su propio GNSS | Nodo tren |
| `panda-itba-tren-lora` | El tren según lo que le llegó por radio al cruce | Nodo cruce |
| `panda-itba-cruce` | La posición del nodo cruce (cada 15 s) | Nodo cruce |
| `panda-itba-reg` | El registrador | Nodo registrador |

Con `panda-itba-tren` y `panda-itba-tren-lora` en el mismo mapa se ve el enlace en vivo: cuando se corta la radio, el segundo punto se congela y el primero sigue.

Para verlo en el celular: app **Traccar Manager** con servidor `https://demo.traccar.org`. El demo es para pruebas y lo pueden borrar. El prefijo `panda-itba` se cambia en `TRACCAR_ID_PREFIX`.

## 3. Compilar y subir

1. Abrí la carpeta `firmware/panda-node` en VS Code (la carpeta, no el repo entero).
2. En la barra inferior de PlatformIO elegí el entorno: **env:tren**, **env:cruce** o **env:registrador**.
3. Conectá la placa por USB-C y tocá **Upload** (flecha).
4. Si no la detecta: mantené apretado **BOOT**, tocá **RST**, soltá BOOT y volvé a subir.
5. Abrí el **Serial Monitor** a 115200 baud. Escribí `h` y Enter para ver los comandos.

Desde la terminal:

```bash
pio run -e tren -t upload -t monitor
```

## 4. Prueba de banco del enlace

1. Cargá `tren` en una placa y `cruce` en la otra, **las dos con antena**.
2. Separalas al menos 1 m. La potencia arranca en **modo BANCO (2 dBm)** para no saturar al receptor.
3. Encendé primero el cruce y después el tren.
4. Lo que tiene que pasar:
   - En el tren, `TX` sube de a 10 por segundo.
   - En el cruce, la consola muestra `RX ok...` o `sint...` subiendo de a 10 por segundo, con RSSI y SNR.
5. **Sin fix de GNSS (bajo techo)** el cruce muestra **SIN DATOS** aunque reciba todo. Es lo correcto: sin tiempo GPS no se puede verificar que el beacon sea fresco, y un dato no verificable nunca es un dato válido. Los paquetes figuran como `SIN_TIEMPO` y sirven igual para medir el enlace.
6. **Con fix en las dos placas** (cerca de una ventana o afuera), el cruce pasa a mostrar el tren con la antigüedad del dato en ms, la pérdida (PER) y la distancia entre nodos.

El RSSI y el SNR a 1 m con 2 dBm deberían estar muy altos (RSSI entre -30 y -45 dBm). Si están bajos, revisá las antenas.

## 4 bis. Nodo cruce: lógica de decisión (fase 3)

### Estados

| Estado | Señal peatonal | Relé PANDA libre | Relé PANDA operativo | Sonido |
|---|---|---|---|---|
| **APAGADO** | apagada (no afirma nada, nunca hay verde) | energizado | energizado | no |
| **NO SEGURO** | encendida | desenergizado = pedido de cierre | energizado | 1 toque por segundo |
| **FALLA** | encendida | desenergizado | desenergizado = ignorar a PANDA | no |
| **INICIANDO** (3 s) | encendida (prueba de lámpara) | desenergizado | desenergizado | no |

### Cuándo es NO SEGURO (cierre en OR)

Basta con una sola de estas condiciones. El cruce se apaga recién cuando no se cumple ninguna (apertura en AND):

1. **Tren aproxima**: su **ETA mínimo** es de 25 s o menos (E-8b, `kAlertEtaS`).
2. **Tren en zona**: está a menos de 280 m. La antena GNSS va en un solo coche, y el resto de una formación de hasta 250 m puede estar sobre el cruce.
3. **SIN DATOS**: un tren que se acercaba o estaba en zona dejó de mandar datos válidos por más de 1 s (E-9). Un tren lejos que se calla pasa a SIN DATOS si, en el peor caso, pudo haber entrado en la zona de alerta durante el silencio.
4. **Tren sin posición o no verificable**: se lo escucha, pero sin fix o sin tiempo para verificar que el dato sea fresco.
5. **Vía ocupada**: el circuito de vía detecta un tren.

Pasa a **FALLA** si el nodo no puede cumplir su función: la tarea de radio no responde, no tiene tiempo GPS propio o no tiene la posición del cruce.

### ETA mínimo

Es el menor tiempo en que el tren *podría* llegar si acelerara al máximo con un perfil de tracción real: 1,0 m/s² hasta 40 km/h, después potencia constante y un tope en la velocidad de la línea (120 km/h). La distancia es en línea recta: en una vía curva sale más corta que la real y la alerta sale antes, así que el error va del lado seguro.

| Velocidad | Alerta desde | Anticipación real a velocidad constante |
|---|---|---|
| detenido | 290 m | (puede arrancar) |
| 40 km/h | 489 m | 44 s |
| 60 km/h | 583 m | 35 s |
| 100 km/h | 792 m | 28,5 s |
| 120 km/h | 833 m | 25,0 s |

A velocidad alta PANDA alerta casi justo a los 25 s, y a velocidad baja es conservador, porque un tren lento sí puede acelerar. Un tren detenido a más de 290 m **no** mantiene el cruce en NO SEGURO: esa es la ganancia de eficiencia frente al sistema estático.

### Liberación de un tren en SIN DATOS

- Vuelve a transmitir y los datos muestran que ya no hay peligro.
- El circuito de vía se ocupa y se libera durante el silencio: el tren pasó.
- A los 120 s de silencio, como último recurso. Se registra como `SilentRelease`. Para la barrera sigue mandando el circuito de vía (apertura en AND), así que esto no puede adelantar una apertura.

Un tren que se alejaba y se deja de escuchar se olvida a los 10 s.

### Posición del cruce

Por orden de prioridad:

1. **Simulador** (`R`, solo en RAM).
2. **Guardada** con el comando `c`: promedio de los últimos 60 s de GNSS propio, con al menos 10 s de fix bueno. `C` la borra.
3. `kFixedRefLatE7` y `kFixedRefLonE7` en `config.h`.
4. Promedio en vivo del GNSS propio, a partir de 5 s de fix.

Para un ensayo, dejá el nodo quieto en el lugar unos minutos y guardá con `c`.

### Vigilancia

- **Vigilante de salidas**: un timer en el core 0 verifica que la decisión (core 1) refresque las salidas cada 50 ms. A los 300 ms sin refresco las lleva a estado seguro por su cuenta.
- **Watchdog de tareas** del ESP-IDF: si la decisión no vuelve en 5 s, reinicia el micro. Durante el reinicio los pull-down externos dejan todo en estado seguro.
- Falta sumar el **watchdog externo TPL5010** (acción del DFMEA).

## 4 ter. Cableado del nodo cruce

Todos los pines están en el header derecho. Los GPIO 45 y 46 no se usan porque son pines de arranque del ESP32-S3.

| GPIO | Función | Estado seguro |
|---|---|---|
| 21 | Relé **PANDA libre** | bajo = pedido de cierre |
| 38 | Relé **PANDA operativo** | bajo = el controlador ignora a PANDA |
| 39 | **Señal peatonal** NO SEGURO (LED rojo) | la maneja la lógica |
| 48 | **Sonido** (PWM de 2,5 kHz a un buzzer o amplificador) | apagado |
| 2 | Entrada **circuito de vía** por optoacoplador | alto (abierto) = OCUPADA |

- Cada salida maneja su carga con un transistor (o un módulo de relé con optoacoplador) y lleva **un pull-down de 10 kΩ a GND en el GPIO**. Así, sin alimentación, en un reinicio o con el micro colgado, la salida queda en bajo, que es su estado seguro.
- Los relés de PANDA se usan con lógica **"energizado = permiso"**: cualquier falla (se corta un cable, se quema la bobina, se cae el nodo) produce el estado seguro.
- Circuito de vía: el optoacoplador conduce (pin a GND) con la vía **libre**. Con la entrada al aire se lee OCUPADA. Por eso viene desactivada en `config.h` (`kTrackCircuitEnabled = false`) y se simula con `v`. Activala recién con el opto cableado.

**Controlador de barrera de referencia.** En el producto, la combinación vive en el controlador de la barrera, no en PANDA. El nodo la calcula igual, la muestra en pantalla (`bar BAJA/alta`) y la registra, para demostrar el invariante:

- PANDA operativo: la barrera baja si PANDA pide cierre **o** la vía está ocupada.
- PANDA no operativo: la barrera baja si la vía está ocupada, que es el comportamiento actual.

## 4 quater. Probar el cruce con una sola placa: simulador de trenes

`tools/sim_trenes.py` genera trenes sobre una vía recta, arma beacons **reales** (cifrados y autenticados) y se los manda al nodo cruce por el USB. El nodo los procesa por el mismo camino que los de radio: CRC, CMAC, contador, frescura y decisión. Si el nodo no tiene fix, el simulador también le da el tiempo GPS desde el reloj de la PC.

1. Cargá `cruce` en la placa. No hace falta antena ni fix.
2. **Cerrá el monitor serie de PlatformIO**: el puerto no se puede compartir.
3. Corré, con el Python de PlatformIO, que ya trae pyserial:

```bash
~/.platformio/penv/Scripts/python.exe tools/sim_trenes.py --puerto COM5 --escenario rapido
```

El simulador muestra todo lo que imprime el nodo. Los cambios de estado salen en amarillo.

> Si la placa tiene fix propio usa su tiempo GPS e ignora el de la PC. En ese caso el reloj de la PC tiene que estar sincronizado (en Windows: Configuración > Hora e idioma > Sincronizar ahora). Con más de 2 s de diferencia, los beacons simulados se rechazan como `VIEJO` o `FUTURO`, y eso es justamente la protección funcionando.

Para ver qué *debería* pasar sin la placa, está el modelo de referencia en Python (`tools/modelo_cruce.py`, con las mismas reglas que `crossing.cpp`):

```bash
python tools/sim_trenes.py --seco --escenario rapido
```

| Escenario | Qué prueba | Resultado esperado |
|---|---|---|
| `rapido` | Tren a 100 km/h desde 1500 m | NO SEGURO a 792 m, en zona a 280 m, PASO, APAGADO 13 s después |
| `lento` | Tren a 40 km/h desde 600 m | NO SEGURO a 489 m |
| `detenido` | Frena y para a 500 m, espera 20 s, arranca | **No** alerta mientras está parado. Alerta a 401 m al arrancar |
| `perdida` | Sin enlace entre 900 y 500 m | SIN DATOS a los 3,3 s de silencio, vuelve con datos |
| `silencio` | El nodo del tren muere a 700 m | SIN DATOS hasta que el circuito de vía (simulado) lo libera |
| `dos` | Dos trenes en sentidos opuestos | Dos ciclos completos, cada uno con su PASO |
| `repeticion` | Un atacante reinyecta un paquete grabado | Se rechaza como `REPETIDO`, sin efecto en el estado |
| `cmac` | Paquetes falsos sin la clave | Se rechazan como `CMAC`, sin efecto |
| `sin_posicion` | Tren que se escucha sin fix | NO SEGURO (sin posición) hasta que da posición lejos |
| `via` | Vía ocupada sin tren equipado | NO SEGURO por vía y un evento "vía ocupada con PANDA libre" |

> La inyección por USB y el tiempo desde la PC son **solo para banco**. En un nodo instalado hay que poner `kAllowInjection` y `kAllowPcTime` en `false` (`config.h`, sección `sim`).

## 5. Botón y consola

| Acción | Efecto |
|---|---|
| Botón BOOT corto | **Marca** numerada en `events.csv` (tiempo exacto y tiempo GPS) |
| Botón BOOT largo (1,5 s) | **Siguiente perfil de radio**. Hay que hacerlo en los DOS nodos |
| `h` | Ayuda |
| `i` | Información del nodo, del perfil y de la ranura TDMA |
| `m` | Marca, igual que el botón |
| `p` | Siguiente perfil de radio |
| `s` | **Barrido de canales**: piso de ruido en los 25 canales de 500 kHz entre 915,5 y 927,5 MHz |
| `w` | (solo tren) Potencia **BANCO 2 dBm / CAMPO 22 dBm** |
| `c` / `C` | (solo cruce) Guardar / borrar la posición del cruce |
| `v` | (solo cruce) Simular el circuito de vía: alterna LIBRE y OCUPADA |
| `x` | (solo cruce) Silenciar o activar el sonido |

El perfil y la potencia quedan guardados en la flash y sobreviven a un reinicio.

Perfiles de radio (todos LoRa, header implícito, 925,0 MHz):

| # | Perfil | Aire del beacon | Sensibilidad aprox. | Ranuras en 100 ms |
|---|---|---|---|---|
| 0 | SF7 / 500 kHz / CR 4/5 | 18 ms | -117 dBm | 5 |
| 1 | SF6 / 500 kHz / CR 4/5 | 10 ms | -114 dBm | 8 |
| 2 | SF7 / 250 kHz / CR 4/5 | 36 ms | -121 dBm | 2 |
| 3 | SF7 / 500 kHz / CR 4/8 | 26 ms | corrige errores de desvanecimiento | 3 |

## 6. Qué muestra la pantalla

**Tren**

```
TREN SD+ WF+ 87%        microSD, Wi-Fi, batería
3D sv14 hAcc 1.8m       fix propio
54.3 km/h  R 123        velocidad y rumbo
TX 12345 r2/5           beacons enviados, ranura 2 de 5 (SIN SINC si no hay tiempo GPS)
SF7/500 2dBm e58        perfil, potencia, antigüedad del fix al transmitir en ms
ID 3FA2 S3 d0 T:ok      id del nodo, sesión de registro, descartes, Traccar
```

**Cruce**

```
CRUCE SD+ WF+ USB
  NO SEGURO             estado en grande (invertido si es NO SEGURO)
tren aproxima A001      motivo y tren
792m 100km/h e25s       distancia al cruce, velocidad, ETA mínimo
bar BAJA via lib* P+    barrera de referencia, circuito de vía (* = simulado), PANDA operativo
```

## 7. Archivos en la microSD

Cada encendido crea `PANDA_NNNN`. Todos los archivos comparten `t_us` (µs desde el arranque de esa placa).

- `gnss.csv`, `imu.csv`, `events.csv`: iguales que en la fase 1 (el cruce no graba IMU).
- `tx.csv` (tren), un renglón por beacon:

| Columna | Descripción |
|---|---|
| `t_us` | Inicio de la transmisión |
| `counter` | Contador anti-repetición |
| `itow_ms` | Tiempo GPS del fix que viajó |
| `tx_age_ms` | Antigüedad de ese fix al transmitir (vacío sin tiempo GPS) |
| `air_us` | Duración medida en el aire |
| `status` | 0 = OK, código de RadioLib o -999 si no llegó el TX_DONE |
| `flags` | Bits del beacon: 1 fix, 2 tiempo GPS, 4 PPS, 8 quieto, 16 IMU ok |
| `slot` | Ranura TDMA (255 = sin sincronía) |
| `perfil`, `tiempo`, `pot_dbm` | Perfil de radio, calidad de tiempo (0 no, 1 grueso, 2 PPS), potencia |

- `rx.csv` (cruce), un renglón por paquete recibido:

| Columna | Descripción |
|---|---|
| `resultado` | `OK`, `SIN_TIEMPO`, `CRC`, `FORMATO`, `CMAC`, `REPETIDO`, `VIEJO`, `FUTURO` |
| `node_id`, `counter`, `itow_ms` | Identificación del beacon |
| `age_ms` | Antigüedad del dato al recibirlo: tiempo GPS del cruce menos tiempo GPS del fix |
| `gap` | Beacons perdidos entre este y el anterior del mismo tren |
| `rssi_dbm`, `snr_db`, `ferr_hz` | Calidad del enlace y error de frecuencia |
| `lat_deg` ... `num_sv` | Contenido del beacon |
| `dist_m` | Distancia entre nodos (vacío si falta algún fix) |
| `tiempo`, `perfil` | Calidad de tiempo del cruce (3 = PC) y perfil de radio |
| `origen` | `RADIO` o `USB` (simulador) |

- `decision.csv` (cruce), un renglón por beacon válido de cada tren y en cada cambio de estado:

| Columna | Descripción |
|---|---|
| `estado`, `motivo` | Estado del cruce y motivo principal |
| `tren`, `fase`, `alerta` | Tren evaluado, su fase (`LEJOS`, `APROXIMA`, `EN_ZONA`, `SE_ALEJA`, `SIN_DATOS`) y si sostiene el NO SEGURO |
| `dist_m`, `vel_mps`, `acerc_mps` | Distancia al cruce proyectada al instante actual, velocidad y velocidad de acercamiento |
| `eta_cv_s`, `eta_min_s` | ETA a velocidad constante y ETA mínimo |
| `edad_ms` | Antigüedad del dato usado |
| `panda_libre` ... `barrera_baja` | Salidas y entradas en ese instante |

Para validar E-8a: el PASO real queda en `events.csv` (automático, por mínimo de distancia, y con el botón), y se compara con `t_us + eta_cv_s` de los renglones anteriores.

## 8. Beacon y seguridad

35 bytes: cabecera en claro (versión, id, contador), 20 bytes cifrados con **AES-128-CTR** y etiqueta **AES-CMAC** de 8 bytes. El formato exacto está en `include/beacon.h`.

El cruce valida en este orden y descarta al primer fallo: CRC, formato, **CMAC**, **contador** (anti-repetición) y **frescura** (antigüedad entre -0,5 y 2 s, E-22). Nada se descifra sin autenticar.

- El contador se reserva en bloques en la flash y **nunca se repite, ni entre reinicios**.
- En cada arranque la placa verifica AES y CMAC con los vectores oficiales (FIPS-197, RFC 4493, NIST SP 800-38A) y arma un beacon de referencia que tiene que coincidir byte a byte con el de la PC. Si algo falla, el nodo no arranca.
- `python tools/verify_crypto.py` corre las mismas pruebas en la PC, sin dependencias, y sirve de decodificador de referencia.
- **Las claves son de prototipo y están en el repositorio.** Sirven para probar la cadena completa, no dan seguridad real. En el producto cada nodo tendría su clave provisionada.

## 9. Tiempos y TDMA

- El 1PPS del GNSS ancla el reloj de cada placa al tiempo GPS con error de microsegundos. Los dos nodos comparten así la misma escala de tiempo.
- La trama de 100 ms empieza en cada época GNSS. La ranura k arranca 50 ms después más k veces el largo de ranura, y el tren toma la ranura `id módulo ranuras`.
- El tren espera su ranura durmiendo y los últimos 1,5 ms en espera activa, así arranca con error de microsegundos.
- El cruce calcula la antigüedad de cada beacon con su propio tiempo GPS: es la medición directa de E-6.

## 10. Arquitectura

```
Core 1 (lazo de seguridad)                 Core 0 (auxiliar)
┌───────────────────┐                      ┌───────────────┐
│ link  prio22      │── estado enlace ───▶ │ ui         3  │──▶ OLED, botón, batería
│ TX en ranura o RX │                      └───────────────┘
└─────────┬─────────┘                      ┌───────────────┐
          │ beacons autenticados           │ console    4  │◀── USB (operador y simulador)
┌─────────▼─────────┐                      └───────────────┘
│ decision prio21   │──▶ relés, señal, sonido     ▲ vigilante de salidas (timer, core 0)
│ 20 Hz, solo cruce │◀── circuito de vía
└───────────────────┘                      ┌───────────────┐
┌───────────────────┐  último fix          │ telemetry  5  │──▶ Wi-Fi ──▶ Traccar
│ gnss  prio20      │────────────────────▶ └───────────────┘
│ NAV-PVT, 1PPS     │──┐                   ┌───────────────┐
└───────────────────┘  │ cola en PSRAM     │ sdlog      8  │──▶ microSD
┌───────────────────┐  ├─────────────────▶ └───────────────┘
│ imu   prio18      │──┘
└───────────────────┘
```

- La radio tiene su propio bus SPI. La microSD y la IMU comparten el otro.
- Solo la tarea `link` toca la radio. Los cambios de perfil, potencia y barrido le llegan como pedidos.
- La telemetría corre aislada en el core 0. Si el Wi-Fi o el servidor fallan, el core 1 no se entera (R-31).

## 11. Pinout verificado

Ver `include/board_pins.h`. Diferencias importantes con lo que suele circular:

- La PMU está en **Wire1 (SDA 42, SCL 41)**, no en 17/18. El OLED sí está en 17/18.
- El GPIO 42 **no es un LED**. El único LED controlable es el de carga de la PMU.
- La PSRAM es **QSPI**, no octal.
- Rieles: radio en **ALDO3**, GNSS en **ALDO4**, microSD en BLDO1/2, sensores en ALDO1/2.
- El SX1262 usa TCXO de **1,8 V** y DIO2 como switch de antena.

## 12. LED de la PMU

| Patrón | Significado |
|---|---|
| Normal (lo maneja el cargador) | Todo bien |
| Titila a 1 Hz | Advertencia (registrador sin radio) |
| Titila a 4 Hz | Falla fatal. Ver la pantalla o la consola |
