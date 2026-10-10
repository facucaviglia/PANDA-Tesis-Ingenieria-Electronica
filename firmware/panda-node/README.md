# PANDA · Firmware de nodo

Firmware para el **LILYGO T-Beam Supreme (V3)** de 915 MHz con SX1262 y u-blox MAX-M10S.
Un solo código con varios roles, elegidos al compilar.

| Fase | Rol | Estado |
|---|---|---|
| 1 | `registrador`: GNSS 10 Hz + IMU a microSD (Traccar como TODO) | Lista |
| 2 | `tren` y `cruce`: enlace LoRa, beacon de 35 bytes cifrado y autenticado con aceleración, TDMA | Lista |
| 3 | `cruce`: PANDA principal con el ciclo de ADIF, atribución del circuito de vía, plausibilidad, OTRO TREN, maqueta de barrera, simulador | **Lista** |
| 4 | IMU: plausibilidad del GNSS contra la IMU, quieto o en marcha, huecos cortos | Pendiente |

Nada de esto corrió todavía en una placa. La lógica está verificada contra el modelo en Python y con la prueba en PC (`test_pc`). La versión 0.4.1 (revisión legal contra el Anexo XII y el SETOP) todavía no se compiló con PlatformIO: el primer paso de la sección 3 es justamente eso.

## 1. Qué hace falta

- VS Code con la extensión **PlatformIO IDE**.
- Cable USB-C de datos (algunos cables solo cargan).
- microSD de 4 a 32 GB en **FAT32** por placa (opcional, sin tarjeta no se graba pero el resto anda).
- Para el enlace: **dos placas, cada una con su antena de 915 MHz conectada**.

> **Nunca encender un nodo `tren` sin antena.** Transmite desde que arranca y el SX1262 se puede dañar.

## 2. Configurar el hotspot y Traccar (TODO, hoy desactivado)

> **Por ahora la telemetría está apagada** (`cfg::telemetry::kEnabled = false` en `include/config.h`). Se activa recién después de probar el enlace entre las dos placas. Mientras tanto toda la información se ve en la pantalla OLED de cada placa, en la consola serie y en la microSD. Esta sección queda para ese momento.

1. Pasá `cfg::telemetry::kEnabled` a `true`.
2. Abrí `include/secrets.h` (no se sube a git) y completá `WIFI_SSID` y `WIFI_PASSWORD` con los datos del hotspot. En iPhone el nombre es el del teléfono (Ajustes > General > Información > Nombre).
3. En el iPhone: Ajustes > Compartir Internet > activar **Permitir a otros conectarse** y **Maximizar compatibilidad**. Sin esto emite en 5 GHz y el ESP32 no lo ve.
4. El iPhone apaga el hotspot si nadie se conecta durante un rato. Dejá abierta la pantalla de Compartir Internet hasta que la placa se conecte. Si se corta, la placa reintenta sola cada 10 s.
5. En <https://demo.traccar.org> creá una cuenta y da de alta los dispositivos que vayas a usar, con estos **identificadores**:

| Identificador | Qué muestra | Lo sube |
|---|---|---|
| `panda-itba-tren` | El tren según su propio GNSS | Nodo tren |
| `panda-itba-tren-lora` | El tren según lo que le llegó por radio al cruce | Nodo cruce |
| `panda-itba-cruce` | La posición del nodo cruce (cada 15 s) | Nodo cruce |
| `panda-itba-reg` | El registrador | Nodo registrador |

Con `panda-itba-tren` y `panda-itba-tren-lora` en el mismo mapa se ve el enlace en vivo: cuando se corta la radio, el segundo punto se congela y el primero sigue.

Para verlo en el celular: app **Traccar Manager** con servidor `https://demo.traccar.org`. El demo es para pruebas y lo pueden borrar. El prefijo `panda-itba` se cambia en `TRACCAR_ID_PREFIX`.

## 3. Primera puesta en marcha: compilar y cargar las dos placas

El código es uno solo. Lo que cambia entre placas es el **entorno** que se elige al compilar:

| Placa | Entorno | Qué hace |
|---|---|---|
| **NC**, nodo cruce | `cruce` | Recibe los beacons, decide y maneja la maqueta, la señal peatonal y el sonido |
| **NT**, nodo tren | `tren` | Transmite su posición, velocidad y aceleración 10 veces por segundo |
| (otra) | `registrador` | Solo graba GNSS e IMU en la microSD, para los viajes |

No hace falta configurar nada antes: `secrets.h` (Wi-Fi) se usa recién cuando se active la telemetría, y sin él compila con `secrets_example.h`. Tampoco hace falta tener los LEDs, el buzzer ni el servo conectados: esas salidas quedan al aire y el resto funciona igual.

### Paso 1. Compilar sin placa

1. Abrí la carpeta `firmware/panda-node` en VS Code (la carpeta, no el repo entero). La primera vez PlatformIO descarga la plataforma del ESP32 y las librerías: tarda varios minutos.
2. En la barra inferior de PlatformIO elegí **env:cruce** y tocá **Build** (el tilde). Repetí con **env:tren**.
3. Las dos tienen que terminar en `SUCCESS`. Si alguna da error, copiá el mensaje completo (desde la primera línea que dice `error`) y pasalo para corregirlo.

Desde la terminal es lo mismo:

```bash
pio run -e cruce
pio run -e tren
```

### Paso 2. Cargar cada placa

**Poné la antena en cada placa antes de conectarla.** El nodo tren transmite apenas arranca y el SX1262 se puede dañar sin antena.

1. Conectá **solo la placa NC** por USB-C (cable de datos, no solo de carga).
2. Elegí **env:cruce** y tocá **Upload** (la flecha).
3. Desconectala, conectá **la placa NT**, elegí **env:tren** y tocá **Upload**.
4. Si no la detecta o no carga: mantené apretado **BOOT**, tocá **RST**, soltá BOOT y volvé a subir.

Con las dos placas conectadas a la vez hay que decirle a cuál va cada una. Fijate los puertos con `pio device list` (en Windows son `COM5`, `COM6`, etc.):

```bash
pio run -e cruce -t upload --upload-port COM5
pio run -e tren  -t upload --upload-port COM6
```

Conviene marcar cada placa con un papelito "NC" y "NT" para no confundirlas.

### Paso 3. Ver qué hace cada una

Abrí el **Serial Monitor** a 115200 baud de cada placa (o `pio device monitor -p COM5 -b 115200`). Escribí `h` y Enter para ver los comandos, `i` para ver la información del nodo.

Al arrancar, cada placa imprime `PANDA nodo CRUCE  fw 0.4.1-panda-principal` (o `TREN`) y la pantalla muestra **PANDA CRUCE** o **PANDA TREN**. Si una placa muestra el rol equivocado, se cargó con el entorno equivocado: volvé al paso 2.

Después seguí con la prueba de banco del enlace (sección 4) y, para la barrera con una sola placa, con el simulador (sección 4 quater).

### Si algo falla, qué pasar

- **No compila:** el mensaje de error completo del paso 1.
- **Carga pero no arranca o se reinicia:** lo que imprime el Serial Monitor desde el arranque (incluido lo que diga `Guru Meditation` o `rst:` si aparece).
- **Arranca pero no se ven entre sí:** la salida del comando `i` en las dos placas y una línea de estado de cada una.

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

## 4 bis. Nodo cruce: lógica de decisión

PANDA es el **sistema principal** del cruce y el circuito de vía queda de **respaldo**. Línea piloto: **Roca** (CSR, 25 kV). Tiempos de ADIF: Anexo XII del pliego de barreras automáticas del Roca, que toma la Tabla I del SETOP 7/81.

### Estados

| Estado | Señal peatonal | Contacto PANDA libre | Contacto PANDA operativo | Sonido |
|---|---|---|---|---|
| **APAGADO** | apagada (no afirma nada, nunca hay verde) | cerrado | cerrado | no |
| **NO SEGURO** | intermitente cada 0,5 s durante t_p (17,1 s) y después fija (Anexo XII 4.2) | abierto = pedido de cierre | cerrado | 1 toque por segundo (2 con OTRO TREN) |
| **FALLA** | encendida fija | abierto | abierto = ignorar a PANDA | no |
| **INICIANDO** (3 s) | encendida (prueba de lámpara) | abierto | abierto | no |

### Cuándo es NO SEGURO

Basta con una sola de estas condiciones. El cruce se apaga recién cuando no se cumple ninguna:

1. **Tren aproxima**: su **ETA de peor caso** es de 32 s o menos (E-8b, `kAlertEtaS`).
2. **Tren en zona**: está a menos de 240 m (formación del Roca de hasta 206 m más margen). El nodo va en la **cabina delantera**, así que después del paso el resto del tren sigue sobre el cruce.
3. **SIN DATOS**: un tren que se acercaba o estaba en zona dejó de mandar datos válidos por más de 1 s (E-9), o un tren lejos que en el peor caso pudo haber entrado en la zona de alerta durante el silencio.
4. **Tren sin posición, no verificable o con dato inconsistente** (plausibilidad).
5. **Vía ocupada sin nodo**: el circuito de vía se ocupó y PANDA no sigue al tren que lo ocupó. Cierra como siempre.

**Liberación (sector de aproximación).** Una vez pedido el cierre, un tren que se acerca lo sigue sosteniendo hasta que su ETA de peor caso supera **40 s** (`kReleaseEtaS` = 32 + 3 s de subida + 5 s de espera). Es el equivalente en tiempo del sector de aproximación del Anexo XII (puntos 19 y 20): la barrera no sube si hay otro tren en aproximación, y entre que el brazo llega arriba y el próximo ciclo pasan al menos 5 s. Un tren detenido no está en aproximación: no la sostiene, y si arranca el umbral de 32 s le da el ciclo completo (SETOP 8.6.13).

Pasa a **FALLA** si el nodo no puede cumplir su función: la tarea de radio no responde, no tiene tiempo GPS propio o no tiene la posición del cruce.

> **Desvío declarado (Anexo XII 4.2).** El Anexo pide verde sin trenes, rojo con sonido simultáneo al aproximarse, y usa la señal apagada como estado de falla (cartel "APAGADO: PARE, MIRE Y ESCUCHE"). PANDA no tiene verde (nunca afirma que se puede cruzar) y en FALLA deja el rojo fijo sin sonido: lo que PANDA no puede asegurar es NO SEGURO, y como no hay verde, apagado se vería igual que "sin trenes". El sonido no acompaña a la falla para no enseñar a ignorarlo.

### Umbral: el ciclo de barrera de ADIF

| Parte | Tiempo | Fuente |
|---|---|---|
| Fonoluminosa (luces y campana antes de bajar) | 7 s | Anexo XII, punto 20 |
| Bajada del brazo (peor caso del pliego, 5 a 10 s) | 10 s | Anexo XII, punto 20 |
| Despejamiento (brazo abajo hasta que llega el tren), vía doble | 14 s | Anexo XII y Tabla I del SETOP (12, 14 o 16 s según la separación entre rieles extremos) |
| Margen propio de PANDA | 1 s | período de decisión y reacción |
| **Total** | **32 s** | |

En un paso **solo peatonal** (`kHasBarrier = false`) el umbral es t_sem + 1 s, con t_sem = d_p / 0,7 m/s + 3 s (Anexo XII, punto 4.2): 21,1 s con 12 m de cruce.

Con barrera, el rojo peatonal se enciende con el pedido de cierre, al menos 31 s antes del tren. Eso cumple el t_sem del Anexo mientras d_p ≤ (31 − 3) × 0,7 = **19,6 m** entre líneas de detención peatonal.

### ETA de peor caso

Es el menor tiempo en que el tren *podría* llegar si acelerara al máximo que permite su material rodante: 1,0 m/s² hasta 40 km/h, después potencia constante, tope 120 km/h (`kLineMaxSpeedMps`). La distancia es en línea recta: en una vía curva sale más corta que la real y la alerta sale antes, así que el error va del lado seguro.

El circuito de vía actual se dimensiona para 120 km/h: 31 s × 33,3 m/s = **1033 m**. PANDA, en cambio, cierra según la velocidad real:

| Velocidad | Circuito actual: barrera baja antes del tren | PANDA cierra a | Ganancia por paso |
|---|---|---|---|
| detenido | (ocupado si está dentro del circuito) | 448 m | toda la detención si está a más de 448 m |
| 20 km/h | 186 s | 574 m | 83 s |
| 40 km/h | 93 s | 682 m | 32 s |
| 60 km/h | 62 s | 794 m | 14 s |
| 80 km/h | 47 s | 922 m | 5 s |
| 100 km/h | 37 s | 1025 m | 0,3 s |

A 120 km/h PANDA y el circuito cierran igual. Si el tramo tiene un límite menor que hace cumplir el ATS, poniendo ese límite en `kLineMaxSpeedMps` la ganancia crece (con 90 km/h: 11 s a 80 km/h).

### Atribución del circuito de vía

Cuando el circuito se ocupa, PANDA mira si en ese instante sigue un tren con dato fresco y coherente, que se acerca, a 1033 ± 100 m y del lado del circuito. Si lo encuentra, la ocupación es de ese tren y la barrera la maneja PANDA según su ETA (ahí está la ganancia). Si no, es un tren sin nodo (o con el nodo caído) y **cierra como siempre**. Además funciona como verificación cruzada de la posición que manda el tren.

Cuando el circuito se libera con ese tren ya alejándose, la cola pasó la junta de salida: PANDA deja de sostener el cierre en ese mismo instante, igual que hoy.

### Plausibilidad (acción del DFMEA contra la posición errónea)

Cada dato nuevo se compara con el **último dato confiable** del tren. La distancia recorrida tiene que estar entre lo que el tren recorre frenando de emergencia (1,2 m/s²) y lo que recorre acelerando al máximo, con una tolerancia de 10 m más 3 veces la precisión informada. También se rechazan saltos de velocidad o aceleraciones imposibles. Mientras el dato no vuelva a esa envolvente, el cruce queda en NO SEGURO (`dato inconsistente`), y 2 s más después.

### Liberación de un tren en SIN DATOS

- Vuelve a transmitir y los datos muestran que ya no hay peligro.
- El circuito de vía se ocupa y se libera durante el silencio: el tren pasó.
- A los 120 s de silencio, como último recurso. Se registra como `SilentRelease`.

Un tren que se alejaba y se deja de escuchar se olvida a los 10 s.

### Posición del cruce

Por orden de prioridad:

1. **Simulador** (`R`, solo en RAM).
2. **Guardada** con el comando `c`: promedio de los últimos 60 s de GNSS propio, con al menos 10 s de fix bueno. `C` la borra.
3. `kFixedRefLatE7` y `kFixedRefLonE7` en `config.h`.
4. Promedio en vivo del GNSS propio, a partir de 5 s de fix.

Para un ensayo, dejá el nodo quieto en el lugar unos minutos y guardá con `c`.

### Vigilancia

- **Vigilante de salidas**: un timer en el core 0 verifica que la decisión (core 1) refresque las salidas cada 50 ms. A los 300 ms sin refresco abre los dos contactos y la barrera vuelve a seguir solo al circuito de vía.
- **Watchdog de tareas** del ESP-IDF: si la decisión no vuelve en 5 s, reinicia el micro. Durante el reinicio los pull-down externos dejan todo en estado seguro.
- **Watchdog externo TPL5010**: TODO, comentado en `config.h` (pin DONE en el GPIO 46).

### Alarmas para el monitoreo remoto (Anexo XII, punto 22)

- **22 g) redefinida.** La original, "brazo de barrera levantado con circuito de vía ocupado", con PANDA principal saltaría en cada tren lento atribuido (entre los 1033 m y el punto de cierre de PANDA). Se reemplaza por "**el brazo no sigue la regla del controlador**": hay pedido de cierre (de PANDA, o del circuito si PANDA no está operativo) y el brazo sigue arriba o subiendo más de 1 s (`kAlarmBarrierMs`). Necesita la posición del brazo, que el punto 22 e) ya exige; en el prototipo la da la maqueta. Evento `AlarmBarrier` (22).
- **22 f)** circuito ocupado más de 10 minutos, igual que hoy. Evento `AlarmTrackLong` (23).

En el producto la telealarma existente aplica la misma regla con los dos contactos de PANDA. Las dos salen en la consola (`i` y línea de estado).

## 4 ter. Cableado del nodo cruce y maqueta de barrera

Con `kBarrierOnBoard = true` (por defecto) la **misma T-Beam maneja la maqueta**. El controlador de la maqueta es un módulo aparte del firmware (`crossing_io.cpp`) que solo ve los dos contactos de PANDA y el circuito de vía, igual que el controlador real:

- PANDA operativo: la barrera baja si PANDA pide cierre.
- PANDA no operativo: la barrera baja si el circuito está ocupado, que es el comportamiento actual.

Secuencia: **fonoluminosa** 7 s (luces alternadas cada 0,5 s y campana, Anexo XII punto 20), **bajada** (6 s en la maqueta, dentro de los 5 a 10 s del pliego), **abajo** hasta que se levanta el pedido, con la campana a nivel reducido (Anexo XII 5.5), y **subida** (3 s) con las señales todavía encendidas hasta que el brazo llega a la vertical (SETOP 8.6.6). Si el pedido vuelve mientras sube, baja de inmediato: las luces vienen encendidas sin corte desde la fonoluminosa, así que el preaviso de 5 s se cumple.

> El Anexo XII (punto 20) corta las señales al iniciar el ascenso, pero el mismo Anexo (4.1) obliga a cumplir el SETOP 7/81, que es obligatorio y no admite acuerdos que lo violen (SETOP 1.2 y 1.4). Por eso manda el 8.6.6.

Limitaciones de la maqueta: hay un solo buzzer para la campana y el aviso de PANDA (comparten tono, y con el brazo horizontal baja todo el buzzer); con un buzzer activo no hay nivel reducido; no hay detección de rotura del brazo (Anexo XII 5.5 pide volver a 95 dB si se rompe).

| GPIO | Con maqueta en la placa | Con controlador externo | Estado seguro |
|---|---|---|---|
| 21 | Servo del brazo (señal, el servo se alimenta aparte con 5 V) | Relé **PANDA libre** | bajo |
| 38 | Luz roja A de la barrera | Relé **PANDA operativo** | bajo |
| 3 | Luz roja B de la barrera (alterna con la A) | sin uso | bajo |
| 39 | **Señal peatonal** NO SEGURO (LED rojo) | igual | la maneja la lógica |
| 45 | **OTRO TREN**, intermitente cada 0,5 s (LED con resistencia a GND, nada de pull-up: es pin de arranque) | igual | bajo |
| 48 | **Sonido**: buzzer pasivo (PWM a 1568 Hz, sol5, nivel reducido con el brazo abajo) o activo (`kBuzzerIsActive`) con transistor | igual | apagado |
| 2 | Entrada **circuito de vía**: interruptor a GND (cerrado = LIBRE), después optoacoplador | igual | abierto = OCUPADA |

- Cada salida lleva **un pull-down de 10 kΩ a GND**. Un LED se puede manejar directo con 330 Ω. El buzzer y la sirena, con un transistor NPN (BC547 o 2N2222) y 1 kΩ en la base.
- La entrada del circuito viene desactivada (`kTrackCircuitEnabled = false`) y se simula con `v`. Activala recién con el interruptor o el opto cableado: con la entrada al aire se lee OCUPADA.
- Limitación de la maqueta: sin energía el servo no baja solo. La barrera real cae por gravedad (Anexo XII, punto 5.2).

## 4 quater. Probar el cruce con una sola placa: simulador de trenes

`tools/sim_trenes.py` genera trenes sobre una vía recta, arma beacons **reales** (cifrados y autenticados) y se los manda al nodo cruce por el USB. El nodo los procesa por el mismo camino que los de radio: CRC, CMAC, contador, frescura, plausibilidad y decisión. Si el nodo no tiene fix, el simulador también le da el tiempo GPS desde el reloj de la PC. Además **hace de circuito de vía**: le pasa su geometría (comando `Q`) y manda `v` cada vez que un tren entra o sale del circuito (1033 m del lado de aproximación hasta 5 m pasado el cruce, tenga nodo o no).

1. Cargá `cruce` en la placa. No hace falta antena ni fix.
2. **Cerrá el monitor serie de PlatformIO**: el puerto no se puede compartir.
3. Corré, con el Python de PlatformIO, que ya trae pyserial:

```bash
~/.platformio/penv/Scripts/python.exe tools/sim_trenes.py --puerto COM5 --escenario lento
```

El simulador muestra todo lo que imprime el nodo. Los cambios de estado salen en amarillo.

> Si la placa tiene fix propio usa su tiempo GPS e ignora el de la PC. En ese caso el reloj de la PC tiene que estar sincronizado (en Windows: Configuración > Hora e idioma > Sincronizar ahora). Con más de 2 s de diferencia, los beacons simulados se rechazan como `VIEJO` o `FUTURO`, y eso es justamente la protección funcionando.

Para ver qué *debería* pasar sin la placa, está el modelo de referencia en Python (`tools/modelo_cruce.py`, con las mismas reglas que `crossing.cpp`). Al final muestra cuánto tiempo pidió PANDA el cierre contra el tiempo que el circuito actual tendría la barrera baja:

```bash
python tools/sim_trenes.py --seco --escenario lento
```

| Escenario | Qué prueba | Resultado esperado (modelo) |
|---|---|---|
| `rapido` | Tren a 100 km/h desde 1500 m | Circuito explicado a 1039 m, NO SEGURO a 1022 m, apaga cuando el circuito se libera. 44,3 s contra 44,8 s |
| `lento` | Tren a 40 km/h desde 1500 m | Circuito explicado a 1034 m, NO SEGURO recién a 681 m. 80,2 s contra 111,9 s (**+31,7 s**) |
| `detenido` | Frena y para a unos 500 m (dentro del circuito), espera 20 s, arranca | NO SEGURO a 793 m mientras frena, APAGADO recién parado a 500 m (mientras frena todavía se acerca y lo sostiene el umbral de 40 s), NO SEGURO al arrancar. **+35,1 s** |
| `perdida` | Sin enlace entre 900 y 500 m | SIN DATOS 1 s después del corte (E-9), vuelve con datos |
| `silencio` | El nodo del tren muere a 700 m | SIN DATOS hasta que el circuito se libera con la cola. +14,5 s |
| `dos` | Dos trenes en sentidos opuestos | Dos ciclos, OTRO TREN cuando se superponen. -2,3 s (la otra vía no tiene su circuito cableado a la placa: se libera por radio, unos 4 s más tarde) |
| `aproximacion` | Pasa un tren a 100 km/h y otro viene en sentido contrario | El segundo sostiene el cierre desde ETA 40 s: la barrera no sube entre los dos (sin la histéresis subiría y volvería a bajar a los 4 s). El sistema actual tampoco sube, por el sector de aproximación. -3,7 s, por lo mismo que `dos` |
| `repeticion` | Un atacante reinyecta un paquete grabado | Se rechaza como `REPETIDO`, sin efecto en el estado |
| `cmac` | Paquetes falsos sin la clave | Se rechazan como `CMAC`, sin efecto |
| `sin_posicion` | Tren que se escucha sin fix | NO SEGURO (sin posición) hasta que da posición lejos |
| `via` | Tren **sin nodo** a 80 km/h | Circuito sin explicar: NO SEGURO como siempre, mismo tiempo que hoy |
| `salto` | El GNSS suma 300 m de error entre 800 y 700 m | `dato inconsistente` sin ningún hueco APAGADO, después sigue normal |

La maqueta y las intermitencias tienen además una prueba en la PC, sin placa ni PlatformIO, con un reloj simulado: `sh test_pc/correr.sh`. Compila los archivos del cruce para los tres roles y verifica la secuencia de la barrera (luces también en la subida, al menos 5 s de luces antes de cada bajada, campana reducida con el brazo abajo) y las intermitencias del Anexo XII.

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

Comandos del simulador (líneas, solo banco): `T <itow_ms>` tiempo GPS, `R <latE7> <lonE7>` posición del cruce, `Q <metros> <rumbo>` geometría del circuito de vía, `B <hex> <rssi> <snr>` beacon.

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
CRUCE SD+ WFx USB
  NO SEGURO             estado en grande (invertido si es NO SEGURO)
2T tren aproxima A001   motivo y tren ("2T" = OTRO TREN)
681m 40km/h e32s        distancia al cruce, velocidad, ETA de peor caso
ABAJO via OCUe* P+      maqueta (ARRIBA, FONO, BAJANDO, ABAJO, SUBIENDO), circuito de vía
                        (e = explicado por un tren, ! = sin nodo, * = simulado), PANDA operativo
```

## 7. Archivos en la microSD

Cada encendido crea `PANDA_NNNN`. Todos los archivos comparten `t_us` (µs desde el arranque de esa placa).

- `gnss.csv`, `imu.csv`, `events.csv`: iguales que en la fase 1 (el cruce no graba IMU). `gnss.csv` suma `acel_mps2`: aceleración longitudinal del GNSS (recta sobre 1 s de velocidades).
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
| `acel_mps2` | Aceleración que mandó el tren (vacío si no la tiene) |

- `decision.csv` (cruce), un renglón por beacon válido de cada tren y en cada cambio de estado:

| Columna | Descripción |
|---|---|
| `estado`, `motivo` | Estado del cruce y motivo principal |
| `tren`, `fase`, `alerta` | Tren evaluado, su fase (`LEJOS`, `APROXIMA`, `EN_ZONA`, `SE_ALEJA`, `SIN_DATOS`) y si sostiene el NO SEGURO |
| `dist_m`, `vel_mps`, `acerc_mps` | Distancia al cruce proyectada al instante actual, velocidad y velocidad de acercamiento |
| `eta_cv_s`, `eta_min_s` | ETA a velocidad constante y ETA mínimo |
| `edad_ms` | Antigüedad del dato usado |
| `panda_libre`, `panda_ok`, `senal`, `via_ocupada` | Contactos, señal peatonal y circuito de vía |
| `pedido_cierre` | Lo que se le pide al controlador de barrera |
| `via_explicada`, `otro_tren` | La ocupación del circuito la explica un tren seguido / hay más de un tren en peligro |

Para validar E-8a: el PASO real queda en `events.csv` (automático, por mínimo de distancia, y con el botón), y se compara con `t_us + eta_cv_s` de los renglones anteriores.

## 8. Beacon y seguridad

35 bytes, versión 2: cabecera en claro (versión, id, contador), 20 bytes cifrados con **AES-128-CTR** y etiqueta **AES-CMAC** de 8 bytes. El formato exacto está en `include/beacon.h`. Dentro va la posición, la velocidad, el rumbo, la precisión (1 byte en dm) y la **aceleración** (1 byte en pasos de 0,02 m/s²). Se le sacó un byte a la precisión para no agrandar el beacon: con 36 bytes el aire pasaría de 18,0 a 19,3 ms y se perdería una de las 5 ranuras.

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
