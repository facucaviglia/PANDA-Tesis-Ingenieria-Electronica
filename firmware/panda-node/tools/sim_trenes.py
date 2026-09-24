"""
PANDA · Simulador de trenes para probar el nodo cruce con una sola placa.

Genera la trayectoria de uno o dos trenes sobre una vía recta que pasa por el
cruce, arma beacons REALES (cifrados y autenticados con el mismo código de
referencia que tools/verify_crypto.py) y se los manda al nodo cruce por el USB.
El nodo los procesa por el mismo camino que los que llegan por radio: CRC,
CMAC, contador, frescura y lógica de decisión.

También le da al nodo:
  - el tiempo GPS, calculado desde el reloj de la PC (si el nodo no tiene fix)
  - una posición de referencia del cruce solo en RAM (no pisa la guardada)

Uso (con el Python de PlatformIO, que ya trae pyserial):
    ~/.platformio/penv/Scripts/python.exe tools/sim_trenes.py --puerto COM5 --escenario rapido
    ... --lista                  muestra los escenarios
    ... --ref -34.6,-58.4        cambia la posición del cruce
    ... --rumbo 60               rumbo de la vía en grados

Mientras corre muestra todo lo que imprime el nodo. Cerrar el monitor serie de
PlatformIO antes: el puerto no se puede compartir.
"""

import argparse
import math
import os
import random
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_crypto import seal  # noqa: E402

try:
    import serial
except ImportError:  # Solo hace falta para hablar con la placa, no para --seco
    serial = None

R_EARTH = 6371008.8
GPS_EPOCH_UNIX = 315964800
LEAP_SECONDS = 18
WEEK_MS = 604800 * 1000
GNSS_LATENCY_MS = 40          # Latencia típica del NAV-PVT respecto de su época
F_FIX, F_TIME, F_PPS = 1, 2, 4


def gps_tow_ms(unix_s=None):
    """Tiempo GPS de la semana en ms a partir del reloj de la PC."""
    if unix_s is None:
        unix_s = time.time()
    gps_ms = int((unix_s - GPS_EPOCH_UNIX + LEAP_SECONDS) * 1000)
    return gps_ms % WEEK_MS


# ---------------------------------------------------------------------------
# Tren: posición sobre la vía (s en metros, 0 = cruce) y velocidad
# ---------------------------------------------------------------------------
class Tren:
    def __init__(self, node_id, s0, v_kmh, sentido=+1):
        self.node_id = node_id
        self.s = float(s0)
        self.v = v_kmh / 3.6
        self.sentido = sentido           # +1 avanza hacia s creciente, -1 al revés
        self.a = 0.0                     # aceleración ordenada por el escenario
        self.v_obj = None                # velocidad objetivo (para frenar o acelerar)
        self.transmite = True
        self.con_fix = True
        # Contador creciente entre corridas: el cruce recuerda el último y
        # descarta como repetición cualquier valor menor o igual.
        self.counter = int(time.time() * 10) % (2 ** 31) + random.randint(0, 1000)
        self.ultimo_paquete = None

    def paso(self, dt):
        if self.v_obj is not None:
            dv = self.v_obj - self.v
            paso = math.copysign(min(abs(dv), abs(self.a) * dt), dv)
            self.v = max(0.0, self.v + paso)
            if abs(self.v_obj - self.v) < 1e-3:
                self.v_obj = None
        self.s += self.sentido * self.v * dt

    def frenar_a(self, v_kmh, decel):
        self.v_obj, self.a = v_kmh / 3.6, decel

    def acelerar_a(self, v_kmh, acel):
        self.v_obj, self.a = v_kmh / 3.6, acel


class Vía:
    def __init__(self, ref_lat, ref_lon, rumbo_deg):
        self.lat0, self.lon0 = ref_lat, ref_lon
        self.rumbo = math.radians(rumbo_deg)

    def posicion(self, s):
        dn, de = s * math.cos(self.rumbo), s * math.sin(self.rumbo)
        lat = self.lat0 + math.degrees(dn / R_EARTH)
        lon = self.lon0 + math.degrees(de / (R_EARTH * math.cos(math.radians(self.lat0))))
        return int(round(lat * 1e7)), int(round(lon * 1e7))

    def rumbo_tren(self, sentido):
        deg = math.degrees(self.rumbo) + (0 if sentido > 0 else 180)
        return int(round((deg % 360) * 100))


# ---------------------------------------------------------------------------
# Escenarios: cada uno es una función que recibe (t, trenes, acciones) y
# decide qué pasa en ese instante. Devuelve False cuando termina.
# ---------------------------------------------------------------------------
def esc_rapido(t, tr, acc):
    """Un tren a 100 km/h desde 1500 m, pasa el cruce y se aleja."""
    if t == 0:
        tr.append(Tren(0xA001, -1500, 100))
    return tr[0].s < 600


def esc_lento(t, tr, acc):
    """Un tren a 40 km/h desde 600 m."""
    if t == 0:
        tr.append(Tren(0xA001, -600, 40))
    return tr[0].s < 350


def esc_detenido(t, tr, acc):
    """Viene a 60 km/h, frena y se detiene a 500 m, espera 20 s y arranca."""
    if t == 0:
        tr.append(Tren(0xA001, -1500, 60))
    x = tr[0]
    if x.s > -620 and x.v > 0 and not hasattr(x, "frenó"):
        x.frenó = True
        x.frenar_a(0, 0.7)
    if hasattr(x, "frenó") and x.v == 0 and not hasattr(x, "parado_desde"):
        x.parado_desde = t
    if hasattr(x, "parado_desde") and t - x.parado_desde > 20 and not hasattr(x, "arrancó"):
        x.arrancó = True
        x.acelerar_a(80, 0.8)
    return x.s < 400


def esc_perdida(t, tr, acc):
    """Como rápido, pero sin enlace entre 900 y 500 m: SIN DATOS y vuelve."""
    if t == 0:
        tr.append(Tren(0xA001, -1500, 100))
    x = tr[0]
    x.transmite = not (-900 < x.s < -500)
    return x.s < 600


def esc_silencio(t, tr, acc):
    """El nodo del tren muere a 700 m. Después pasa por el circuito de vía
    (simulado con 'v') y el cruce lo libera por esa vía."""
    if t == 0:
        tr.append(Tren(0xA001, -1200, 60))
    x = tr[0]
    if x.s > -700:
        x.transmite = False
    if x.s > -5 and not hasattr(x, "via_ocupada"):
        x.via_ocupada = t
        acc.append("v")          # circuito de vía: OCUPADA
    if hasattr(x, "via_ocupada") and t - x.via_ocupada > 12 and not hasattr(x, "via_libre"):
        x.via_libre = True
        acc.append("v")          # circuito de vía: LIBRE
    return not hasattr(x, "via_libre") or t - x.via_ocupada < 20


def esc_dos(t, tr, acc):
    """Dos trenes en sentidos opuestos, desfasados 15 s."""
    if t == 0:
        tr.append(Tren(0xA001, -1500, 90, +1))
    if abs(t - 15) < 1e-6:
        tr.append(Tren(0xA002, 1800, 70, -1))
    return len(tr) < 2 or tr[0].s < 600 or tr[1].s > -600


def esc_repeticion(t, tr, acc):
    """Tren normal más un atacante que reinyecta un paquete grabado cada 2 s."""
    if t == 0:
        tr.append(Tren(0xA001, -1500, 100))
    if abs(t - 5) < 1e-6:
        tr[0].grabado = tr[0].ultimo_paquete
    if t > 5 and int(t * 10) % 20 == 0 and getattr(tr[0], "grabado", None):
        acc.append(("raw", tr[0].grabado))
    return tr[0].s < 600


def esc_cmac(t, tr, acc):
    """Tren normal más paquetes falsos sin la clave (etiqueta inválida)."""
    if t == 0:
        tr.append(Tren(0xA001, -1500, 100))
    if int(t * 10) % 5 == 0 and tr[0].ultimo_paquete:
        falso = bytearray(tr[0].ultimo_paquete)
        falso[12:20] = os.urandom(8)
        acc.append(("raw", bytes(falso)))
    return tr[0].s < 600


def esc_sin_posicion(t, tr, acc):
    """Un tren se escucha sin fix durante 15 s y después aparece alejándose."""
    if t == 0:
        x = Tren(0xA001, 1500, 80, +1)
        x.con_fix = False
        tr.append(x)
    if t > 15:
        tr[0].con_fix = True
    return t < 30


def esc_via(t, tr, acc):
    """Sin trenes con nodo: el circuito de vía detecta uno que PANDA no ve."""
    if abs(t - 5) < 1e-6 or abs(t - 20) < 1e-6:
        acc.append("v")
    return t < 30


ESCENARIOS = {
    "rapido": esc_rapido,
    "lento": esc_lento,
    "detenido": esc_detenido,
    "perdida": esc_perdida,
    "silencio": esc_silencio,
    "dos": esc_dos,
    "repeticion": esc_repeticion,
    "cmac": esc_cmac,
    "sin_posicion": esc_sin_posicion,
    "via": esc_via,
}


# ---------------------------------------------------------------------------
# Enlace con el nodo
# ---------------------------------------------------------------------------
def lector(ser, parar):
    """Muestra todo lo que imprime el nodo. Resalta los cambios de estado."""
    buf = b""
    while not parar.is_set():
        try:
            data = ser.read(512)
        except serial.SerialException:
            break
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            linea, buf = buf.split(b"\n", 1)
            texto = linea.decode("utf-8", "replace").rstrip()
            if ">>>" in texto:
                print(f"\033[1;33m{texto}\033[0m")
            elif texto.startswith("[CRUCE]"):
                print(f"\033[36m{texto}\033[0m")
            else:
                print(texto)


def rssi_simulado(d):
    """Pérdida de dos rayos aproximada a 2 dBm, más un poco de ruido."""
    d = max(d, 10.0)
    return 2 - (40 * math.log10(d) - 20 * math.log10(2.5 * 3.0)) + random.gauss(0, 2), 10 - d / 300


def correr_seco(args):
    """Ejecuta el escenario sin puerto serie y sin tiempo real. Verifica que
    termine, cuenta lo que mandaría y comprueba que cada beacon abre bien."""
    from verify_crypto import open_beacon
    from modelo_cruce import ModeloCruce
    modelo = ModeloCruce()
    via_sim = False
    lat, lon = (float(x) for x in args.ref.split(","))
    via = Vía(lat, lon, args.rumbo)
    trenes, t, dt = [], 0.0, 0.1
    beacons, otras, crudos, min_d = 0, [], 0, {}
    escenario = ESCENARIOS[args.escenario]
    while t < 600:
        acciones = []
        sigue = escenario(round(t, 1), trenes, acciones)
        for x in trenes:
            if x.transmite:
                lat_e7, lon_e7 = via.posicion(x.s)
                x.counter += 1
                pkt = seal(x.node_id, x.counter, F_TIME | F_PPS | (F_FIX if x.con_fix else 0), 1000, lat_e7, lon_e7,
                           int(round(x.v * 100)), via.rumbo_tren(x.sentido), 150, 14)
                assert open_beacon(pkt) is not None
                x.ultimo_paquete = pkt
                beacons += 1
                modelo.beacon(t, x.node_id, x.s, x.v, x.sentido, x.con_fix)
            min_d[x.node_id] = min(min_d.get(x.node_id, 1e9), abs(x.s))
            x.paso(dt)
        for a in acciones:
            if isinstance(a, tuple):
                crudos += 1
            else:
                otras.append((round(t, 1), a))
                if a == "v":
                    via_sim = not via_sim
                    modelo.set_via(via_sim)
        # La decisión corre a 20 Hz: dos vueltas por cada paso de 100 ms.
        modelo.tick(t)
        modelo.tick(t + dt / 2)
        if not sigue:
            break
        t += dt
    for _ in range(200):  # 10 s más, para ver cómo se apaga
        t += dt / 2
        modelo.tick(t)

    print(f"{args.escenario}: {beacons} beacons válidos, {crudos} paquetes crudos, comandos {otras}, "
          f"distancia mínima por tren {{{', '.join(f'{k:04X}: {v:.0f} m' for k, v in min_d.items())}}}")
    print("  Estados esperados en el cruce (modelo de referencia):")
    eventos = []
    for tt, est, motivo, tren, d, eta in modelo.transiciones:
        if tren and not math.isnan(d):
            extra = f"  tren {tren:04X} a {d:.0f} m, ETA mín {eta:.1f} s"
        elif tren:
            extra = f"  tren {tren:04X}"
        else:
            extra = ""
        eventos.append((tt, f"{est:9s}  {motivo}{extra}"))
    for tt, tren, dmin in modelo.pasos:
        eventos.append((tt, f"PASO del tren {tren:04X}, distancia mínima {dmin} m"))
    for tt, texto in sorted(eventos, key=lambda e: e[0]):
        print(f"    t={tt:6.1f} s  {texto}")
    if modelo.via_sin_panda:
        print(f"    vía ocupada con PANDA en vía libre: {modelo.via_sin_panda}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", help="Puerto serie del nodo cruce (COM5, /dev/ttyACM0, ...)")
    ap.add_argument("--escenario", default="rapido", choices=sorted(ESCENARIOS))
    ap.add_argument("--ref", default="-34.6000,-58.4000", help="lat,lon del cruce")
    ap.add_argument("--rumbo", type=float, default=60.0, help="rumbo de la vía en grados")
    ap.add_argument("--lista", action="store_true", help="lista los escenarios y sale")
    ap.add_argument("--seco", action="store_true",
                    help="corre el escenario sin placa y sin esperar, para verificar el script")
    args = ap.parse_args()

    if args.seco:
        return correr_seco(args)

    if args.lista or not args.puerto:
        for nombre, f in sorted(ESCENARIOS.items()):
            print(f"  {nombre:13s} {f.__doc__.strip()}")
        if not args.puerto:
            print("\nFalta --puerto")
        return

    if serial is None:
        print("Falta pyserial. Usá el Python de PlatformIO (~/.platformio/penv/Scripts/python.exe)")
        print("o instalalo con: pip install pyserial")
        return 1

    lat, lon = (float(x) for x in args.ref.split(","))
    via = Vía(lat, lon, args.rumbo)

    ser = serial.Serial()
    ser.port = args.puerto
    ser.baudrate = 115200
    ser.timeout = 0.05
    # Sin tocar DTR/RTS: en el USB del ESP32-S3 esa secuencia reinicia la placa.
    ser.dtr = False
    ser.rts = False
    ser.open()

    parar = threading.Event()
    threading.Thread(target=lector, args=(ser, parar), daemon=True).start()

    def enviar(linea):
        ser.write((linea + "\n").encode())

    print(f"== Escenario {args.escenario}: {ESCENARIOS[args.escenario].__doc__.strip()}")
    print(f"== Cruce en {lat:.6f}, {lon:.6f}, vía a {args.rumbo:.0f} grados. Ctrl+C para cortar.\n")
    enviar(f"R {int(round(lat * 1e7))} {int(round(lon * 1e7))}")
    enviar(f"T {gps_tow_ms()}")
    time.sleep(0.5)

    trenes, t, dt = [], 0.0, 0.1
    escenario = ESCENARIOS[args.escenario]
    proximo = time.monotonic()
    ultimo_t = -1.0
    try:
        while True:
            acciones = []
            sigue = escenario(round(t, 1), trenes, acciones)
            tow = gps_tow_ms()
            if int(t * 10) % 10 == 0:
                enviar(f"T {tow}")

            for x in trenes:
                if x.transmite:
                    lat_e7, lon_e7 = via.posicion(x.s)
                    # iTOW de la época GNSS más reciente, con la latencia del receptor.
                    itow = ((tow - GNSS_LATENCY_MS) // 100) * 100
                    flags = F_TIME | F_PPS | (F_FIX if x.con_fix else 0)
                    x.counter += 1
                    pkt = seal(x.node_id, x.counter, flags, itow, lat_e7, lon_e7, int(round(x.v * 100)),
                               via.rumbo_tren(x.sentido), 150, 14)
                    x.ultimo_paquete = pkt
                    rssi, snr = rssi_simulado(abs(x.s))
                    enviar(f"B {pkt.hex()} {rssi:.1f} {snr:.1f}")
                x.paso(dt)

            for a in acciones:
                if isinstance(a, tuple) and a[0] == "raw":
                    enviar(f"B {a[1].hex()} -90.0 5.0")
                else:
                    enviar(a)

            if t - ultimo_t >= 2.0 and trenes:
                ultimo_t = t
                estado = "  ".join(
                    f"{x.node_id:04X} s={x.s:7.0f} m v={x.v * 3.6:5.1f} km/h{'' if x.transmite else ' (mudo)'}"
                    for x in trenes)
                print(f"\033[90m[SIM t={t:5.1f}s] {estado}\033[0m")

            if not sigue:
                print("\n== Fin del escenario. Se sigue escuchando 5 s.")
                time.sleep(5)
                break
            t += dt
            proximo += dt
            time.sleep(max(0.0, proximo - time.monotonic()))
    except KeyboardInterrupt:
        print("\n== Cortado por el usuario")
    finally:
        parar.set()
        time.sleep(0.2)
        ser.close()


if __name__ == "__main__":
    main()
