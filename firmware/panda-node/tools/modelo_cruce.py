"""
PANDA · Modelo de referencia de la lógica del cruce, en Python.

Reproduce las reglas de src/crossing.cpp con los mismos parámetros de
include/config.h (cfg::crossing). Si se cambia uno, hay que cambiarlo en los
dos lados. Lo usa tools/sim_trenes.py --seco para mostrar, sin placa, qué
secuencia de estados tendría que dar el nodo cruce en cada escenario, y cuánto
tiempo estaría baja la barrera con PANDA contra el sistema actual.

Simplificación: la vía es recta y pasa por el cruce, así que la distancia es
|s| y la velocidad de acercamiento es +v o -v según el sentido. El circuito de
vía de la placa es uno solo, del lado s < 0.
"""

import math

# --- Parámetros (espejo de cfg::crossing y cfg::beacon) ----------------------
FONOLUMINOSA_S = 7.0          # Anexo XII de ADIF, punto 20
ARM_DOWN_S = 10.0
CLEARANCE_S = 14.0            # vía doble, separación entre rieles extremos 5 a 10 m
BARRIER_CYCLE_S = FONOLUMINOSA_S + ARM_DOWN_S + CLEARANCE_S
LATENCY_MARGIN_S = 1.0
ALERT_ETA_S = BARRIER_CYCLE_S + LATENCY_MARGIN_S       # 32 s
MAX_ACCEL = 1.0               # perfil de peor caso del CSR del Roca
ACCEL_KNEE = 11.1
LINE_MAX = 33.3
MAX_TRAIN_LENGTH_M = 210.0
OCCUPIED_RADIUS_M = MAX_TRAIN_LENGTH_M + 30.0
DESIGN_SPEED = 120.0 / 3.6
TRACK_CIRCUIT_DIST_M = BARRIER_CYCLE_S * DESIGN_SPEED   # 1033 m
ATTRIBUTION_TOL_M = 100.0
CIRCUIT_SIDE = -1             # lado del circuito: trenes con s < 0
PLAUS_POS_TOL_M = 10.0
PLAUS_HACC_FACTOR = 3.0
PLAUS_BRAKE = 1.2
PLAUS_ACCEL = 2.2
PLAUS_SPEED_TOL = 0.5
PLAUS_ANCHOR_MAX_AGE_S = 30.0
PLAUS_HOLD_S = 2.0
MIN_CLOSING = 0.5
CLEAR_HOLD_S = 3.0
FORGET_S = 10.0
SILENT_RELEASE_S = 120.0
RELEVANT_RADIUS_M = 5000.0
LINK_TIMEOUT_S = 1.0


def max_distance(v0, t):
    a0, vk = MAX_ACCEL, ACCEL_KNEE
    vmax = max(LINE_MAX, v0)
    x, v, rem = 0.0, v0, t
    if v < vk:
        ta = min(rem, (vk - v) / a0)
        x += v * ta + 0.5 * a0 * ta * ta
        v += a0 * ta
        rem -= ta
        if rem <= 0:
            return x
    if v < vmax:
        k = 2 * a0 * vk
        tb = min(rem, (vmax * vmax - v * v) / k)
        v2 = math.sqrt(v * v + k * tb)
        x += (v2 ** 3 - v ** 3) / (3 * a0 * vk)
        v = v2
        rem -= tb
        if rem <= 0:
            return x
    return x + v * rem


def min_eta(closing, d):
    t_brake = 0.0
    if closing < 0:
        t_brake = -closing / MAX_ACCEL
        d += closing * closing / (2 * MAX_ACCEL)
        closing = 0.0
    if d <= 0:
        return t_brake
    lo, hi = 0.0, 600.0
    if max_distance(closing, hi) < d:
        return t_brake + hi
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if max_distance(closing, mid) >= d:
            hi = mid
        else:
            lo = mid
    return t_brake + hi


def close_distance(v, eta=ALERT_ETA_S):
    """Distancia a la que PANDA ordena el cierre para un tren que se acerca a v."""
    lo, hi = 0.0, 5000.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if min_eta(v, mid) <= eta:
            lo = mid
        else:
            hi = mid
    return lo


class TrenModelo:
    def __init__(self, node_id, t):
        self.id = node_id
        self.last_auth = t
        self.last_valid = None
        self.s = self.v = 0.0
        self.a = None
        self.sentido = 1
        self.sin_pos = False
        self.alerting = False
        self.reason = None
        self.phase = "LEJOS"
        self.clear_since = None
        self.silent_since = None
        self.track_seen = False
        self.dist = float("nan")
        self.closing = 0.0
        self.eta_min = float("nan")
        self.implausible_until = -1.0
        self.ancla = None             # (t, s, v, hacc) del último dato confiable
        self.cola_paso = False
        self.approach = False
        self.passage = False
        self.min_dist = float("inf")


class ModeloCruce:
    def __init__(self):
        self.trenes = {}
        self.via = False
        self.via_explicada = False
        self.via_tren = 0
        self.estado = None
        self.transiciones = []
        self.pasos = []
        self.eventos = []
        self.via_sin_panda = 0
        self.inconsistentes = 0
        self.panda_libre = True
        self.otro_tren = False
        self.pedido_cierre = False
        self.t_prev = None
        self.t_cierre_panda = 0.0     # tiempo con pedido de cierre de PANDA
        self.t_via_ocupada = 0.0      # tiempo con algún circuito ocupado (sistema actual)
        self.via_b = False            # circuito de la otra vía: solo para comparar con el sistema actual

    def beacon(self, t, node_id, s, v, sentido, con_fix, a=None, hacc=1.5):
        x = self.trenes.get(node_id) or self.trenes.setdefault(node_id, TrenModelo(node_id, t))
        x.last_auth = t
        if not con_fix:
            x.sin_pos = True
            return
        causa = 0
        if a is not None and abs(a) > PLAUS_ACCEL:
            causa = 3
        elif x.ancla is not None and 0 < t - x.ancla[0] <= PLAUS_ANCHOR_MAX_AGE_S:
            ta, sa, va, ha = x.ancla
            dt = t - ta
            t_stop = va / PLAUS_BRAKE
            d_min = va * va / (2 * PLAUS_BRAKE) if dt >= t_stop else va * dt - 0.5 * PLAUS_BRAKE * dt * dt
            d_max = max_distance(va, dt)
            tol = PLAUS_POS_TOL_M + PLAUS_HACC_FACTOR * max(ha, hacc)
            movido = abs(s - sa)
            if movido < d_min - tol or movido > d_max + tol:
                causa = 1
            elif abs(v - va) > PLAUS_ACCEL * dt + PLAUS_SPEED_TOL:
                causa = 2
        if causa:
            if t >= x.implausible_until:   # se avisa solo al empezar cada episodio
                self.eventos.append((round(t, 1), f"dato inconsistente del tren {node_id:04X} (causa {causa})"))
            x.implausible_until = t + PLAUS_HOLD_S
            self.inconsistentes += 1
        else:
            x.ancla = (t, s, v, hacc)
        x.last_valid, x.s, x.v, x.a, x.sentido, x.sin_pos = t, s, v, a, sentido, False

    def set_via(self, t, ocupada):
        subio = ocupada and not self.via
        self.via_fell = self.via and not ocupada
        self.via = ocupada
        if subio:
            mejor, err_min = None, float("inf")
            for x in self.trenes.values():
                fresco = x.last_valid is not None and t - x.last_valid <= LINK_TIMEOUT_S
                if not fresco or t < x.implausible_until or math.isnan(x.dist) or x.closing <= MIN_CLOSING:
                    continue
                lado = (x.s < 0) == (CIRCUIT_SIDE < 0)
                err = abs(x.dist - TRACK_CIRCUIT_DIST_M)
                if err <= ATTRIBUTION_TOL_M and lado and err < err_min:
                    mejor, err_min = x, err
            self.via_explicada = mejor is not None
            self.via_tren = mejor.id if mejor else 0
            if mejor:
                self.eventos.append((round(t, 1), f"circuito ocupado por el tren {mejor.id:04X} "
                                                  f"(a {mejor.dist:.0f} m)"))
            else:
                self.eventos.append((round(t, 1), "circuito ocupado SIN tren PANDA: cierra como siempre"))
                if self.panda_libre:
                    self.via_sin_panda += 1
        if self.via_fell and self.via_explicada:
            b = self.trenes.get(self.via_tren)
            if b is not None and b.closing < -MIN_CLOSING:
                b.cola_paso = True
        if not ocupada:
            self.via_explicada, self.via_tren = False, 0

    def set_via_b(self, ocupada):
        self.via_b = ocupada

    def tick(self, t):
        via_fell = getattr(self, "via_fell", False)
        self.via_fell = False
        for x in list(self.trenes.values()):
            fresh = x.last_valid is not None and t - x.last_valid <= LINK_TIMEOUT_S
            if x.silent_since is not None and self.via:
                x.track_seen = True
            if fresh:
                x.silent_since, x.track_seen = None, False
                x.dist = abs(x.s)
                acerca = (x.s < 0 and x.sentido > 0) or (x.s > 0 and x.sentido < 0)
                x.closing = x.v if acerca else -x.v
                x.eta_min = min_eta(x.closing, x.dist)
                if x.closing > MIN_CLOSING:
                    x.cola_paso = False
                in_zone = x.dist <= OCCUPIED_RADIUS_M and not x.cola_paso
                implausible = t < x.implausible_until
                danger = implausible or (x.dist <= RELEVANT_RADIUS_M and (in_zone or x.eta_min <= ALERT_ETA_S))
                x.phase = ("EN_ZONA" if in_zone else "APROXIMA" if danger
                           else "SE_ALEJA" if x.closing < -MIN_CLOSING else "LEJOS")
                if danger:
                    x.alerting, x.clear_since = True, None
                    x.reason = "dato inconsistente" if implausible else "tren en zona" if in_zone else "tren aproxima"
                elif x.alerting:
                    x.clear_since = x.clear_since if x.clear_since is not None else t
                    if x.cola_paso or t - x.clear_since >= CLEAR_HOLD_S:
                        x.alerting, x.clear_since = False, None
                if x.closing > MIN_CLOSING:
                    x.approach = True
                if x.approach:
                    x.min_dist = min(x.min_dist, x.dist)
                    if not x.passage and x.closing < -MIN_CLOSING and x.dist > x.min_dist + 5:
                        x.passage = True
                        self.pasos.append((round(t, 1), x.id, round(x.min_dist)))
                if not x.alerting and not in_zone and x.phase != "APROXIMA" and x.passage:
                    x.approach, x.passage, x.min_dist = False, False, float("inf")
                continue
            if t - x.last_auth <= LINK_TIMEOUT_S and x.sin_pos:
                x.alerting, x.reason, x.phase, x.silent_since, x.clear_since = True, "tren sin posicion", "SIN_DATOS", None, None
                continue
            silent = (t - x.last_valid) if x.last_valid is not None else 0.0
            could = (x.last_valid is not None and not math.isnan(x.eta_min) and x.dist <= RELEVANT_RADIUS_M
                     and x.eta_min - silent <= ALERT_ETA_S)
            if x.alerting or could:
                if x.silent_since is None:
                    x.silent_since, x.track_seen = t, self.via
                x.alerting, x.reason, x.phase = True, "SIN DATOS", "SIN_DATOS"
                if (x.track_seen and via_fell) or t - x.silent_since > SILENT_RELEASE_S:
                    del self.trenes[x.id]
                continue
            receding = x.closing < -MIN_CLOSING
            far = not (x.dist <= RELEVANT_RADIUS_M)
            if (x.last_valid is None or receding or far) and t - x.last_auth > FORGET_S:
                del self.trenes[x.id]

        # Atribución vigente: el tren al que se atribuyó ya pasó y se alejó.
        if self.via and self.via_explicada:
            b = self.trenes.get(self.via_tren)
            if b is None or (b.phase == "SE_ALEJA" and b.dist > OCCUPIED_RADIUS_M):
                self.via_explicada, self.via_tren = False, 0
                self.eventos.append((round(t, 1), "el tren ya pasó y el circuito sigue ocupado: cierra"))
        via_sin_explicar = self.via and not self.via_explicada

        alerta = [x for x in self.trenes.values() if x.alerting]
        orden = {"SIN DATOS": 5, "tren en zona": 4, "tren sin posicion": 3, "dato inconsistente": 3,
                 "tren aproxima": 2}
        if alerta:
            p = max(alerta, key=lambda x: (orden.get(x.reason, 1), -x.dist if not math.isnan(x.dist) else 0))
            estado = ("NO SEGURO", p.reason, p.id, p.dist, p.eta_min)
        elif via_sin_explicar:
            estado = ("NO SEGURO", "via ocupada sin nodo", 0, float("nan"), float("nan"))
        else:
            estado = ("APAGADO", "sin trenes", 0, float("nan"), float("nan"))
        self.panda_libre = estado[0] == "APAGADO"
        self.otro_tren = len(alerta) >= 2 or (len(alerta) >= 1 and via_sin_explicar)
        self.pedido_cierre = not self.panda_libre     # PANDA operativo en el modelo

        if self.t_prev is not None:
            dt = t - self.t_prev
            self.t_cierre_panda += dt if self.pedido_cierre else 0.0
            self.t_via_ocupada += dt if (self.via or self.via_b) else 0.0
        self.t_prev = t

        clave = estado[:3] + (self.otro_tren,)
        if self.estado is None or clave != self.estado:
            self.transiciones.append((round(t, 1),) + estado + (self.otro_tren,))
        self.estado = clave
