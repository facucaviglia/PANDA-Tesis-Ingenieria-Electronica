"""
PANDA · Modelo de referencia de la lógica del cruce, en Python.

Reproduce las reglas de src/crossing.cpp con los mismos parámetros de
include/config.h (cfg::crossing). Si se cambia uno, hay que cambiarlo en los
dos lados. Lo usa tools/sim_trenes.py --seco para mostrar, sin placa, qué
secuencia de estados tendría que dar el nodo cruce en cada escenario.

Simplificación: la vía es recta y pasa por el cruce, así que la distancia es
|s| y la velocidad de acercamiento es +v o -v según el sentido.
"""

import math

# --- Parámetros (espejo de cfg::crossing y cfg::beacon) ----------------------
ALERT_ETA_S = 25.0
MAX_ACCEL = 1.0
ACCEL_KNEE = 11.1
LINE_MAX = 33.3
OCCUPIED_RADIUS_M = 250.0 + 30.0
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


class TrenModelo:
    def __init__(self, node_id, t):
        self.id = node_id
        self.last_auth = t
        self.last_valid = None
        self.s = self.v = 0.0
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
        self.approach = False
        self.passage = False
        self.min_dist = float("inf")


class ModeloCruce:
    def __init__(self):
        self.trenes = {}
        self.via = False
        self.estado = None
        self.transiciones = []
        self.pasos = []
        self.via_sin_panda = 0
        self.panda_libre = True

    def beacon(self, t, node_id, s, v, sentido, con_fix):
        x = self.trenes.get(node_id) or self.trenes.setdefault(node_id, TrenModelo(node_id, t))
        x.last_auth = t
        if con_fix:
            x.last_valid, x.s, x.v, x.sentido, x.sin_pos = t, s, v, sentido, False
        else:
            x.sin_pos = True

    def set_via(self, ocupada):
        if ocupada and not self.via and self.panda_libre:
            self.via_sin_panda += 1
        self.via_fell = self.via and not ocupada
        self.via = ocupada

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
                in_zone = x.dist <= OCCUPIED_RADIUS_M
                danger = x.dist <= RELEVANT_RADIUS_M and (in_zone or x.eta_min <= ALERT_ETA_S)
                x.phase = ("EN_ZONA" if in_zone else "APROXIMA" if danger
                           else "SE_ALEJA" if x.closing < -MIN_CLOSING else "LEJOS")
                if danger:
                    x.alerting, x.reason, x.clear_since = True, ("tren en zona" if in_zone else "tren aproxima"), None
                elif x.alerting:
                    x.clear_since = x.clear_since if x.clear_since is not None else t
                    if t - x.clear_since >= CLEAR_HOLD_S:
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

        alerta = [x for x in self.trenes.values() if x.alerting]
        orden = {"SIN DATOS": 5, "tren en zona": 4, "tren sin posicion": 3, "tren aproxima": 2}
        if alerta:
            p = max(alerta, key=lambda x: (orden.get(x.reason, 1), -x.dist if not math.isnan(x.dist) else 0))
            estado = ("NO SEGURO", p.reason, p.id, p.dist, p.eta_min)
        elif self.via:
            estado = ("NO SEGURO", "via ocupada", 0, float("nan"), float("nan"))
        else:
            estado = ("APAGADO", "sin trenes", 0, float("nan"), float("nan"))
        self.panda_libre = not alerta
        if self.estado is None or estado[:3] != self.estado[:3]:
            self.transiciones.append((round(t, 1),) + estado)
        self.estado = estado
