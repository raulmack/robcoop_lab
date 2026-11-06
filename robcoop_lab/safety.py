"""Filtro de seguridad (proporcionado, no hay que modificarlo).

Se aplica a TODAS las órdenes de velocidad antes de enviarlas al robot:
  - limita v y ω a los máximos de la práctica;
  - si el LiDAR ve algo a menos de STOP_DIST en el sector hacia el que avanza
    el robot, anula la velocidad lineal en ese sentido (puede seguir girando).
"""
import math

from .kinematics import V_MAX, W_MAX, wrap

STOP_DIST = 0.20                 # m, medido desde el centro del LiDAR
SECTOR = math.radians(35)        # semiancho del sector vigilado
MIN_MARGIN = 0.005               # m: lecturas a range_min + MIN_MARGIN o menos no son válidas


def min_range_in_sector(ranges, angle_min, angle_increment, range_min, center):
    """Distancia mínima válida en el sector [center - SECTOR, center + SECTOR].

    Las lecturas iguales al mínimo del sensor se ignoran: un LiDAR que no mide (p. ej. el
    simulado en algunas GPU) devuelve range_min en todos los rayos y bloquearía el robot.
    """
    best = math.inf
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= range_min + MIN_MARGIN:
            continue
        a = wrap(angle_min + i * angle_increment - center)
        if abs(a) <= SECTOR and r < best:
            best = r
    return best


def scan_is_degenerate(scan):
    """True si TODOS los rayos marcan el mínimo del sensor: el LiDAR no está midiendo.

    (Los ceros del LDS-01 real, que significan «sin eco», no cuentan.)
    """
    if scan is None or not scan.ranges:
        return False
    lo, hi = scan.range_min - 1e-3, scan.range_min + MIN_MARGIN
    return all(math.isfinite(r) and lo <= r <= hi for r in scan.ranges)


class SafetyFilter:
    """Guarda el último scan y filtra (v, ω)."""

    def __init__(self, stop_dist=STOP_DIST, v_max=V_MAX, w_max=W_MAX):
        self.stop_dist = stop_dist
        self.v_max = v_max
        self.w_max = w_max
        self.scan = None
        self.blocked = False          # True si la última orden se ha recortado

    def update_scan(self, msg):
        self.scan = msg

    def filter(self, v, w):
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))
        self.blocked = False
        if self.scan is not None and abs(v) > 1e-3:
            center = 0.0 if v > 0 else math.pi
            s = self.scan
            if min_range_in_sector(s.ranges, s.angle_min, s.angle_increment,
                                   max(s.range_min, 0.05), center) < self.stop_dist:
                v = 0.0
                self.blocked = True
        return v, w
