"""Puntos objetivo (tareas) de la Práctica 2 (proporcionado; se pueden añadir disposiciones).

Coordenadas en metros, marco 'map' de la arena (origen en el centro, paredes en ±2 m).
Cada tarea tiene un nombre P0, P1, ... y en la arena real se marca con cinta en el suelo.

    A  repartidos por toda la arena
    B  agrupados en una zona (cerca de tb3_0): la carga se reparte mal
    C  en dos filas que cruzan la arena
    aleatoria   6 puntos al azar (argumento semilla), separados al menos 0.6 m
"""
import math
import os
import random

from .arena import START_POSES

LAYOUTS = {
    'A': [(0.0, 1.5), (1.5, 0.0), (0.0, -1.5), (-1.5, 0.0), (0.5, 0.4), (-0.5, -0.4)],
    'B': [(1.5, -0.2), (1.5, 0.5), (0.8, -0.6), (0.8, 0.1), (0.2, 0.6), (1.5, -0.9)],
    'C': [(-1.5, 0.3), (-0.5, 0.3), (0.5, 0.3), (-1.0, -0.4), (0.0, -0.4), (1.0, -0.4)],
}
NAMES = tuple(LAYOUTS) + ('aleatoria',)

MIN_SEP = 0.6          # m entre puntos
MIN_START = 0.5        # m a las marcas de salida
LIMIT = 1.6            # |x|, |y| máximos (las paredes están en ±2 m)


def as_dict(points):
    """[(x, y), ...] -> {'P0': (x, y), 'P1': ...} (el orden de los nombres es el de la lista)."""
    return {f'P{i}': (float(x), float(y)) for i, (x, y) in enumerate(points)}


def random_points(n=6, seed=0):
    """n puntos al azar dentro de la arena, separados entre sí y de las marcas de salida."""
    rng = random.Random(seed)
    pts = []
    for _ in range(20000):
        p = (round(rng.uniform(-LIMIT, LIMIT), 2), round(rng.uniform(-LIMIT, LIMIT), 2))
        if all(math.dist(p, q) >= MIN_SEP for q in pts) and \
           all(math.dist(p, s[:2]) >= MIN_START for s in START_POSES):
            pts.append(p)
            if len(pts) == n:
                return pts
    raise RuntimeError('No se han podido colocar los puntos; prueba otra semilla')


def parse_points(text):
    """'x,y; x,y; ...' -> [(x, y), ...]"""
    pts = []
    for item in text.split(';'):
        if item.strip():
            x, y = (float(v) for v in item.split(',')[:2])
            pts.append((x, y))
    return pts


def get(layout='A', points='', seed=0, n=6):
    """Tareas como diccionario {nombre: (x, y)}.

    points (texto 'x,y; ...') tiene prioridad sobre layout.
    """
    if points.strip():
        return as_dict(parse_points(points))
    if layout == 'aleatoria':
        return as_dict(random_points(n, seed))
    if layout not in LAYOUTS:
        raise ValueError(f'Disposición desconocida: {layout}. Opciones: {", ".join(NAMES)}')
    return as_dict(LAYOUTS[layout])


def default_out_dir():
    """Carpeta de resultados de la P2: resultados/p2 del repositorio de la pareja."""
    repo = os.path.expanduser('~/ws/src/robcoop_lab')
    if os.path.isdir(repo):
        return os.path.join(repo, 'resultados', 'p2')
    return os.path.expanduser('~/ws/resultados/p2')
