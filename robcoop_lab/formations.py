"""Offsets de formación δ_i (proporcionado; se pueden añadir formaciones propias).

Los offsets se dan respecto al centro de la formación, en metros y en el marco 'map'.
El orden importa: el offset i corresponde al robot i de la lista 'robots'. Las tablas
están ordenadas para que, desde las marcas de salida de la arena (config/p1.yaml),
las trayectorias no se crucen con ninguna de las topologías.
"""
import math

FORMATIONS = ('reunion', 'linea', 'poligono', 'triangulo')

_H = 0.8 * math.sqrt(3) / 2        # altura del triángulo de lado 0.8 m
_TABLES = {
    # fila sobre el eje y, separación 0.5 m
    ('linea', 3): [(0.0, 0.0), (0.0, 0.5), (0.0, -0.5)],
    ('linea', 4): [(0.0, 0.25), (0.0, 0.75), (0.0, -0.25), (0.0, -0.75)],
    # triángulo de lado 0.8 m con el vértice hacia +y; con 4 robots, uno en el centro de la base
    ('triangulo', 3): [(0.4, -_H / 3), (0.0, 2 * _H / 3), (-0.4, -_H / 3)],
    ('triangulo', 4): [(0.0, 2 * _H / 3), (0.0, -_H / 3), (-0.4, -_H / 3), (0.4, -_H / 3)],
}


def _polygon(n, radius, phase=math.pi / 4):
    return [(radius * math.cos(phase + 2 * math.pi * i / n),
             radius * math.sin(phase + 2 * math.pi * i / n)) for i in range(n)]


def offsets(formation, n):
    """Lista de n offsets (dx, dy).

    reunion   : los robots se juntan en un círculo pequeño (r = 0.30 m). Con δ_i = 0
                todos irían al mismo punto, imposible con robots de verdad.
    linea     : en fila sobre el eje y, separados 0.5 m (3 o 4 robots).
    poligono  : polígono regular de radio 0.6 m (triángulo con 3 robots, cuadrado con 4).
    triangulo : triángulo de lado 0.8 m (3 o 4 robots).
    """
    if formation == 'reunion':
        return _polygon(n, 0.30)
    if formation == 'poligono':
        return _polygon(n, 0.60)
    if (formation, n) in _TABLES:
        return list(_TABLES[(formation, n)])
    if formation in FORMATIONS:
        raise ValueError(f'La formación {formation} está definida para 3 o 4 robots')
    raise ValueError(f'Formación desconocida: {formation}. Opciones: {", ".join(FORMATIONS)}')
