"""PRÁCTICA 3 — Cobertura con Voronoi.  >>> ESTE ES EL FICHERO QUE PROGRAMÁIS <<<

Solo hay que completar las cuatro funciones marcadas con TODO. El resto del paquete
(nodos ROS, rejilla, densidades, cinemática, seguridad, registro y gráficas) ya está hecho
y las usa.

Comprobación rápida, sin ROS ni Gazebo:
    cd ~/ws/src/robcoop_lab
    python3 -m pytest test/test_coverage_law.py -v
    ros2 run robcoop_lab check_p3

Notación (ver el guion):
    points     rejilla de la zona a cubrir: array (M, 2), una fila (x, y) por casilla
    weights    importancia φ de cada casilla: array (M,)  (todo unos si es uniforme)
    p_i        posición (x, y) del robot i (su punto de control), en metros y en 'map'
    V_i        celda de Voronoi del robot i: las casillas que le quedan más cerca que a nadie
    C_i        centroide de V_i: media de sus casillas, ponderada con φ
"""
import numpy as np


def voronoi_mask(points, p_i, p_others):
    """Celda de Voronoi del robot i sobre la rejilla.

    Una casilla q es del robot i si está más cerca de p_i (o igual) que de CUALQUIER otro:

        |q − p_i| <= |q − p_j|   para todos los demás robots j

    points:    array (M, 2)
    p_i:       array (2,), posición del robot i
    p_others:  lista de arrays (2,) con las posiciones de los demás (puede estar vacía:
               entonces toda la rejilla es del robot i)
    Devuelve:  array booleano (M,): True en las casillas que son del robot i.
    Pista: np.linalg.norm(points − p, axis=1) da la distancia de cada casilla a p.
    """
    points = np.asarray(points, dtype=float)
    mask = np.ones(len(points), dtype=bool)
    # TODO (1): poner a False las casillas que están más cerca de algún otro robot
    return mask


def centroid(points, weights, mask):
    """Centroide de una celda: la media de sus casillas ponderada por la densidad.

        C = Σ φ(q) · q  /  Σ φ(q)        (sumas sobre las casillas con mask = True)

    points:   array (M, 2);  weights: array (M,);  mask: array booleano (M,)
    Devuelve: array (2,) con [C_x, C_y], o None si la celda no tiene masa (Σ φ = 0, por
              ejemplo si no tiene ninguna casilla).
    Pista: points[mask] y weights[mask] se quedan solo con las casillas de la celda.
    """
    points = np.asarray(points, dtype=float)
    weights = np.asarray(weights, dtype=float)
    # TODO (2): calcular y devolver el centroide (o None si la masa es 0)
    return None


def coverage_velocity(p_i, c_i, k):
    """Velocidad deseada del robot i: ir hacia el centroide de su celda.

        u_i = k · (C_i − p_i)

    p_i, c_i:  arrays (2,);  k: ganancia (> 0)
    Devuelve:  array (2,) con [u_x, u_y] en m/s (sin saturar: eso ya se hace fuera)
    """
    p_i = np.asarray(p_i, dtype=float)
    c_i = np.asarray(c_i, dtype=float)
    u = np.zeros(2)
    # TODO (3): calcular u a partir de p_i, c_i y k
    return u


def coverage_cost(points, weights, robots, cell_area):
    """Coste de cobertura del equipo (cuanto MENOR, mejor cubierta está la zona).

    Cada casilla aporta su importancia por el cuadrado de la distancia a su robot más cercano:

        H = Σ_q φ(q) · min_i |q − p_i|² · ΔA

    points:    array (M, 2);  weights: array (M,)
    robots:    lista de arrays (2,) con las posiciones de los robots (al menos uno)
    cell_area: ΔA, área de una casilla (m²)
    Devuelve:  float.
    Pista: np.minimum(a, b) da, casilla a casilla, el menor de los dos arrays.
    """
    points = np.asarray(points, dtype=float)
    weights = np.asarray(weights, dtype=float)
    # TODO (4): calcular y devolver H
    return 0.0
