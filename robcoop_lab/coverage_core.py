"""Cobertura con Voronoi sin ROS (proporcionado, no hay que modificarlo).

La zona a cubrir Q es un cuadrado centrado en el origen de 'map'. Se trabaja sobre una REJILLA:
Q se divide en casillas de lado RES y cada casilla se representa por su centro. Las integrales
de la teoría pasan a ser sumas sobre las casillas (ΔA = área de una casilla):

    masa de la celda i      M_i = Σ_{q ∈ V_i} φ(q) · ΔA
    centroide de la celda   C_i = Σ_{q ∈ V_i} φ(q) · q  /  Σ_{q ∈ V_i} φ(q)
    coste de cobertura      H   = Σ_q φ(q) · min_i |q − p_i|² · ΔA

Aquí están:
  - la rejilla (make_grid), las densidades φ (density) y las posiciones de salida (STARTS);
  - lo que hace cada robot con VUESTRO coverage_law.py (robot_step); lo usan igual el nodo de
    ROS (coverage_node.py) y el simulador sin ROS (kinsim_p3.py);
  - funciones de REFERENCIA que no dependen de vuestro código (ref_*): se usan para medir los
    experimentos y para comprobar vuestras funciones (check_p3).
"""
import math
import os

import numpy as np

from .arena import START_POSES
from .kinematics import control_point, integrator_to_unicycle, saturate

LIMIT = 1.5            # m: la zona a cubrir es el cuadrado [-LIMIT, LIMIT] x [-LIMIT, LIMIT]
RES = 0.05             # m: lado de una casilla de la rejilla (60 x 60 = 3600 casillas)

K_DEFAULT = 0.5        # 1/s: ganancia de la ley u_i = k (C_i − p_i)
TOL = 0.02             # m: se da por desplegado cuando todos están a menos de TOL de su centroide
HOLD = 3.0             # s: ... durante HOLD segundos seguidos
STALE = 1.0            # s sin noticias de un robot -> se le da por caído y se le excluye
DEADBAND = 0.002       # m/s: por debajo de esta velocidad el robot no se mueve

# --- Densidades ------------------------------------------------------------------------------
DENSITIES = ('uniforme', 'zona')
ZONE_CENTER = (0.6, -0.6)   # m: centro de la zona de interés
ZONE_SIGMA = 0.5            # m: tamaño de la zona (desviación típica de la gaussiana)
ZONE_PEAK = 12.0            # importancia del centro de la zona respecto al resto (que vale 1)

# --- Posiciones de salida (x, y, yaw) --------------------------------------------------------
#   marcas     las marcas del suelo de la P1 y la P2 (repartidas por la arena)
#   agrupados  los robots juntos en una esquina de la zona: el despliegue clásico
STARTS = {
    'marcas': list(START_POSES),
    'agrupados': [(-0.9, -1.4, 0.8), (-1.4, -0.9, 0.8), (-1.4, -1.4, 0.8), (-0.9, -0.9, 0.8)],
}


def default_out_dir():
    """Carpeta de resultados de la P3: resultados/p3 del repositorio de la pareja."""
    repo = os.path.expanduser('~/ws/src/robcoop_lab')
    if os.path.isdir(repo):
        return os.path.join(repo, 'resultados', 'p3')
    return os.path.expanduser('~/ws/resultados/p3')


def start_poses(name, n):
    """Las n primeras poses (x, y, yaw) de la salida 'name'."""
    if name not in STARTS:
        raise ValueError(f'Salida desconocida: {name}. Opciones: {", ".join(STARTS)}')
    if not 1 <= n <= len(STARTS[name]):
        raise ValueError(f'La salida {name} está definida para 1 a {len(STARTS[name])} robots')
    return list(STARTS[name][:n])


def poses_arg(name, n):
    """Cadena para el argumento 'poses' de multi_robot.launch.py."""
    return '; '.join(f'{x},{y},{yaw}' for x, y, yaw in start_poses(name, n))


# --- Rejilla y densidad ----------------------------------------------------------------------
def make_grid(limit=LIMIT, res=RES):
    """Rejilla que cubre el cuadrado [-limit, limit]².

    Devuelve (points, cell_area):
        points     array (M, 2) con el centro (x, y) de cada casilla
        cell_area  área de una casilla, ΔA = res² (m²)
    """
    n = int(round(2 * limit / res))
    c = -limit + res * (np.arange(n) + 0.5)
    xx, yy = np.meshgrid(c, c)
    return np.column_stack([xx.ravel(), yy.ravel()]), float(res * res)


def density(name='uniforme', points=None, center=ZONE_CENTER, sigma=ZONE_SIGMA, peak=ZONE_PEAK):
    """Importancia φ(q) de cada punto de la rejilla: array (M,).

    uniforme  φ = 1 en toda la zona: todos los puntos importan lo mismo.
    zona      φ = 1 + (peak − 1) · exp(−|q − center|² / (2 sigma²)): hay una zona de interés
              alrededor de 'center' en la que los puntos importan hasta 'peak' veces más.
    """
    if points is None:
        points = make_grid()[0]
    points = np.asarray(points, dtype=float)
    if name == 'uniforme':
        return np.ones(len(points))
    if name == 'zona':
        d2 = np.sum((points - np.asarray(center, dtype=float)) ** 2, axis=1)
        return 1.0 + (float(peak) - 1.0) * np.exp(-d2 / (2.0 * float(sigma) ** 2))
    raise ValueError(f'Densidad desconocida: {name}. Opciones: {", ".join(DENSITIES)}')


# --- Lo que hace UN robot en cada ciclo, con el código del alumno ----------------------------
def robot_step(law, points, weights, p_i, p_others, k):
    """Centroide de la celda del robot y velocidad deseada, con las funciones de 'law'.

    p_i: (x, y) del robot; p_others: lista con las posiciones de los demás que conoce.
    Devuelve (C_i, u_i) como arrays (2,). Si la celda está vacía, (None, [0, 0]).
    """
    p_i = np.asarray(p_i, dtype=float)
    others = [np.asarray(p, dtype=float) for p in p_others]
    mask = np.asarray(law.voronoi_mask(points, p_i, others), dtype=bool)
    c = law.centroid(points, weights, mask)
    if c is None:
        return None, np.zeros(2)
    c = np.asarray(c, dtype=float).reshape(2)
    if not np.all(np.isfinite(c)):
        return None, np.zeros(2)
    u = np.asarray(law.coverage_velocity(p_i, c, k), dtype=float).reshape(2)
    return c, u


def robot_command(law, points, weights, pose, p_others, k):
    """Orden (v, ω) de un robot. Es lo mismo en el nodo de ROS y en kinsim_p3:

      1. q_i = punto de control del robot (10 cm delante del eje, como en la P1);
      2. C_i y u_i = k (C_i − q_i) con VUESTRO coverage_law.py (robot_step);
      3. saturación a V_MAX y conversión a (v, ω) con la cinemática de la P1.

    No hay evitación de obstáculos: cada robot va en línea recta hacia un punto de SU celda,
    que es convexa, así que no sale de ella ni se cruza con los demás. El filtro de seguridad
    (safety.py) sigue actuando después, por si acaso.

    pose: (x, y, θ); p_others: puntos de control de los demás robots que siguen activos.
    Devuelve (v, ω, C_i, |C_i − q_i|). Sin centroide (celda vacía): (0, 0, None, None).
    """
    x, y, th = pose
    q = np.array(control_point(x, y, th))
    c, u = robot_step(law, points, weights, q, p_others, k)
    if c is None:
        return 0.0, 0.0, None, None
    dist = float(np.hypot(c[0] - q[0], c[1] - q[1]))
    ux, uy = saturate(float(u[0]), float(u[1]))
    if math.hypot(ux, uy) < DEADBAND:
        return 0.0, 0.0, c, dist
    v, w = integrator_to_unicycle(ux, uy, th)
    return v, w, c, dist


# --- Referencia (independiente del código del alumno) ----------------------------------------
def _sq_dists(points, robots):
    """Matriz (n robots, M puntos) con las distancias al cuadrado."""
    robots = np.asarray(robots, dtype=float).reshape(-1, 2)
    diff = points[None, :, :] - robots[:, None, :]
    return np.sum(diff * diff, axis=2)


def ref_owner(points, robots):
    """Índice del robot más cercano a cada punto: array (M,) de enteros."""
    return np.argmin(_sq_dists(points, robots), axis=0)


def ref_mask(points, p_i, p_others):
    """Celda de Voronoi de referencia (los puntos de la frontera son de las dos celdas)."""
    d_i = np.sum((points - np.asarray(p_i, dtype=float)) ** 2, axis=1)
    mask = np.ones(len(points), dtype=bool)
    for p in p_others:
        mask &= d_i <= np.sum((points - np.asarray(p, dtype=float)) ** 2, axis=1) + 1e-12
    return mask


def ref_centroids(points, weights, robots, cell_area=1.0):
    """Centroides y masas de las celdas de todos los robots.

    Devuelve (C, M): C array (n, 2) (NaN si la celda está vacía) y M array (n,).
    """
    robots = np.asarray(robots, dtype=float).reshape(-1, 2)
    C = np.full((len(robots), 2), np.nan)
    M = np.zeros(len(robots))
    for i in range(len(robots)):
        mask = ref_mask(points, robots[i], [robots[j] for j in range(len(robots)) if j != i])
        m = float(np.sum(weights[mask]))
        M[i] = m * cell_area
        if m > 0:
            C[i] = (weights[mask, None] * points[mask]).sum(axis=0) / m
    return C, M


def ref_cost(points, weights, robots, cell_area):
    """Coste de cobertura H de referencia."""
    return float(np.sum(weights * np.min(_sq_dists(points, robots), axis=0)) * cell_area)


def rms_distance(cost, weights, cell_area):
    """Distancia cuadrática media (m) de un punto de la zona a su robot más cercano:
    d_rms = sqrt(H / masa total). Es H en unidades fáciles de interpretar."""
    return math.sqrt(max(cost, 0.0) / (float(np.sum(weights)) * cell_area))


def ref_error(points, weights, robots):
    """Error de cobertura: max_i |C_i − p_i| (m). 0 = cada robot está en su centroide."""
    robots = np.asarray(robots, dtype=float).reshape(-1, 2)
    C, _ = ref_centroids(points, weights, robots)
    d = np.linalg.norm(C - robots, axis=1)
    return float(np.nanmax(d)) if np.any(np.isfinite(d)) else 0.0


def lloyd(robots, points, weights, cell_area, law=None, iters=150, tol=1e-3):
    """Algoritmo de Lloyd ideal: cada robot salta a su centroide hasta que nadie se mueve.

    Es el límite de la ley u_i = k (C_i − p_i) para robots que se mueven sin restricciones;
    sirve de referencia de adónde debería llegar el equipo.
    law: módulo con voronoi_mask, centroid y coverage_cost (None = funciones de referencia).
    Devuelve (posiciones finales (n, 2), [H tras cada iteración]).
    """
    P = np.asarray(robots, dtype=float).reshape(-1, 2).copy()

    def cost(P):
        if law is None:
            return ref_cost(points, weights, P, cell_area)
        return float(law.coverage_cost(points, weights, [p for p in P], cell_area))

    def centroids(P):
        if law is None:
            C, _ = ref_centroids(points, weights, P)
            return np.where(np.isfinite(C), C, P)
        C = P.copy()
        for i in range(len(P)):
            c, _ = robot_step(law, points, weights, P[i],
                              [P[j] for j in range(len(P)) if j != i], 1.0)
            if c is not None:
                C[i] = c
        return C

    costs = [cost(P)]
    for _ in range(iters):
        C = centroids(P)
        move = float(np.max(np.linalg.norm(C - P, axis=1)))
        P = C
        costs.append(cost(P))
        if move < tol:
            break
    return P, costs
