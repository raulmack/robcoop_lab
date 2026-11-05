"""Simulador cinemático de la Práctica 2, sin ROS ni Gazebo (proporcionado).

Hace la subasta con VUESTRO auction_law.py y después mueve los robots por sus rutas con el
mismo controlador (navigation.py), un LiDAR simplificado (cada robot ve a los demás) y el
mismo filtro de seguridad. Sirve para ver en segundos qué hace la asignación antes de ir a
Gazebo, y es lo que usa check_p2.

    ros2 run robcoop_lab kinsim_p2 --disposicion B --criterio max --plot
    python3 -m robcoop_lab.kinsim_p2 --disposicion aleatoria --semilla 3
"""
import argparse
import math

import numpy as np

from . import auction_core, auction_law, tasks as tasks_mod
from .arena import START_POSES
from .kinematics import control_point
from .navigation import GOAL_TOL, GoToPoint
from .safety import SECTOR, STOP_DIST

R_LIDAR_TARGET = 0.05     # radio con el que un robot «ve» a otro (la torre del LiDAR)
R_BODY = 0.105            # radio del robot: dos centros a < 2·R_BODY = choque


def _scan_points(i, poses, max_range=0.6):
    """Puntos de los otros robots en el marco del robot i (LiDAR simplificado)."""
    x, y, th = poses[i]
    c, s = math.cos(th), math.sin(th)
    pts = []
    for j, (xj, yj, _) in enumerate(poses):
        if j == i:
            continue
        dx, dy = xj - x, yj - y
        D = math.hypot(dx, dy)
        if D - R_LIDAR_TARGET > max_range or D < 1e-6:
            continue
        bx, by = c * dx + s * dy, -s * dx + c * dy          # centro del otro, marco del robot
        base = math.atan2(by, bx)
        half = math.asin(min(1.0, R_LIDAR_TARGET / D))
        for f in (-1, -0.5, 0, 0.5, 1):
            a = base + f * half
            chord = max(0.0, R_LIDAR_TARGET ** 2 - (D * math.sin(a - base)) ** 2)
            r = D * math.cos(a - base) - math.sqrt(chord)
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def execute(poses, task_xy, routes, dt=0.05, t_max=240.0, dwell=1.0):
    """Mueve los robots por sus rutas. Devuelve trayectorias, tiempos y métricas.

    poses: [(x, y, θ)] iniciales; routes: [[nombre, ...] por robot]; task_xy: {nombre: (x, y)}.
    """
    poses = [list(p) for p in poses]
    n = len(poses)
    idx = [0] * n
    wait = [0.0] * n
    finish = [None] * n
    reached = {}
    dist = [0.0] * n
    nav = [GoToPoint(priority=i) for i in range(n)]
    for i in range(n):
        if routes[i]:
            nav[i].set_goal(task_xy[routes[i][0]])
    T, P = [], []
    min_d, blocked = math.inf, 0
    t = 0.0
    while t <= t_max:
        T.append(t)
        P.append([p[:] for p in poses])
        for i in range(n):
            for j in range(i + 1, n):
                min_d = min(min_d, math.dist(poses[i][:2], poses[j][:2]))
        if all(f is not None for f in finish):
            break
        new = [p[:] for p in poses]
        for i in range(n):
            if finish[i] is not None:
                continue
            if idx[i] >= len(routes[i]):
                finish[i] = t
                continue
            if wait[i] > 0:
                wait[i] -= dt
                if wait[i] <= 0:
                    idx[i] += 1
                    if idx[i] < len(routes[i]):
                        nav[i].set_goal(task_xy[routes[i][idx[i]]])
                continue
            name = routes[i][idx[i]]
            obs = _scan_points(i, poses)
            v, w, d = nav[i].command(poses[i], obs, t)
            if d < GOAL_TOL:
                reached[name] = (i, round(t, 2))
                wait[i] = dwell
                continue
            if abs(v) > 1e-3:
                center = 0.0 if v > 0 else math.pi
                for ox, oy in obs:
                    a = (math.atan2(oy, ox) - center + math.pi) % (2 * math.pi) - math.pi
                    if math.hypot(ox, oy) < STOP_DIST and abs(a) <= SECTOR:
                        v = 0.0
                        blocked += 1
                        break
            x, y, th = poses[i]
            new[i] = [x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt]
            dist[i] += abs(v) * dt
        poses = new
        t += dt
    done = len(reached)
    return {'t': np.array(T), 'poses': np.array(P), 'reached': reached, 'finish': finish,
            'makespan': max(f for f in finish) if all(f is not None for f in finish) else None,
            'completed': done, 'total': sum(len(r) for r in routes), 'distance': dist,
            'min_dist': min_d, 'collision': min_d < 2 * R_BODY, 'blocked_steps': blocked,
            'detours': [g.detours for g in nav], 'backups': [g.backups for g in nav]}


def simulate(layout='A', criterion='suma', n=3, points='', seed=0, law=None, **kw):
    """Subasta (sin ROS) + ejecución. Devuelve (resultado de la subasta, de la ejecución)."""
    law = law or auction_law
    poses = START_POSES[:n]
    robots = [f'tb3_{i}' for i in range(n)]
    task_xy = tasks_mod.get(layout, points, seed)
    starts = {r: control_point(*p) for r, p in zip(robots, poses)}
    auc = auction_core.ssi(starts, task_xy, criterion, law)
    ex = execute(poses, task_xy, [auc['routes'][r] for r in robots], **kw)
    return auc, ex, task_xy


def main():
    p = argparse.ArgumentParser(description='Subasta + ejecución cinemática (sin ROS)')
    p.add_argument('--disposicion', default='A', choices=tasks_mod.NAMES)
    p.add_argument('--criterio', default='suma', choices=auction_core.CRITERIA)
    p.add_argument('--puntos', default='', help='"x,y; x,y; ..." (sustituye a la disposición)')
    p.add_argument('--semilla', type=int, default=0, help='para la disposición aleatoria')
    p.add_argument('-n', type=int, default=3, help='número de robots (2 a 4)')
    p.add_argument('--plot', action='store_true', help='mostrar gráfica')
    a = p.parse_args()

    auc, ex, task_xy = simulate(a.disposicion, a.criterio, a.n, a.puntos, a.semilla)
    print(f'Subasta ({a.criterio}): distancia total planificada {auc["sum"]:.2f} m, '
          f'ruta más larga {auc["max"]:.2f} m, {auc["messages"]} mensajes')
    for r, route in auc['routes'].items():
        print(f'  {r}: {" -> ".join(route) or "(sin tareas)"}   {auc["lengths"][r]:.2f} m')
    mk = f'{ex["makespan"]:.1f} s' if ex['makespan'] is not None else 'NO TERMINA'
    print(f'Ejecución: {ex["completed"]}/{ex["total"]} tareas, tiempo total {mk}, '
          f'distancia mínima entre robots {ex["min_dist"]:.2f} m'
          + ('  ¡CHOQUE!' if ex['collision'] else ''))
    if a.plot:
        import matplotlib.pyplot as plt
        from .plot_p2 import COLORS
        fig, ax = plt.subplots(figsize=(6.5, 6.5))
        for i in range(a.n):
            c = COLORS[i % len(COLORS)]
            ax.plot(ex['poses'][:, i, 0], ex['poses'][:, i, 1], color=c, label=f'tb3_{i}')
            ax.plot(*ex['poses'][0, i, :2], 'o', color=c)
        for name, (x, y) in task_xy.items():
            ax.plot(x, y, 's', color='#555', ms=9, mfc='none')
            ax.annotate(name, (x, y), xytext=(6, 6), textcoords='offset points')
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.4)
        ax.legend()
        ax.set_title(f'kinsim_p2 · {a.disposicion} · {a.criterio}')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
