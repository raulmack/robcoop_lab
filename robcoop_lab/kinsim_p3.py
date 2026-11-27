"""Simulador cinemático de la Práctica 3, sin ROS ni Gazebo (proporcionado).

Mueve los robots con VUESTRO coverage_law.py, la misma cinemática, un LiDAR simplificado
(cada robot ve a los demás) y el mismo filtro de seguridad. Sirve para ver en segundos adónde
va el equipo antes de ir a Gazebo, y es lo que usa check_p3.

    ros2 run robcoop_lab kinsim_p3 --plot
    ros2 run robcoop_lab kinsim_p3 --salida agrupados --densidad zona --plot
    python3 -m robcoop_lab.kinsim_p3 -n 4 --fallo 1
"""
import argparse
import math

import numpy as np

from . import coverage_core as core
from . import coverage_law
from .kinematics import control_point
from .kinsim_p2 import R_BODY, _scan_points
from .safety import SECTOR, STOP_DIST


def simulate(start='marcas', n=3, density='uniforme', k=core.K_DEFAULT, law=None, fail=None,
             poses=None, center=core.ZONE_CENTER, sigma=core.ZONE_SIGMA, peak=core.ZONE_PEAK,
             dt=0.05, t_max=180.0, tol=core.TOL, hold=core.HOLD, safety=True):
    """Simula el despliegue y devuelve un diccionario con trayectorias y métricas.

    law:  módulo con las cuatro funciones (por defecto, el del alumno).
    fail: índice del robot que se avería cuando el equipo ya se ha desplegado (None = ninguno).
    """
    law = law or coverage_law
    poses = [list(p) for p in (poses or core.start_poses(start, n))]
    n = len(poses)
    points, cell_area = core.make_grid()
    weights = core.density(density, points, center, sigma, peak)
    alive = [True] * n
    phase, t, stable = 'cobertura', 0.0, 0.0
    t_conv = t_fail = t_reconv = None
    T, H, HL, E, P, A = [], [], [], [], [], []
    min_d, blocked, travelled = math.inf, 0, [0.0] * n
    converged = False
    while t <= t_max:
        q = [np.array(control_point(*p)) for p in poses]
        act = [i for i in range(n) if alive[i]]
        qa = [q[i] for i in act]
        cost = core.ref_cost(points, weights, qa, cell_area)
        try:
            cost_law = float(law.coverage_cost(points, weights, qa, cell_area))
        except Exception:  # noqa: BLE001
            cost_law = float('nan')
        err = core.ref_error(points, weights, qa)
        T.append(t)
        H.append(cost)
        HL.append(cost_law)
        E.append(err)
        P.append([p[:] for p in poses])
        A.append(list(alive))
        for i in range(n):
            for j in range(i + 1, n):
                min_d = min(min_d, math.dist(poses[i][:2], poses[j][:2]))
        stable = stable + dt if err < tol else 0.0
        if stable >= hold:
            reached = round(t - stable + dt, 2)
            if phase == 'cobertura':
                t_conv = reached
                if fail is not None and alive[fail]:
                    phase, t_fail, stable = 'recuperacion', t, 0.0
                    alive[fail] = False
                else:
                    converged = True
                    break
            else:
                t_reconv = round(reached - t_fail, 2)
                converged = True
                break
        new = [p[:] for p in poses]
        for i in act:
            if not alive[i]:
                continue
            others = [q[j] for j in act if j != i and alive[j]]
            v, w, _, _ = core.robot_command(law, points, weights, poses[i], others, k)
            obs = _scan_points(i, poses)
            if safety and abs(v) > 1e-3:
                center_a = 0.0 if v > 0 else math.pi
                for ox, oy in obs:
                    a = (math.atan2(oy, ox) - center_a + math.pi) % (2 * math.pi) - math.pi
                    if math.hypot(ox, oy) < STOP_DIST and abs(a) <= SECTOR:
                        v = 0.0
                        blocked += 1
                        break
            x, y, th = poses[i]
            new[i] = [x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt]
            travelled[i] += abs(v) * dt
        poses = new
        t += dt
    act = [i for i in range(n) if alive[i]]
    qf = np.array([control_point(*poses[i]) for i in act])
    C, M = core.ref_centroids(points, weights, qf, cell_area)
    return {'t': np.array(T), 'cost': np.array(H), 'cost_law': np.array(HL),
            'error': np.array(E), 'poses': np.array(P), 'alive': np.array(A),
            'converged': converged, 't_conv': t_conv, 't_fail': t_fail, 't_reconv': t_reconv,
            'final_error': E[-1], 'final_cost': H[-1], 'active': act, 'final_q': qf,
            'centroids': C, 'masses': M, 'min_dist': min_d, 'collision': min_d < 2 * R_BODY,
            'blocked_steps': blocked, 'travelled': travelled,
            'rms': core.rms_distance(H[-1], weights, cell_area),
            'points': points, 'weights': weights, 'cell_area': cell_area}


def main():
    p = argparse.ArgumentParser(description='Cobertura con Voronoi, simulación cinemática (sin ROS)')
    p.add_argument('--salida', default='marcas', choices=tuple(core.STARTS))
    p.add_argument('--densidad', default='uniforme', choices=core.DENSITIES)
    p.add_argument('-k', type=float, default=core.K_DEFAULT, help='ganancia (0.5)')
    p.add_argument('-n', type=int, default=3, help='número de robots (2 a 4)')
    p.add_argument('--fallo', type=int, default=None,
                   help='número del robot que se avería tras el despliegue (p. ej. 1)')
    p.add_argument('--plot', action='store_true', help='mostrar gráficas')
    a = p.parse_args()

    r = simulate(a.salida, a.n, a.densidad, a.k, fail=a.fallo)
    tc = f'{r["t_conv"]:.1f} s' if r['t_conv'] is not None else 'NO CONVERGE'
    print(f'{a.salida} · {a.densidad} · {a.n} robots · k={a.k}')
    print(f'  despliegue: {tc}   coste H: {r["cost"][0]:.2f} -> {r["final_cost"]:.2f} m⁴   '
          f'distancia media a un robot: {r["rms"]:.2f} m')
    if a.fallo is not None:
        tr = f'{r["t_reconv"]:.1f} s' if r['t_reconv'] is not None else 'NO CONVERGE'
        print(f'  tb3_{a.fallo} se avería en t={r["t_fail"]:.1f} s; los demás se recolocan en {tr}')
    for i, qf, m in zip(r['active'], r['final_q'], r['masses']):
        print(f'  tb3_{i}: ({qf[0]:+.2f}, {qf[1]:+.2f})   masa de su celda {m:.2f}   '
              f'recorrido {r["travelled"][i]:.2f} m')
    print(f'  distancia mínima entre robots {r["min_dist"]:.2f} m'
          + ('  ¡CHOQUE!' if r['collision'] else ''))
    if a.plot:
        import matplotlib.pyplot as plt
        from .plot_p3 import draw_arena
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.4))
        names = [f'tb3_{i}' for i in range(a.n)]
        xy = {nm: np.array([control_point(*p) for p in r['poses'][:, i]])
              for i, nm in enumerate(names)}
        draw_arena(ax1, names, xy, [names[i] for i in r['active']], r['points'], r['weights'])
        ax2.plot(r['t'], r['cost'], color='#14213D')
        ax2.set_xlabel('t (s)')
        ax2.set_ylabel('coste de cobertura H (m⁴)')
        ax2.grid(True, alpha=0.4)
        if r['t_fail'] is not None:
            ax2.axvline(r['t_fail'], color='#C2410C', ls='--', label='avería')
            ax2.legend()
        fig.suptitle(f'kinsim_p3 · {a.salida} · {a.densidad}')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
