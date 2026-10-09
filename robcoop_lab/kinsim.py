"""Simulador cinemático rápido, sin ROS ni Gazebo (proporcionado).

Usa la MISMA ley (consensus_law.py), la misma cinemática y un filtro de seguridad
equivalente al de los robots. Sirve para probar la ley en segundos antes de ir a Gazebo
y es lo que usa check_p1 para evaluar.

    ros2 run robcoop_lab kinsim --topologia anillo --formacion reunion
    python3 -m robcoop_lab.kinsim --topologia cadena -k 0.2 --plot
"""
import argparse
import math

import numpy as np

from . import consensus_law, formations, graphs
from .kinematics import (D_CONTROL, V_MAX, control_point, integrator_to_unicycle,
                         saturate)
from .arena import START_POSES
from .safety import SECTOR, STOP_DIST


def simulate(topology='completo', formation='reunion', k=0.1, n=4, poses=None,
             law=None, dt=0.05, t_max=150.0, tol=0.05, hold=3.0, safety=True):
    """Simula el equipo y devuelve un diccionario con trayectorias y métricas.

    law: módulo con consensus_velocity (por defecto, el del alumno).
    """
    law = law or consensus_law
    poses = np.array(poses or START_POSES[:n], dtype=float)
    A = graphs.adjacency(topology, n)
    delta = np.array(formations.offsets(formation, n))
    nb = [np.nonzero(A[i])[0] for i in range(n)]

    t, t_conv, stable = 0.0, None, 0.0
    T, E, P, min_dist, blocked_steps = [], [], [], math.inf, 0
    while t <= t_max:
        q = np.array([control_point(*p) for p in poses])
        xi = q - delta
        err = float(np.max(np.linalg.norm(xi - xi.mean(axis=0), axis=1)))
        T.append(t)
        E.append(err)
        P.append(poses.copy())
        dists = [np.linalg.norm(poses[i, :2] - poses[j, :2])
                 for i in range(n) for j in range(i + 1, n)]
        min_dist = min(min_dist, min(dists))
        if err < tol:
            stable += dt
            if t_conv is None:
                t_conv = t
            if stable >= hold:
                break
        else:
            stable, t_conv = 0.0, None

        new = poses.copy()
        for i in range(n):
            u = np.asarray(law.consensus_velocity(xi[i], [xi[j] for j in nb[i]], k), float)
            ux, uy = saturate(u[0], u[1], V_MAX)
            v, w = integrator_to_unicycle(ux, uy, poses[i, 2], D_CONTROL)
            if safety and abs(v) > 1e-3:
                head = poses[i, 2] + (0 if v > 0 else math.pi)
                for j in range(n):
                    if j == i:
                        continue
                    dx, dy = poses[j, :2] - poses[i, :2]
                    # distancia a la superficie del otro robot (radio ~0.09 m)
                    rng = math.hypot(dx, dy) - 0.09
                    ang = (math.atan2(dy, dx) - head + math.pi) % (2 * math.pi) - math.pi
                    if rng < STOP_DIST and abs(ang) <= SECTOR:
                        v = 0.0
                        blocked_steps += 1
                        break
            x, y, th = poses[i]
            new[i] = (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt)
        poses = new
        t += dt
    return {'t': np.array(T), 'error': np.array(E), 'poses': np.array(P),
            't_conv': t_conv, 'converged': t_conv is not None and stable >= hold,
            'final_error': E[-1], 'min_dist': min_dist, 'blocked_steps': blocked_steps,
            'lambda2': graphs.lambda2_reference(A), 'offsets': delta}


def main():
    p = argparse.ArgumentParser(description='Simulación cinemática del consenso (sin ROS)')
    p.add_argument('--topologia', default='completo', choices=graphs.TOPOLOGIES)
    p.add_argument('--formacion', default='reunion', choices=formations.FORMATIONS)
    p.add_argument('-k', type=float, default=0.1, help='ganancia (por defecto 0.1)')
    p.add_argument('-n', type=int, default=4, help='número de robots (3 o 4)')
    p.add_argument('--plot', action='store_true', help='mostrar gráficas')
    a = p.parse_args()

    r = simulate(a.topologia, a.formacion, a.k, a.n)
    tc = f'{r["t_conv"]:.1f} s' if r['converged'] else 'NO CONVERGE'
    print(f'{a.topologia:9s} {a.formacion:9s} k={a.k:<5} λ2={r["lambda2"]:.3f}  '
          f't_conv={tc}  error final={r["final_error"]:.3f} m  '
          f'distancia mínima={r["min_dist"]:.2f} m')
    if a.plot:
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        for i in range(a.n):
            ax1.plot(r['poses'][:, i, 0], r['poses'][:, i, 1], label=f'tb3_{i}')
            ax1.plot(*r['poses'][0, i, :2], 'o', color=ax1.lines[-1].get_color())
        ax1.set_aspect('equal')
        ax1.grid(True)
        ax1.legend()
        ax1.set_title('Trayectorias (kinsim)')
        ax2.semilogy(r['t'], r['error'])
        ax2.axhline(0.05, ls='--', c='gray')
        ax2.set_xlabel('t (s)')
        ax2.set_ylabel('error de consenso (m)')
        ax2.grid(True, which='both')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
