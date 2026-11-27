"""Simulador cinemático del reto final, sin ROS ni Gazebo (proporcionado).

Juega la misión completa en unos segundos con VUESTRAS tres leyes y VUESTRO reto_config.py:
los mismos controladores, la misma cinemática, un LiDAR simplificado (cada robot ve a los
demás) y el mismo filtro de seguridad. Puntúa con las mismas reglas que el árbitro de ROS
(mission_core.py). Es lo que usa check_p4.

    ros2 run robcoop_lab kinsim_p4                       # disposición A, avería de un robot al azar
    ros2 run robcoop_lab kinsim_p4 --disposicion aleatoria --semilla 7 --averia tb3_2 --plot
    python3 -m robcoop_lab.kinsim_p4 --disposicion C --averia ninguna
"""
import argparse
import math
import time

import numpy as np

from . import (auction_core, auction_law, consensus_law, coverage_core as core, coverage_law,
               kinsim, kinsim_p2, mission_core as mc, tasks as tasks_mod)
from .arena import START_POSES
from .kinematics import control_point
from .safety import SECTOR, STOP_DIST


def _deploy(poses, alive, points, weights, cell_area, k, law, t_max, dt):
    """Despliegue de cobertura de los robots vivos hasta que se colocan o se agota t_max.

    Devuelve (poses finales, hecho, t, H inicial, H final, distancia mínima, trayectoria).
    """
    poses = [list(p) for p in poses]
    n = len(poses)
    act = [i for i in range(n) if alive[i]]
    t, stable, done, t_done = 0.0, 0.0, False, None
    min_d, traj = math.inf, []
    h0 = core.ref_cost(points, weights, [control_point(*poses[i]) for i in act], cell_area)
    while t <= t_max:
        q = [np.array(control_point(*p)) for p in poses]
        traj.append([p[:] for p in poses])
        for i in range(n):
            for j in range(i + 1, n):
                min_d = min(min_d, math.dist(poses[i][:2], poses[j][:2]))
        err = core.ref_error(points, weights, [q[i] for i in act])
        stable = stable + dt if err < core.TOL else 0.0
        if stable >= core.HOLD:
            done, t_done = True, round(t - stable + dt, 2)
            break
        new = [p[:] for p in poses]
        for i in act:
            v, w, _, _ = core.robot_command(law, points, weights, poses[i],
                                            [q[j] for j in act if j != i], k)
            if abs(v) > 1e-3:                       # filtro de seguridad
                center = 0.0 if v > 0 else math.pi
                for ox, oy in kinsim_p2._scan_points(i, poses):
                    a = (math.atan2(oy, ox) - center + math.pi) % (2 * math.pi) - math.pi
                    if math.hypot(ox, oy) < STOP_DIST and abs(a) <= SECTOR:
                        v = 0.0
                        break
            x, y, th = poses[i]
            new[i] = [x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt]
        poses = new
        t += dt
    h_end = core.ref_cost(points, weights, [control_point(*poses[i]) for i in act], cell_area)
    return poses, done, t_done, h0, h_end, min_d, traj


def simulate(cfg=None, layout='A', points='', seed=0, n=3, failure='auto', num_points=6,
             density='uniforme', center=core.ZONE_CENTER, sigma=core.ZONE_SIGMA,
             peak=core.ZONE_PEAK, laws=None, fail_seed=None, dt=0.05):
    """Juega una misión y devuelve un diccionario con las fases, el marcador y las trayectorias.

    cfg:     configuración (por defecto, vuestro reto_config.py).
    laws:    (consensus_law, auction_law, coverage_law); por defecto, los vuestros.
    failure: 'auto' | 'ninguna' | nombre del robot que se avería en la fase 4.
    """
    cfg = cfg or mc.load_config(n=n)
    claw, alaw, vlaw = laws or (consensus_law, auction_law, coverage_law)
    robots = [f'tb3_{i}' for i in range(n)]
    task_xy = tasks_mod.get(layout, points, seed, num_points)
    phases, segments = {}, []
    min_d = math.inf

    # --- Fase 1 · formación ------------------------------------------------------------------
    r1 = kinsim.simulate(cfg['topology'], cfg['formation'], cfg['k_consensus'], n,
                         poses=START_POSES[:n], law=claw, dt=dt,
                         t_max=mc.T_LIMIT['formacion'], tol=mc.TOL_FORMATION,
                         hold=mc.HOLD_FORMATION)
    phases['formacion'] = {'played': True, 'ok': bool(r1['converged']),
                           't': round(float(r1['t_conv']), 2) if r1['converged'] else None,
                           'e0': float(r1['error'][0]), 'e_end': float(r1['final_error'])}
    min_d = min(min_d, float(r1['min_dist']))
    segments.append(('formacion', np.asarray(r1['poses'])))
    poses = [list(p) for p in r1['poses'][-1]]

    # --- Fase 2 · subasta e inspección -------------------------------------------------------
    starts = {r: control_point(*p) for r, p in zip(robots, poses)}
    f2 = {'played': True, 'ok': False, 't': None, 'done': 0, 'total': len(task_xy)}
    routes = {r: [] for r in robots}
    try:
        auc = auction_core.ssi(starts, task_xy, cfg['criterion'], alaw)
        routes = auc['routes']
        f2.update({'planned_sum': auc['sum'], 'planned_max': auc['max']})
        ex = kinsim_p2.execute(poses, task_xy, [routes[r] for r in robots], dt=dt,
                               t_max=mc.T_LIMIT['inspeccion'])
        f2.update({'done': ex['completed'], 'ok': ex['completed'] == len(task_xy),
                   't': round(max(t for _, t in ex['reached'].values()), 2)
                   if ex['completed'] == len(task_xy) else None})
        min_d = min(min_d, ex['min_dist'])
        segments.append(('inspeccion', np.asarray(ex['poses'])))
        poses = [list(p) for p in ex['poses'][-1]]
    except Exception as e:  # noqa: BLE001  (p. ej. auction_law sin programar)
        f2['error'] = repr(e)
    phases['inspeccion'] = f2

    # --- Fases 3 y 4 · cobertura y avería ----------------------------------------------------
    grid, cell_area = core.make_grid()
    weights = core.density(density, grid, center, sigma, peak)
    alive = [True] * n
    q0 = [control_point(*p) for p in poses]
    poses, ok, t3, h0, h1, d3, traj = _deploy(poses, alive, grid, weights, cell_area,
                                              cfg['k_coverage'], vlaw, mc.T_LIMIT['cobertura'],
                                              dt)
    min_d = min(min_d, d3)
    segments.append(('cobertura', np.asarray(traj)))
    phases['cobertura'] = {'played': True, 'ok': ok, 't': t3, 'h0': round(h0, 4),
                           'h_end': round(h1, 4),
                           'h_lloyd': round(core.lloyd(q0, grid, weights, cell_area)[1][-1], 4)}

    failed = mc.pick_failure(failure, robots, fail_seed)
    f4 = {'played': False, 'ok': False, 't': None, 'robot': failed}
    if failed is not None:
        alive[robots.index(failed)] = False
        rest = [control_point(*poses[i]) for i in range(n) if alive[i]]
        lloyd4 = core.lloyd(rest, grid, weights, cell_area)[1][-1]
        poses, ok, t4, h0, h1, d4, traj = _deploy(poses, alive, grid, weights, cell_area,
                                                  cfg['k_coverage'], vlaw, mc.T_LIMIT['averia'],
                                                  dt)
        min_d = min(min_d, d4)
        segments.append(('averia', np.asarray(traj)))
        f4.update({'played': True, 'ok': ok, 't': t4, 'h0': round(h0, 4), 'h_end': round(h1, 4),
                   'h_lloyd': round(lloyd4, 4)})
    phases['averia'] = f4

    board = mc.scoreboard(phases, min_d)
    return {'config': cfg, 'robots': robots, 'tasks': task_xy, 'routes': routes,
            'phases': phases, 'scoreboard': board, 'min_dist': min_d,
            'collision': min_d < 2 * kinsim_p2.R_BODY, 'failed': failed, 'alive': alive,
            'segments': segments, 'final_poses': poses, 'grid': grid, 'weights': weights}


def plot(r, title=''):
    """Una arena por fase con las trayectorias (se dibuja el eje de las ruedas)."""
    import matplotlib.pyplot as plt
    from .plot_p3 import COLORS
    segs = r['segments']
    fig, axes = plt.subplots(1, len(segs), figsize=(4.3 * len(segs), 4.6), squeeze=False)
    for ax, (name, P) in zip(axes[0], segs):
        for i, rb in enumerate(r['robots']):
            c = COLORS[i % len(COLORS)]
            ax.plot(P[:, i, 0], P[:, i, 1], color=c, lw=1.8, label=rb)
            ax.plot(P[0, i, 0], P[0, i, 1], 'o', color=c, mfc='white', ms=7)
            dead = name == 'averia' and rb == r['failed']
            ax.plot(P[-1, i, 0], P[-1, i, 1], 'X' if dead else 'o', color=c, ms=10 if dead else 7)
        if name == 'inspeccion':
            for tname, (x, y) in r['tasks'].items():
                ax.plot(x, y, 's', color='#555', ms=9, mfc='none')
                ax.annotate(tname, (x, y), xytext=(5, 5), textcoords='offset points', fontsize=8)
        if name in ('cobertura', 'averia'):
            L = core.LIMIT
            ax.plot([-L, L, L, -L, -L], [-L, -L, L, L, -L], color='#888', lw=1, ls='--')
        f = r['phases'][name]
        ax.set_title(f'{mc.TITLES[name]}\n{mc.phase_note(name, f)}', fontsize=9)
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.4)
    axes[0][0].legend(fontsize=8, loc='lower right')
    b = r['scoreboard']
    fig.suptitle(f'{title}   marcador {b["total"]:.0f}/100 · tiempo {b["time"]:.1f} s',
                 fontsize=11)
    fig.tight_layout()
    return fig


def main():
    p = argparse.ArgumentParser(description='Reto final: misión completa sin ROS')
    p.add_argument('--disposicion', default='A', choices=tasks_mod.NAMES)
    p.add_argument('--puntos', default='', help='"x,y; x,y; ..." (sustituye a la disposición)')
    p.add_argument('--semilla', type=int, default=0, help='para la disposición aleatoria')
    p.add_argument('--averia', default='auto',
                   help='auto (un robot al azar) | ninguna | tb3_0, tb3_1...')
    p.add_argument('--densidad', default='uniforme', choices=core.DENSITIES)
    p.add_argument('-n', type=int, default=3, help='número de robots (3 o 4)')
    p.add_argument('--plot', action='store_true', help='mostrar las trayectorias de cada fase')
    a = p.parse_args()
    if a.averia == 'profesor':
        p.error('sin ROS no hay profesor: usad auto, ninguna o el nombre de un robot')
    try:
        cfg = mc.load_config(n=a.n)
    except ValueError as e:
        raise SystemExit(str(e))
    t0 = time.time()
    r = simulate(cfg, a.disposicion, a.puntos, a.semilla, a.n, a.averia, density=a.densidad)
    lay = 'puntos propios' if a.puntos.strip() else a.disposicion + \
        (f' (semilla {a.semilla})' if a.disposicion == 'aleatoria' else '')
    print(f'Reto final · {a.n} robots · puntos: {lay} · avería: {r["failed"] or "ninguna"}')
    print(f'  configuración: {mc.describe(cfg)}')
    for rb in r['robots']:
        print(f'  {rb}: {" -> ".join(r["routes"][rb]) or "(sin puntos)"}')
    if 'error' in r['phases']['inspeccion']:
        print(f'  la subasta ha fallado: {r["phases"]["inspeccion"]["error"]}')
    print(mc.format_scoreboard(r['phases'], r['scoreboard'], r['min_dist']))
    print(f'  (simulado en {time.time() - t0:.1f} s)')
    if a.plot:
        import matplotlib.pyplot as plt
        plot(r, f'kinsim_p4 · {lay}')
        plt.show()


if __name__ == '__main__':
    main()
