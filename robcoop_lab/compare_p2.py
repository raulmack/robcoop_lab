"""Comparación de métodos de asignación, sin ROS (proporcionado).

Para cada disposición de puntos y con los robots en sus marcas de salida, calcula con VUESTRO
auction_law.py:
    subasta (suma), subasta (max), húngaro por rondas,
y como referencia el óptimo por fuerza bruta (suma y max). Para cada asignación da la
distancia total, la ruta más larga, los mensajes y el tiempo hasta completar todas las
tareas simulando la ejecución con kinsim_p2 (mismo controlador que los robots).

    ros2 run robcoop_lab compare_p2                       # disposiciones A, B y C
    ros2 run robcoop_lab compare_p2 --disposiciones A aleatoria --semilla 5
    ros2 run robcoop_lab compare_p2 -n 4                  # con 4 robots

Guarda resultados/p2/<nombre>.csv, .md y .png (rutas de cada método).
"""
import argparse
import csv
import os

import matplotlib

from . import auction_core, auction_law, kinsim_p2, tasks as tasks_mod
from .arena import START_POSES
from .kinematics import control_point
from .plot_p2 import COLORS
from .tasks import default_out_dir


def run(layouts, n=3, seed=0, law=None, execute=True):
    law = law or auction_law
    robots = [f'tb3_{i}' for i in range(n)]
    poses = START_POSES[:n]
    starts = {r: control_point(*p) for r, p in zip(robots, poses)}
    results = []
    for lay in layouts:
        task_xy = tasks_mod.get(lay, seed=seed)
        name = f'aleatoria{seed}' if lay == 'aleatoria' else lay
        for res in auction_core.compare(starts, task_xy, law):
            res = dict(res)
            res['layout'] = name
            res['tasks'] = task_xy
            res['starts'] = starts
            if execute:
                ex = kinsim_p2.execute(poses, task_xy, [res['routes'][r] for r in robots])
                res['makespan'] = ex['makespan']
                res['completed'] = ex['completed']
            results.append(res)
    return results


def table(results):
    lines = ['| disposición | método | distancia total (m) | ruta más larga (m) | '
             'tiempo total sim. (s) | mensajes |', '|---|---|---|---|---|---|']
    for r in results:
        mk = r.get('makespan')
        mk = f'{mk:.1f}' if mk is not None else ('—' if 'makespan' not in r else 'no termina')
        lines.append(f'| {r["layout"]} | {r["method"]} | {r["sum"]:.2f} | {r["max"]:.2f} | '
                     f'{mk} | {r["messages"] if r["messages"] is not None else "—"} |')
    return '\n'.join(lines)


def plot(results, out):
    import matplotlib.pyplot as plt
    layouts = list(dict.fromkeys(r['layout'] for r in results))
    methods = list(dict.fromkeys(r['method'] for r in results))
    fig, axes = plt.subplots(len(layouts), len(methods),
                             figsize=(3.2 * len(methods), 3.3 * len(layouts)), squeeze=False)
    for r in results:
        ax = axes[layouts.index(r['layout'])][methods.index(r['method'])]
        robots = sorted(r['starts'])
        for i, rb in enumerate(robots):
            c = COLORS[i % len(COLORS)]
            pts = [r['starts'][rb]] + [r['tasks'][t] for t in r['routes'][rb]]
            xs, ys = zip(*pts)
            ax.plot(xs, ys, '-', color=c, lw=1.6)
            ax.plot(xs[0], ys[0], 'o', color=c, ms=6)
        for tname, (x, y) in r['tasks'].items():
            ax.plot(x, y, 's', color='#333', ms=6, mfc='white')
            ax.annotate(tname, (x, y), xytext=(4, 4), textcoords='offset points', fontsize=7)
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f'{r["layout"]} · {r["method"]}\nΣ={r["sum"]:.2f} m  máx={r["max"]:.2f} m',
                     fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main():
    p = argparse.ArgumentParser(description='Compara subasta, húngaro y óptimo (sin ROS)')
    p.add_argument('--disposiciones', nargs='+', default=['A', 'B', 'C'],
                   choices=tasks_mod.NAMES)
    p.add_argument('--semilla', type=int, default=0, help='para la disposición aleatoria')
    p.add_argument('-n', type=int, default=3, help='número de robots (2 a 4)')
    p.add_argument('--sin-ejecucion', action='store_true',
                   help='no simular la ejecución (más rápido, sin tiempo total)')
    p.add_argument('--nombre', default='comparacion_metodos')
    p.add_argument('--out-dir', default='')
    a = p.parse_args()
    matplotlib.use('Agg')

    results = run(a.disposiciones, a.n, a.semilla, execute=not a.sin_ejecucion)
    out_dir = a.out_dir or default_out_dir()
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, a.nombre)
    md = table(results)
    print(md)
    with open(base + '.md', 'w') as f:
        f.write(md + '\n')
    with open(base + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['disposicion', 'metodo', 'distancia_total', 'ruta_mas_larga',
                    'tiempo_total_sim', 'mensajes'] + [f'ruta_{r}' for r in sorted(
                        results[0]['routes'])])
        for r in results:
            w.writerow([r['layout'], r['method'], r['sum'], r['max'], r.get('makespan'),
                        r['messages']] + ['-'.join(r['routes'][rb]) for rb in
                                          sorted(r['routes'])])
    print(f'\nTabla: {base}.md   Datos: {base}.csv   Figura: {plot(results, base + ".png")}')


if __name__ == '__main__':
    main()
