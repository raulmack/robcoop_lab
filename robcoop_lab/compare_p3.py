"""Análisis del algoritmo de Lloyd, sin ROS (proporcionado).

Responde con VUESTRO coverage_law.py a dos preguntas del informe:

  1. ¿Llega el equipo siempre a la misma configuración? Para cada número de robots repite el
     algoritmo de Lloyd ideal desde muchas posiciones de salida al azar (y desde las marcas y
     desde 'agrupados') y compara los costes finales. Lloyd solo garantiza un mínimo LOCAL.
  2. ¿Cuánto mejora la cobertura al añadir robots? Mejor coste encontrado con 1, 2, ... robots.

    ros2 run robcoop_lab compare_p3                       # densidad uniforme, 1 a 6 robots
    ros2 run robcoop_lab compare_p3 --densidad zona --nombre comparacion_zona
    ros2 run robcoop_lab compare_p3 --salidas 40 --semilla 7

Guarda resultados/p3/<nombre>.md, .csv y .png (mejor configuración con cada número de robots,
coste frente a número de robots y costes finales de todas las salidas).
"""
import argparse
import csv
import os

import matplotlib
import numpy as np

from . import coverage_core as core
from . import coverage_law
from .kinematics import control_point
from .plot_p3 import COLORS

MAX_ROBOTS = 6


def run(density='uniforme', n_max=MAX_ROBOTS, starts=20, seed=0, law=None, **zone):
    """Lista con un diccionario por número de robots."""
    law = law or coverage_law
    points, cell_area = core.make_grid()
    weights = core.density(density, points, **zone)
    rng = np.random.default_rng(seed)
    results = []
    for n in range(1, n_max + 1):
        finals = []
        for _ in range(starts):
            P0 = rng.uniform(-core.LIMIT, core.LIMIT, size=(n, 2))
            P, costs = core.lloyd(P0, points, weights, cell_area, law)
            finals.append((costs[-1], P))
        named = {}
        for name in core.STARTS:
            if n <= len(core.STARTS[name]):
                P0 = [control_point(*p) for p in core.start_poses(name, n)]
                P, costs = core.lloyd(P0, points, weights, cell_area, law)
                named[name] = costs[-1]
                finals.append((costs[-1], P))
        best = min(finals, key=lambda f: f[0])
        costs = np.array([f[0] for f in finals[:starts]])
        results.append({
            'n': n, 'best': float(best[0]), 'best_q': best[1], 'worst': float(costs.max()),
            'costs': costs, 'named': named,
            # finales distintos: costes que difieren en más de un 0.5 % del mejor
            'distinct': len(set(np.round(costs / (0.005 * best[0])).astype(int))),
            'not_best': int(np.sum(costs > 1.005 * best[0])),
            'rms': core.rms_distance(best[0], weights, cell_area),
            'points': points, 'weights': weights})
    return results


def table(results, starts):
    f = lambda v: f'{v:.2f}' if v is not None else '—'  # noqa: E731
    lines = [f'| robots | mejor H (m⁴) | peor H (m⁴) | salidas al azar que NO llegan al mejor '
             f'(de {starts}) | dist. media con el mejor (m) | H desde marcas | '
             'H desde agrupados | N · mejor H |', '|---|---|---|---|---|---|---|---|']
    for r in results:
        lines.append(f'| {r["n"]} | {r["best"]:.2f} | {r["worst"]:.2f} | {r["not_best"]} | '
                     f'{r["rms"]:.2f} | {f(r["named"].get("marcas"))} | '
                     f'{f(r["named"].get("agrupados"))} | {r["n"] * r["best"]:.1f} |')
    return '\n'.join(lines)


def plot(results, out, density):
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    n_max = len(results)
    fig = plt.figure(figsize=(2.6 * n_max, 7.2))
    gs = fig.add_gridspec(2, n_max, height_ratios=[1, 1.25])
    L = core.LIMIT
    for j, r in enumerate(results):
        ax = fig.add_subplot(gs[0, j])
        m = int(round(np.sqrt(len(r['points']))))
        owner = core.ref_owner(r['points'], r['best_q']).reshape(m, m)
        ax.imshow(owner, origin='lower', extent=[-L, L, -L, L], alpha=0.35, vmin=-0.5,
                  vmax=r['n'] - 0.5, cmap=ListedColormap(COLORS[:r['n']]),
                  interpolation='nearest')
        if np.ptp(r['weights']) > 1e-9:
            ax.contour(r['points'][:, 0].reshape(m, m), r['points'][:, 1].reshape(m, m),
                       r['weights'].reshape(m, m), levels=4, colors='#444', linewidths=0.6)
        ax.plot(r['best_q'][:, 0], r['best_q'][:, 1], 'ko', ms=5)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f'{r["n"]} robot{"s" if r["n"] > 1 else ""}\nH = {r["best"]:.2f} m⁴',
                     fontsize=9)
    ns = [r['n'] for r in results]
    ax1 = fig.add_subplot(gs[1, :n_max // 2])
    ax1.plot(ns, [r['best'] for r in results], 'o-', color='#14213D', label='mejor H encontrado')
    ax1.plot(ns, [results[0]['best'] / n for n in ns], '--', color='gray', label='H(1) / N')
    ax1.set_xlabel('número de robots N')
    ax1.set_ylabel('coste de cobertura H (m⁴)')
    ax1.set_xticks(ns)
    ax1.grid(True, alpha=0.4)
    ax1.legend(fontsize=8)
    ax2 = fig.add_subplot(gs[1, n_max // 2:])
    rng = np.random.default_rng(0)
    for r in results:
        x = r['n'] + rng.uniform(-0.15, 0.15, size=len(r['costs']))
        ax2.plot(x, 100 * (r['costs'] / r['best'] - 1), 'o', ms=3.5, color='#0082c8', alpha=0.6,
                 label=f'{len(r["costs"])} salidas al azar' if r['n'] == ns[0] else None)
        for k, (name, c) in enumerate(r['named'].items()):
            ax2.plot(r['n'], 100 * (c / r['best'] - 1), ['s', '^'][k % 2], color=COLORS[k * 3],
                     ms=8, mfc='none', mew=2, label=f'desde {name}' if r['n'] == 1 else None)
    ax2.set_xlabel('número de robots N')
    ax2.set_ylabel('H final respecto al mejor (%)')
    ax2.set_xticks(ns)
    ax2.grid(True, alpha=0.4)
    ax2.legend(fontsize=8)
    fig.suptitle(f'Algoritmo de Lloyd ideal · densidad {density}', fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main():
    p = argparse.ArgumentParser(description='Lloyd desde muchas salidas y con 1..N robots')
    p.add_argument('--densidad', default='uniforme', choices=core.DENSITIES)
    p.add_argument('--robots', type=int, default=MAX_ROBOTS, help='hasta cuántos robots (6)')
    p.add_argument('--salidas', type=int, default=20, help='salidas al azar por caso (20)')
    p.add_argument('--semilla', type=int, default=0)
    p.add_argument('--nombre', default='comparacion_lloyd')
    p.add_argument('--out-dir', default='')
    a = p.parse_args()
    matplotlib.use('Agg')

    results = run(a.densidad, a.robots, a.salidas, a.semilla)
    out_dir = a.out_dir or core.default_out_dir()
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, a.nombre)
    md = table(results, a.salidas)
    print(md)
    with open(base + '.md', 'w') as f:
        f.write(md + '\n')
    with open(base + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['robots', 'mejor_H', 'peor_H', 'no_llegan_al_mejor', 'dist_media',
                    'H_marcas', 'H_agrupados'] + [f'H_salida_{i}' for i in range(a.salidas)])
        for r in results:
            w.writerow([r['n'], round(r['best'], 4), round(r['worst'], 4), r['not_best'],
                        round(r['rms'], 4), r['named'].get('marcas'),
                        r['named'].get('agrupados')] + [round(float(c), 4) for c in r['costs']])
    print(f'\nTabla: {base}.md   Datos: {base}.csv   '
          f'Figura: {plot(results, base + ".png", a.densidad)}')


if __name__ == '__main__':
    main()
