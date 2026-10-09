"""Gráficas de la Práctica 1 a partir de los CSV del registro (proporcionado).

Un experimento  -> trayectorias en la arena + error de consenso en el tiempo:
    ros2 run robcoop_lab plot_p1 resultados/p1/sim_anillo_reunion_k0.1_....csv

Varios          -> comparación del error (p. ej. las cuatro topologías) y tabla resumen:
    ros2 run robcoop_lab plot_p1 resultados/p1/sim_*_reunion_*.csv --nombre topologias

Las figuras se guardan en PNG junto a los CSV (listas para el informe).
"""
import argparse
import csv
import json
import os

import matplotlib
import numpy as np

COLORS = ['#e6194b', '#3cb44b', '#0082c8', '#f58231', '#911eb4', '#46f0f0']


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    cols = {h: i for i, h in enumerate(header)}
    meta_path = os.path.splitext(path)[0] + '.json'
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    robots = meta.get('robots') or [h[:-2] for h in header[3:] if h.endswith('_x')]
    t = np.array([float(r[cols['t']]) for r in data])
    err = np.array([float(r[cols['error']]) for r in data])
    phase = [r[cols['fase']] for r in data]
    xy = {rb: np.array([[float(r[cols[f'{rb}_x']]), float(r[cols[f'{rb}_y']])] for r in data])
          for rb in robots}
    return {'path': path, 'meta': meta, 'robots': robots, 't': t, 'error': err,
            'phase': phase, 'xy': xy}


def label(run):
    m = run['meta']
    return (f'{m.get("mode", "?")} {m.get("topology", "?")} {m.get("formation", "")} '
            f'k={m.get("k", "?")} (λ2={m.get("lambda2", float("nan")):.2f})')


def plot_single(run, out):
    import matplotlib.pyplot as plt
    m = run['meta']
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    for i, rb in enumerate(run['robots']):
        c = COLORS[i % len(COLORS)]
        xy = run['xy'][rb]
        ax1.plot(xy[:, 0], xy[:, 1], color=c, label=rb)
        ax1.plot(xy[0, 0], xy[0, 1], 'o', color=c)
        ax1.plot(xy[-1, 0], xy[-1, 1], 'x', color=c, ms=10, mew=2)
    final = np.array([run['xy'][rb][-1] for rb in run['robots']])
    if len(final) > 2:
        loop = np.vstack([final, final[:1]])
        ax1.plot(loop[:, 0], loop[:, 1], ':', color='gray')
    ax1.set_aspect('equal')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.grid(True)
    ax1.legend(loc='best', fontsize=8)
    ax1.set_title('Trayectorias en map (○ inicio, × final)')

    ax2.semilogy(run['t'], np.maximum(run['error'], 1e-4), color='#14213D')
    ax2.axhline(m.get('tol', 0.05), ls='--', color='gray', label=f'tol = {m.get("tol", 0.05)} m')
    if m.get('t_conv') is not None:
        ax2.axvline(m['t_conv'], color='#C2410C', label=f't_conv = {m["t_conv"]:.1f} s')
    if m.get('t_disturb') is not None:
        d0 = m['t_disturb']
        ax2.axvspan(d0, d0 + m.get('disturb_duration', 0), color='#F4A259', alpha=0.3,
                    label=f'perturbación {m.get("disturb")}')
    ax2.set_xlabel('t (s)')
    ax2.set_ylabel('error de consenso (m)')
    ax2.grid(True, which='both', alpha=0.4)
    ax2.legend(fontsize=8)
    ax2.set_title('Error  e(t) = max_i ‖ξ_i − media(ξ)‖')
    fig.suptitle(label(run))
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    return out


def plot_compare(runs, out):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5))
    for i, run in enumerate(sorted(runs, key=lambda r: -r['meta'].get('lambda2', 0))):
        ax.semilogy(run['t'], np.maximum(run['error'], 1e-4), color=COLORS[i % len(COLORS)],
                    label=label(run))
    ax.axhline(runs[0]['meta'].get('tol', 0.05), ls='--', color='gray')
    ax.set_xlabel('t (s)')
    ax.set_ylabel('error de consenso (m)')
    ax.grid(True, which='both', alpha=0.4)
    ax.legend(fontsize=8)
    ax.set_title('Comparación de experimentos')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    return out


def table(runs):
    lines = ['| experimento | topología | λ2 | k | t_conv (s) | error final (m) |',
             '|---|---|---|---|---|---|']
    for run in sorted(runs, key=lambda r: -r['meta'].get('lambda2', 0)):
        m = run['meta']
        tc = f'{m["t_conv"]:.1f}' if m.get('t_conv') is not None else 'no converge'
        lines.append(f'| {os.path.basename(run["path"])} | {m.get("topology")} | '
                     f'{m.get("lambda2", float("nan")):.3f} | {m.get("k")} | {tc} | '
                     f'{m.get("final_error")} |')
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser(description='Gráficas de la Práctica 1')
    p.add_argument('csv', nargs='+', help='uno o varios CSV de resultados/p1')
    p.add_argument('--nombre', default='comparacion', help='nombre de la figura comparativa')
    p.add_argument('--show', action='store_true', help='abrir las figuras en pantalla')
    a = p.parse_args()
    if not a.show:
        matplotlib.use('Agg')

    runs = [load(c) for c in a.csv]
    for run in runs:
        print('Figura:', plot_single(run, os.path.splitext(run['path'])[0] + '.png'))
    if len(runs) > 1:
        out = os.path.join(os.path.dirname(runs[0]['path']), a.nombre + '.png')
        print('Comparación:', plot_compare(runs, out))
    print()
    print(table(runs))
    if a.show:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == '__main__':
    main()
