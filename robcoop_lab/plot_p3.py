"""Gráficas de la Práctica 3 a partir de los CSV/JSON del registro (proporcionado).

Un experimento -> arena con las celdas de Voronoi finales y las trayectorias + coste H(t) y
                  error e(t). Si hubo avería, también la arena justo antes de la avería:
    ros2 run robcoop_lab plot_p3 resultados/p3/sim_marcas_uniforme_....csv

Varios         -> tabla resumen (Markdown) y figura comparativa de H(t) y e(t):
    ros2 run robcoop_lab plot_p3 resultados/p3/sim_k*.csv --nombre ganancias

Las figuras se guardan en PNG junto a los CSV (listas para el informe).
"""
import argparse
import csv
import json
import math
import os

import matplotlib
import numpy as np

from . import coverage_core as core
from .kinematics import D_CONTROL

COLORS = ['#e6194b', '#3cb44b', '#0082c8', '#f58231', '#911eb4', '#46f0f0']


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    cols = {h: i for i, h in enumerate(header)}
    meta_path = os.path.splitext(path)[0] + '.json'
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    robots = meta.get('robots') or [h[:-2] for h in header if h.endswith('_x')]

    def col(name):
        return np.array([float(r[cols[name]]) if r[cols[name]] != '' else np.nan for r in data])
    # se dibuja el PUNTO DE CONTROL (10 cm delante del eje), que es el que va al centroide
    xy, alive = {}, {}
    for rb in robots:
        x, y, th = col(f'{rb}_x'), col(f'{rb}_y'), col(f'{rb}_theta')
        xy[rb] = np.column_stack([x + D_CONTROL * np.cos(th), y + D_CONTROL * np.sin(th)])
        alive[rb] = col(f'{rb}_activo') > 0.5
    points, cell_area = core.make_grid(meta.get('limit', core.LIMIT), meta.get('res', core.RES))
    weights = core.density(meta.get('density', 'uniforme'), points,
                           tuple(meta.get('center', core.ZONE_CENTER)),
                           meta.get('sigma', core.ZONE_SIGMA), meta.get('peak', core.ZONE_PEAK))
    return {'path': path, 'meta': meta, 'robots': robots, 't': col('t'), 'cost': col('coste'),
            'cost_ref': col('coste_ref'), 'error': col('error'),
            'phase': [r[cols['fase']] for r in data], 'xy': xy, 'alive': alive,
            'points': points, 'weights': weights, 'cell_area': cell_area}


def label(run):
    m = run['meta']
    txt = f'{m.get("mode", "?")} {m.get("start", "?")} {m.get("density", "?")} k={m.get("k", "?")}'
    return txt + (f' avería {m["fail"]}' if m.get('fail') else '')


def draw_arena(ax, robots, xy, active, points, weights, upto=None, title=None, limit=core.LIMIT):
    """Celdas de Voronoi de los robots activos en el instante 'upto' (por defecto, el final),
    trayectorias hasta ese instante (○ salida, ● posición, + centroide, ✕ robot averiado)."""
    from matplotlib.colors import ListedColormap
    idx = len(next(iter(xy.values()))) - 1 if upto is None else upto
    act = [r for r in robots if r in active]
    q = np.array([xy[r][idx] for r in act])
    n = int(round(math.sqrt(len(points))))
    owner = core.ref_owner(points, q).reshape(n, n)
    cmap = ListedColormap([COLORS[robots.index(r) % len(COLORS)] for r in act])
    ax.imshow(owner, origin='lower', extent=[-limit, limit, -limit, limit], cmap=cmap,
              alpha=0.28, vmin=-0.5, vmax=len(act) - 0.5, interpolation='nearest')
    if np.ptp(weights) > 1e-9:
        ax.contour(points[:, 0].reshape(n, n), points[:, 1].reshape(n, n),
                   weights.reshape(n, n), levels=4, colors='#444', linewidths=0.7, alpha=0.7)
    C, _ = core.ref_centroids(points, weights, q)
    for i, rb in enumerate(robots):
        c = COLORS[i % len(COLORS)]
        tr = xy[rb][:idx + 1]
        ax.plot(tr[:, 0], tr[:, 1], color=c, lw=1.8, label=rb)
        ax.plot(tr[0, 0], tr[0, 1], 'o', color=c, ms=8, mfc='white', mew=2)
        if rb in act:
            ax.plot(tr[-1, 0], tr[-1, 1], 'o', color=c, ms=9)
            cx, cy = C[act.index(rb)]
            ax.plot(cx, cy, '+', color='k', ms=11, mew=2)
        else:
            ax.plot(tr[-1, 0], tr[-1, 1], 'X', color='#333', ms=11)
    ax.plot([-limit, limit, limit, -limit, -limit], [-limit, -limit, limit, limit, -limit],
            color='#333', lw=1)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    if title:
        ax.set_title(title, fontsize=9)


def _log_ticks(ax):
    """Eje logarítmico del error con marcas legibles (en metros)."""
    from matplotlib.ticker import FixedLocator, NullFormatter
    ticks = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0]
    ax.yaxis.set_major_locator(FixedLocator(ticks))
    ax.yaxis.set_major_formatter(lambda v, _: f'{v:g}')
    ax.yaxis.set_minor_formatter(NullFormatter())


def plot_single(run, out):
    import matplotlib.pyplot as plt
    m = run['meta']
    robots, t = run['robots'], run['t']
    failed = m.get('t_fail') is not None
    active = m.get('active') or [r for r in robots if run['alive'][r][-1]]
    ncol = 3 if failed else 2
    fig = plt.figure(figsize=(6.2 * ncol + 0.6, 5.6))
    gs = fig.add_gridspec(2, ncol, width_ratios=[1.1] * (ncol - 1) + [1.0])
    ax = fig.add_subplot(gs[:, 0])
    args = (robots, run['xy'], robots if failed else active, run['points'], run['weights'])
    if failed:
        i_fail = int(np.searchsorted(t, m['t_fail']))
        draw_arena(ax, *args, upto=max(i_fail - 1, 0),
                   title=f'Justo antes de la avería (t = {m["t_fail"]:.1f} s)')
        ax = fig.add_subplot(gs[:, 1])
        draw_arena(ax, robots, run['xy'], active, run['points'], run['weights'],
                   title=f'Final: {m["fail"]} averiado (✕), los demás recolocados')
    else:
        draw_arena(ax, *args, title='Celdas finales, punto de control (—), ○ salida, '
                                    '● final, + centroide')

    ax1 = fig.add_subplot(gs[0, ncol - 1])
    ax1.plot(t, run['cost_ref'], color='#14213D', lw=1.8, label='H')
    bad = np.isfinite(run['cost']).all() and \
        np.max(np.abs(run['cost'] - run['cost_ref'])) > 0.01 * np.max(run['cost_ref'])
    if bad or not np.isfinite(run['cost']).all():
        ax1.plot(t, run['cost'], '--', color='#C2410C', label='vuestra coverage_cost (¡revisar!)')
    if m.get('lloyd_cost') is not None:
        ax1.axhline(m['lloyd_cost'], ls=':', color='gray',
                    label=f'Lloyd ideal = {m["lloyd_cost"]:.2f}')
    if failed and m.get('lloyd_cost_fail') is not None:
        ax1.axhline(m['lloyd_cost_fail'], ls=':', color='#C2410C',
                    label=f'Lloyd ideal sin {m["fail"]} = {m["lloyd_cost_fail"]:.2f}')
    ax1.set_ylabel('coste de cobertura H (m⁴)')
    ax1.grid(True, alpha=0.4)
    ax2 = fig.add_subplot(gs[1, ncol - 1], sharex=ax1)
    ax2.semilogy(t, np.maximum(run['error'], 1e-3), color='#14213D', lw=1.5)
    ax2.axhline(m.get('tol', core.TOL), ls='--', color='gray',
                label=f'tol = {m.get("tol", core.TOL)} m')
    _log_ticks(ax2)
    ax2.set_xlabel('t (s)')
    ax2.set_ylabel('error  max |C_i − p_i|  (m)')
    ax2.grid(True, which='both', alpha=0.4)
    for a in (ax1, ax2):
        if m.get('t_conv') is not None:
            a.axvline(m['t_conv'], color='#3cb44b', lw=1,
                      label=f'desplegados: {m["t_conv"]:.1f} s' if a is ax2 else None)
        if failed:
            a.axvline(m['t_fail'], color='#C2410C', ls='--', lw=1,
                      label=f'avería de {m["fail"]}' if a is ax2 else None)
    ax1.legend(fontsize=8)
    ax2.legend(fontsize=8)
    h0, h1 = m.get('cost_initial'), m.get('cost_final')
    extra = ''
    if failed and h0 is not None and m.get('cost_deployed') is not None:
        extra = (f' · H {h0:.2f} → {m["cost_deployed"]:.2f} → (avería) '
                 f'{m["cost_after_fail"]:.2f} → {h1:.2f} m⁴')
    elif h0 is not None and h1 is not None:
        extra = (f' · H {h0:.2f} → {h1:.2f} m⁴ · distancia media a un robot '
                 f'{m.get("rms_initial", 0):.2f} → {m.get("rms_final", 0):.2f} m')
    fig.suptitle(label(run) + extra, fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def plot_compare(runs, out):
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    for i, run in enumerate(runs):
        c = COLORS[i % len(COLORS)]
        ax1.plot(run['t'], run['cost_ref'], color=c, label=label(run))
        ax2.semilogy(run['t'], np.maximum(run['error'], 1e-3), color=c)
    ax1.set_xlabel('t (s)')
    ax1.set_ylabel('coste de cobertura H (m⁴)')
    ax1.legend(fontsize=8)
    ax2.axhline(runs[0]['meta'].get('tol', core.TOL), ls='--', color='gray')
    _log_ticks(ax2)
    ax2.set_xlabel('t (s)')
    ax2.set_ylabel('error  max |C_i − p_i|  (m)')
    for ax in (ax1, ax2):
        ax.grid(True, which='both', alpha=0.4)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def table(runs):
    lines = ['| experimento | modo | salida | densidad | k | robots | t despliegue (s) | '
             'H inicial (m⁴) | H desplegados (m⁴) | Lloyd ideal (m⁴) | dist. media (m) | '
             'recorrido total (m) | avería | H tras la avería (m⁴) | t recolocación (s) | '
             'H final (m⁴) |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    f2 = lambda v: f'{v:.2f}' if v is not None else '—'  # noqa: E731
    for run in runs:
        m = run['meta']
        tc = f'{m["t_conv"]:.1f}' if m.get('t_conv') is not None else 'no converge'
        failed = m.get('t_fail') is not None
        tr = '—'
        if failed:
            tr = f'{m["t_reconv"]:.1f}' if m.get('t_reconv') is not None else 'no converge'
        dep = m.get('cost_deployed') if failed else m.get('cost_final')
        mass = float(np.sum(run['weights'])) * run['cell_area']
        rms = f'{math.sqrt(dep / mass):.2f}' if dep is not None else '—'
        lines.append(
            f'| {os.path.basename(run["path"])} | {m.get("mode")} | {m.get("start")} | '
            f'{m.get("density")} | {m.get("k")} | {len(run["robots"])} | {tc} | '
            f'{f2(m.get("cost_initial"))} | {f2(dep)} | {f2(m.get("lloyd_cost"))} | {rms} | '
            f'{sum(m.get("travelled", {}).values()):.2f} | {m.get("fail") or "—"} | '
            f'{f2(m.get("cost_after_fail")) if failed else "—"} | {tr} | '
            f'{f2(m.get("cost_final"))} |')
    return '\n'.join(lines)


def cells_table(run):
    """Posición final y celda de cada robot (del JSON)."""
    m = run['meta']
    if not m.get('final_q'):
        return ''
    lines = [f'Celdas finales de {os.path.basename(run["path"])}:',
             '| robot | posición final (m) | área de su celda (m²) | masa de su celda | '
             'recorrido (m) |', '|---|---|---|---|---|']
    for r in run['robots']:
        x, y = m['final_q'][r]
        if r in m.get('active', []):
            lines.append(f'| {r} | ({x:+.2f}, {y:+.2f}) | {m["areas"][r]:.2f} | '
                         f'{m["masses"][r]:.2f} | {m["travelled"][r]:.2f} |')
        else:
            lines.append(f'| {r} | ({x:+.2f}, {y:+.2f}) averiado | — | — | '
                         f'{m["travelled"][r]:.2f} |')
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser(description='Gráficas de la Práctica 3')
    p.add_argument('csv', nargs='+', help='uno o varios CSV de resultados/p3')
    p.add_argument('--nombre', default='comparacion_p3', help='nombre de la figura comparativa')
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
    for run in runs:
        ct = cells_table(run)
        if ct:
            print()
            print(ct)
    if a.show:
        import matplotlib.pyplot as plt
        for path in [os.path.splitext(r['path'])[0] + '.png' for r in runs]:
            img = plt.imread(path)
            plt.figure(figsize=(13, 5.6))
            plt.imshow(img)
            plt.axis('off')
        plt.show()


if __name__ == '__main__':
    main()
