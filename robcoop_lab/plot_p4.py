"""Gráficas del reto final a partir de los CSV/JSON del árbitro (proporcionado).

Una misión  -> una arena por fase (formación, inspección, cobertura, avería) con las
               trayectorias, y debajo la línea de tiempo de la misión; además, el marcador:
    ros2 run robcoop_lab plot_p4 resultados/p4/sim_reto.csv

Varias      -> tabla comparativa (Markdown) y figura con el tiempo de cada fase:
    ros2 run robcoop_lab plot_p4 resultados/p4/sim_reto.csv resultados/p4/real_reto_1.csv \
        --nombre sim_vs_real

Las figuras (PNG) y las tablas (.md) se guardan junto a los CSV.
"""
import argparse
import csv
import json
import os

import matplotlib
import numpy as np

from . import coverage_core as core
from . import formations
from . import mission_core as mc
from .kinematics import D_CONTROL
from .plot_p3 import COLORS, draw_arena

PHASE_COLORS = {'formacion': '#4C78A8', 'inspeccion': '#F58518', 'cobertura': '#54A24B',
                'averia': '#E45756'}
# fases del CSV que pertenecen a cada fase de la misión
CSV_PHASES = {'formacion': ('formacion',), 'inspeccion': ('subasta', 'confirmacion', 'inspeccion'),
              'cobertura': ('cobertura',), 'averia': ('averia',)}


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    cols = {h: i for i, h in enumerate(header)}
    meta_path = os.path.splitext(path)[0] + '.json'
    if not os.path.exists(meta_path):
        raise SystemExit(f'Falta {meta_path}: hace falta el JSON que se guarda junto al CSV')
    meta = json.load(open(meta_path))
    robots = meta['robots']

    def col(name):
        return np.array([float(r[cols[name]]) if r[cols[name]] != '' else np.nan for r in data])
    xy, alive = {}, {}
    for rb in robots:                   # se dibuja el PUNTO DE CONTROL, como en las prácticas
        x, y, th = col(f'{rb}_x'), col(f'{rb}_y'), col(f'{rb}_theta')
        xy[rb] = np.column_stack([x + D_CONTROL * np.cos(th), y + D_CONTROL * np.sin(th)])
        alive[rb] = col(f'{rb}_activo') > 0.5
    points, cell_area = core.make_grid(meta.get('limit', core.LIMIT), meta.get('res', core.RES))
    weights = core.density(meta.get('density', 'uniforme'), points,
                           tuple(meta.get('center', core.ZONE_CENTER)),
                           meta.get('sigma', core.ZONE_SIGMA), meta.get('peak', core.ZONE_PEAK))
    return {'path': path, 'meta': meta, 'robots': robots, 't': col('t'),
            'phase': np.array([r[cols['fase']] for r in data]), 'e_form': col('e_formacion'),
            'done': col('puntos'), 'cost': col('coste'), 'e_cov': col('e_cobertura'),
            'xy': xy, 'alive': alive, 'points': points, 'weights': weights}


def _span(run, name):
    """Índices (primero, último) de las filas de una fase, o None si no se jugó."""
    idx = np.nonzero(np.isin(run['phase'], CSV_PHASES[name]))[0]
    return (int(idx[0]), int(idx[-1])) if len(idx) else None


def _trajectories(ax, run, i0, i1):
    for i, rb in enumerate(run['robots']):
        c = COLORS[i % len(COLORS)]
        tr = run['xy'][rb][i0:i1 + 1]
        ax.plot(tr[:, 0], tr[:, 1], color=c, lw=1.8, label=rb)
        ax.plot(tr[0, 0], tr[0, 1], 'o', color=c, ms=8, mfc='white', mew=2)
        ax.plot(tr[-1, 0], tr[-1, 1], 'o', color=c, ms=9)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.grid(True, alpha=0.3)


def plot_single(run, out):
    import matplotlib.pyplot as plt
    m, robots, t = run['meta'], run['robots'], run['t']
    ph, board = m['phases'], m['scoreboard']
    cfg = m['config']
    fig = plt.figure(figsize=(17, 7.6))
    gs = fig.add_gridspec(2, 4, height_ratios=[2.5, 1.0])
    for k, name in enumerate(mc.PHASES):
        ax = fig.add_subplot(gs[0, k])
        span = _span(run, name)
        title = f'{mc.TITLES[name]} · {board["points"][name]:.0f}/{mc.POINTS[name]} puntos\n' \
                f'{mc.phase_note(name, ph[name])}'
        if span is None:
            ax.text(0.5, 0.5, 'no jugada', ha='center', va='center', transform=ax.transAxes,
                    color='#888')
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.set_aspect('equal')
            ax.set_title(title, fontsize=9)
            continue
        i0, i1 = span
        if name == 'formacion':
            _trajectories(ax, run, i0, i1)
            delta = np.array(formations.offsets(cfg['formation'], len(robots)))
            end = np.array([run['xy'][r][i1] for r in robots])
            center = (end - delta).mean(axis=0)
            for i in range(len(robots)):            # dónde tenía que acabar cada robot
                ax.plot(*(center + delta[i]), 's', color=COLORS[i % len(COLORS)], ms=13,
                        mfc='none', mew=1.5)
            ax.legend(loc='upper right', fontsize=8)
            ax.set_ylabel('y (m)')
        elif name == 'inspeccion':
            _trajectories(ax, run, i0, i1)
            owner = {tk: r for r, route in ph[name].get('routes', {}).items() for tk in route}
            reached = ph[name].get('reached', {})
            for tk, (x, y) in m['tasks'].items():
                c = COLORS[robots.index(owner[tk]) % len(COLORS)] if tk in owner else '#666'
                ax.plot(x, y, 's', color=c, ms=11, mfc=c if tk in reached else 'none', mew=1.8,
                        alpha=0.55)
                ax.annotate(tk, (x, y), xytext=(6, 6), textcoords='offset points', fontsize=8)
        else:
            sub = {r: run['xy'][r][i0:i1 + 1] for r in robots}
            act = [r for r in robots if run['alive'][r][i1]]
            draw_arena(ax, robots, sub, act, run['points'], run['weights'])
            ax.set_ylabel('')
            if ax.get_legend():
                ax.get_legend().remove()
        ax.set_title(title, fontsize=9)

    ax = fig.add_subplot(gs[1, :])              # línea de tiempo
    ax2 = ax.twinx()
    for name in mc.PHASES:
        span = _span(run, name)
        if span is None:
            continue
        i0, i1 = span
        ax.axvspan(t[i0], t[i1], color=PHASE_COLORS[name], alpha=0.10)
        ax.text((t[i0] + t[i1]) / 2, 1.03, mc.TITLES[name], ha='center', va='bottom',
                transform=ax.get_xaxis_transform(), fontsize=9, color=PHASE_COLORS[name])
        sl = slice(i0, i1 + 1)
        if name == 'formacion':
            ax.plot(t[sl], 100 * run['e_form'][sl], color=PHASE_COLORS[name], lw=1.8)
        elif name == 'inspeccion':
            ax2.step(t[sl], run['done'][sl], where='post', color=PHASE_COLORS[name], lw=1.8)
        else:
            ax.plot(t[sl], 100 * run['e_cov'][sl], color=PHASE_COLORS[name], lw=1.8)
            ax2.plot(t[sl], run['cost'][sl], color='#333', lw=1.2, ls='--')
    ax.set_yscale('log')
    ax.set_ylim(0.5, 300)
    ax.axhline(100 * mc.TOL_FORMATION, color=PHASE_COLORS['formacion'], ls=':', lw=1)
    ax.axhline(100 * core.TOL, color=PHASE_COLORS['cobertura'], ls=':', lw=1)
    ax.set_xlabel('tiempo de misión (s), pausas incluidas')
    ax.set_ylabel('error (cm): formación y cobertura')
    ax2.set_ylabel('puntos visitados · H en m⁴ (- -)')
    ax.grid(True, which='major', alpha=0.3)
    fig.suptitle(f'{m.get("name", "")} ({m.get("mode")}) · {mc.describe(cfg)}\n'
                 f'MARCADOR {board["total"]:.0f}/100 · tiempo de misión {board["time"]:.1f} s'
                 + ('' if board['complete'] else ' (incompleta)'), fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def scoreboard_md(run):
    """Marcador de una misión en Markdown."""
    m = run['meta']
    ph, board = m['phases'], m['scoreboard']
    lines = [f'Marcador de {os.path.basename(run["path"])} ({m.get("mode")}, {m.get("date")}):',
             '', f'Configuración: {mc.describe(m["config"])}', '',
             '| fase | puntos | resultado |', '|---|---|---|']
    for name in mc.PHASES:
        lines.append(f'| {mc.TITLES[name]} | {board["points"][name]:.1f} / {mc.POINTS[name]} | '
                     f'{mc.phase_note(name, ph[name])} |')
    md = m.get('min_dist')
    lines.append(f'| Seguridad | {board["points"]["seguridad"]:.1f} / {mc.POINTS["seguridad"]} | '
                 + (f'distancia mínima entre robots {md:.2f} m' if md is not None else '')
                 + (' · parada de emergencia' if m.get('emergency') else '') + ' |')
    lines.append(f'| **Total** | **{board["total"]:.1f} / 100** | tiempo de misión '
                 f'{board["time"]:.1f} s' + ('' if board['complete'] else ' (incompleta)') + ' |')
    return '\n'.join(lines)


def table(runs):
    """Tabla comparativa de varias misiones (Markdown)."""
    lines = ['| misión | modo | puntos | avería | formación (s) | inspección (s) | '
             'cobertura (s) | avería (s) | tiempo (s) | marcador |', '|' + '---|' * 10]
    for run in runs:
        m = run['meta']
        ph, board = m['phases'], m['scoreboard']

        def tt(name):
            f = ph[name]
            if not f.get('played'):
                return '—'
            return f'{f["t"]:.1f}' if f.get('ok') and f.get('t') is not None else 'NO'
        lines.append(f'| {os.path.basename(run["path"])} | {m.get("mode")} | {m.get("layout")} | '
                     f'{ph["averia"].get("robot") or "—"} | {tt("formacion")} | '
                     f'{tt("inspeccion")} ({ph["inspeccion"].get("done", 0)}/'
                     f'{ph["inspeccion"].get("total", 0)}) | {tt("cobertura")} | {tt("averia")} | '
                     f'{board["time"]:.1f}' + ('' if board['complete'] else ' (incompleta)')
                     + f' | {board["total"]:.0f} |')
    return '\n'.join(lines)


def plot_compare(runs, out):
    """Barras apiladas: lo que dura cada fase en cada misión."""
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 1.2 + 0.7 * len(runs)))
    labels = []
    for k, run in enumerate(runs):
        m = run['meta']
        left = 0.0
        for name in mc.PHASES:
            f = m['phases'][name]
            if not f.get('played'):
                continue
            ok = f.get('ok') and f.get('t') is not None
            w = f['t'] if ok else mc.T_LIMIT[name]
            ax.barh(k, w, left=left, color=PHASE_COLORS[name], alpha=0.9 if ok else 0.35,
                    hatch=None if ok else '//', edgecolor='white',
                    label=mc.TITLES[name] if k == 0 else None)
            ax.text(left + w / 2, k, f'{w:.0f}' if ok else 'NO', ha='center', va='center',
                    fontsize=8, color='white' if ok else '#333')
            left += w
        ax.text(left + 1, k, f'{m["scoreboard"]["total"]:.0f} puntos', va='center', fontsize=9)
        labels.append(f'{m.get("name", os.path.basename(run["path"]))}\n({m.get("mode")})')
    ax.set_yticks(range(len(runs)))
    ax.set_yticklabels(labels, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel('tiempo de misión (s)')
    ax.margins(x=0.12)
    ax.legend(ncol=4, fontsize=8, loc='lower center', bbox_to_anchor=(0.5, 1.0))
    ax.grid(True, axis='x', alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main():
    p = argparse.ArgumentParser(description='Gráficas del reto final')
    p.add_argument('csv', nargs='+', help='uno o varios CSV de resultados/p4')
    p.add_argument('--nombre', default='comparacion_p4', help='nombre de la comparación')
    p.add_argument('--show', action='store_true', help='abrir las figuras en pantalla')
    a = p.parse_args()
    if not a.show:
        matplotlib.use('Agg')

    runs = [load(c) for c in a.csv]
    for run in runs:
        base = os.path.splitext(run['path'])[0]
        print('Figura:', plot_single(run, base + '.png'))
        md = scoreboard_md(run)
        with open(base + '_marcador.md', 'w') as f:
            f.write(md + '\n')
        print('Marcador:', base + '_marcador.md')
        print()
        print(md)
        print()
    if len(runs) > 1:
        base = os.path.join(os.path.dirname(runs[0]['path']), a.nombre)
        print('Comparación:', plot_compare(runs, base + '.png'))
        tb = table(runs)
        with open(base + '.md', 'w') as f:
            f.write(tb + '\n')
        print('Tabla:', base + '.md')
        print()
        print(tb)
    if a.show:
        import matplotlib.pyplot as plt
        for run in runs:
            img = plt.imread(os.path.splitext(run['path'])[0] + '.png')
            plt.figure(figsize=(15, 6.7))
            plt.imshow(img)
            plt.axis('off')
        plt.show()


if __name__ == '__main__':
    main()
