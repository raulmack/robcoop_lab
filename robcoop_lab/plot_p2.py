"""Gráficas de la Práctica 2 a partir de los CSV/JSON del registro (proporcionado).

Un experimento -> arena con tareas, rutas planificadas y trayectorias + cronograma de llegadas:
    ros2 run robcoop_lab plot_p2 resultados/p2/sim_A_suma_....csv

Varios         -> tabla resumen (Markdown) y figura comparativa de distancia y tiempo:
    ros2 run robcoop_lab plot_p2 resultados/p2/*.csv --nombre criterios

Las figuras se guardan en PNG junto a los CSV (listas para el informe).
"""
import argparse
import csv
import json
import os

import matplotlib
import numpy as np

from .kinematics import D_CONTROL

COLORS = ['#e6194b', '#3cb44b', '#0082c8', '#f58231', '#911eb4', '#46f0f0']


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    cols = {h: i for i, h in enumerate(header)}
    meta_path = os.path.splitext(path)[0] + '.json'
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    robots = meta.get('robots') or [h[:-2] for h in header[2:] if h.endswith('_x')]
    t = np.array([float(r[cols['t']]) for r in data])
    phase = [r[cols['fase']] for r in data]

    def col(rb, c):
        return np.array([float(r[cols[f'{rb}_{c}']]) if r[cols[f'{rb}_{c}']] != '' else np.nan
                         for r in data])
    # se dibuja el PUNTO DE CONTROL (10 cm delante del eje), que es el que llega a las tareas
    xy = {}
    for rb in robots:
        x, y, th = col(rb, 'x'), col(rb, 'y'), col(rb, 'theta')
        xy[rb] = np.column_stack([x + D_CONTROL * np.cos(th), y + D_CONTROL * np.sin(th)])
    return {'path': path, 'meta': meta, 'robots': robots, 't': t, 'phase': phase, 'xy': xy}


def label(run):
    m = run['meta']
    return f'{m.get("mode", "?")} {m.get("layout", "?")} {m.get("criterion", "?")}'


def plot_single(run, out):
    import matplotlib.pyplot as plt
    m = run['meta']
    robots = run['robots']
    tasks = m.get('tasks', {})
    routes = m.get('routes', {})
    reached = m.get('reached', {})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.6),
                                   gridspec_kw={'width_ratios': [1.15, 1]})
    # --- Arena -----------------------------------------------------------------------------
    for i, rb in enumerate(robots):
        c = COLORS[i % len(COLORS)]
        xy = run['xy'][rb]
        ax1.plot(xy[:, 0], xy[:, 1], color=c, lw=1.8, label=rb)
        ax1.plot(xy[0, 0], xy[0, 1], 'o', color=c, ms=8)
        q0 = m.get('start_q', {}).get(rb)
        if q0 and routes.get(rb):
            pts = np.array([q0] + [tasks[t] for t in routes[rb]])
            ax1.plot(pts[:, 0], pts[:, 1], '--', color=c, lw=1, alpha=0.6)
            for k, tname in enumerate(routes[rb]):
                x, y = tasks[tname]
                ax1.annotate(f'{k + 1}', (x, y), xytext=(-12, -14), textcoords='offset points',
                             color=c, fontsize=8, fontweight='bold')
    owner = {t: rb for rb, ts in routes.items() for t in ts}
    for tname, (x, y) in tasks.items():
        c = COLORS[robots.index(owner[tname]) % len(COLORS)] if tname in owner else '#888'
        done = tname in reached
        ax1.plot(x, y, 's', ms=12, mec=c, mfc=c if done else 'white', mew=2)
        ax1.annotate(tname, (x, y), xytext=(7, 6), textcoords='offset points', fontsize=9)
    ax1.set_xlim(-2, 2)
    ax1.set_ylim(-2, 2)
    ax1.set_aspect('equal')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.grid(True, alpha=0.4)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.set_title('Punto de control (—), rutas planificadas (- -), ■ tarea hecha, □ pendiente',
                  fontsize=9)

    # --- Cronograma ------------------------------------------------------------------------
    t_start = m.get('t_start') or 0.0
    finish = m.get('finish_time', {})
    for i, rb in enumerate(robots):
        c = COLORS[i % len(COLORS)]
        end = finish.get(rb)
        if end:
            ax2.barh(i, end, color=c, alpha=0.25, height=0.5)
        for tname in routes.get(rb, []):
            if tname in reached:
                tt = reached[tname][1]
                ax2.plot(tt, i, 'o', color=c, ms=9)
                ax2.annotate(tname, (tt, i), xytext=(-8, 9), textcoords='offset points',
                             fontsize=8)
    mk = m.get('makespan')
    if mk is not None:
        ax2.axvline(mk, color='#14213D', ls='--', lw=1, label=f'tiempo total {mk:.1f} s')
        ax2.legend(fontsize=8, loc='lower right')
    ax2.set_yticks(range(len(robots)))
    ax2.set_yticklabels(robots)
    ax2.set_ylim(-0.7, len(robots) + 0.4)
    ax2.set_xlabel('t desde la salida (s)')
    ax2.grid(True, axis='x', alpha=0.4)
    ax2.set_title(f'Tareas alcanzadas  (subasta: {m.get("t_auction")} s, salida t={t_start} s)',
                  fontsize=9)
    trav = m.get('travelled', {})
    txt = [f'{rb}: plan. {m.get("planned_lengths", {}).get(rb, 0):.2f} m · '
           f'recorrido {trav.get(rb, 0):.2f} m' for rb in robots]
    ax2.text(0.01, 0.99, '\n'.join(txt), transform=ax2.transAxes, va='top', fontsize=8,
             family='monospace', bbox={'fc': 'white', 'alpha': 0.8, 'ec': '#ccc'})
    fig.suptitle(f'{label(run)} · {m.get("completed", "?")}/{len(tasks)} tareas · '
                 f'distancia planificada total {m.get("planned_sum") or 0:.2f} m, '
                 f'ruta más larga {m.get("planned_max") or 0:.2f} m · {m.get("messages")} mensajes')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def table(runs):
    lines = ['| experimento | disposición | criterio | modo | tareas | dist. plan. total (m) | '
             'ruta plan. más larga (m) | dist. recorrida (m) | tiempo total (s) | mensajes |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for run in runs:
        m = run['meta']
        trav = sum(m.get('travelled', {}).values())
        mk = f'{m["makespan"]:.1f}' if m.get('makespan') is not None else 'no termina'
        lines.append(f'| {os.path.basename(run["path"])} | {m.get("layout")} | '
                     f'{m.get("criterion")} | {m.get("mode")} | {m.get("completed")}/'
                     f'{len(m.get("tasks", {}))} | {m.get("planned_sum") or 0:.2f} | '
                     f'{m.get("planned_max") or 0:.2f} | {trav:.2f} | {mk} | {m.get("messages")} |')
    return '\n'.join(lines)


def comparison_table(run):
    """Tabla de la comparación guardada en el JSON (misma situación, otros métodos)."""
    comp = run['meta'].get('comparison') or []
    if not comp:
        return ''
    lines = [f'Comparación para {os.path.basename(run["path"])} (mismas posiciones de salida):',
             '| método | distancia total (m) | ruta más larga (m) | mensajes |',
             '|---|---|---|---|']
    for c in comp:
        lines.append(f'| {c["method"]} | {c["sum"]:.2f} | {c["max"]:.2f} | '
                     f'{c["messages"] if c["messages"] is not None else "—"} |')
    return '\n'.join(lines)


def plot_compare(runs, out):
    import matplotlib.pyplot as plt
    labels = [label(r) for r in runs]
    x = np.arange(len(runs))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    ax1.bar(x - 0.2, [r['meta'].get('planned_sum') or 0 for r in runs], 0.4,
            label='distancia total planificada', color='#0082c8')
    ax1.bar(x + 0.2, [r['meta'].get('planned_max') or 0 for r in runs], 0.4,
            label='ruta más larga planificada', color='#f58231')
    ax1.set_ylabel('m')
    ax1.legend(fontsize=8)
    ax2.bar(x, [r['meta'].get('makespan') or 0 for r in runs], 0.5, color='#14213D')
    ax2.set_ylabel('tiempo total (s)')
    for ax in (ax1, ax2):
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha='right', fontsize=8)
        ax.grid(True, axis='y', alpha=0.4)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def main():
    p = argparse.ArgumentParser(description='Gráficas de la Práctica 2')
    p.add_argument('csv', nargs='+', help='uno o varios CSV de resultados/p2')
    p.add_argument('--nombre', default='comparacion_p2', help='nombre de la figura comparativa')
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
    seen = set()
    for run in runs:            # una tabla de comparación por situación (disposición + salida)
        key = (run['meta'].get('layout'), json.dumps(run['meta'].get('start_q'), sort_keys=True))
        ct = comparison_table(run)
        if ct and key not in seen:
            seen.add(key)
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
