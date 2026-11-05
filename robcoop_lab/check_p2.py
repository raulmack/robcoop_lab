"""Comprobación de la Práctica 2 (la misma que usa el profesor para evaluar).

    ros2 run robcoop_lab check_p2          # o:  python3 -m robcoop_lab.check_p2

Comprueba las cuatro funciones de auction_law.py, la subasta completa y el húngaro por
rondas en las disposiciones A, B y C, y simula la ejecución (kinsim_p2) con los dos
criterios. No necesita ROS ni Gazebo.
"""
import itertools
import math

import numpy as np

from . import auction_core, auction_law, kinsim_p2, tasks
from .arena import START_POSES
from .kinematics import control_point

OK, FAIL = '\033[1;32mPASA\033[0m', '\033[1;31mFALLA\033[0m'
INF = math.inf

# (suma de las rutas, ruta más larga) esperadas con 3 robots en sus marcas
EXPECTED = {
    ('A', 'suma'): (6.4530, 3.5427), ('A', 'max'): (6.4530, 3.5427),
    ('B', 'suma'): (4.2142, 4.2142), ('B', 'max'): (5.9569, 2.0327),
    ('C', 'suma'): (5.0204, 3.6105), ('C', 'max'): (5.7465, 2.5863),
    ('A', 'hungaro'): (7.0619, 3.1326), ('B', 'hungaro'): (6.0185, 2.7141),
    ('C', 'hungaro'): (4.9514, 1.7913),
}


def _close(a, b, tol=1e-6):
    return abs(float(a) - float(b)) <= tol


def check_route_length(law=auction_law):
    cases = [(((0, 0), []), 0.0), (((0, 0), [(3, 4)]), 5.0),
             (((1, 1), [(1, 2), (3, 2), (3, 0)]), 5.0), (((0, 0), [(1, 0), (0, 0)]), 2.0)]
    for (start, route), exp in cases:
        got = law.route_length(start, list(route))
        if not _close(got, exp):
            return False, f'start={start}, ruta={route}: L={got}, se esperaba {exp}'
    return True, 'longitud correcta en los casos de prueba'


def check_best_insertion(law=auction_law):
    s2 = math.sqrt(2)
    cases = [
        (((0, 0), [], (3, 4)), (5.0, 0)),
        (((0, 0), [(2, 0)], (1, 0)), (0.0, 0)),
        (((0, 0), [(1, 0), (2, 0)], (3, 0)), (1.0, 2)),
        (((1, 0), [(3, 0), (-1, 0)], (0, 0)), (0.0, 1)),
        (((0, 0), [(0, 1), (0, 2)], (1, 1)), (s2, 0)),      # empate en las 3 posiciones
    ]
    for (start, route, task), (exp_c, exp_k) in cases:
        route_copy = list(route)
        got = law.best_insertion(start, route_copy, task)
        if route_copy != list(route):
            return False, 'best_insertion no debe modificar la ruta que recibe'
        try:
            c, k = got
        except (TypeError, ValueError):
            return False, f'debe devolver (coste_extra, k), devuelve {got!r}'
        if not _close(c, exp_c) or int(k) != exp_k:
            return False, (f'start={start}, ruta={route}, tarea={task}: ({c:.4f}, {k}), '
                           f'se esperaba ({exp_c:.4f}, {exp_k})')
    return True, 'mejor inserción correcta (también con empates)'


def check_choose_winner(law=auction_law):
    cases = [
        ([], None),
        ([(1.2, 'tb3_0', 'P1'), (0.8, 'tb3_1', 'P2'), (0.9, 'tb3_2', 'P0')], (0.8, 'tb3_1', 'P2')),
        ([(0.5, 'tb3_2', 'P1'), (0.5, 'tb3_0', 'P3'), (0.7, 'tb3_1', 'P0')], (0.5, 'tb3_0', 'P3')),
        ([(0.5, 'tb3_1', 'P4'), (0.5, 'tb3_1', 'P2')], (0.5, 'tb3_1', 'P2')),
        ([(INF, 'tb3_0', 'P0'), (2.0, 'tb3_1', 'P0')], (2.0, 'tb3_1', 'P0')),
        ([(INF, 'tb3_0', 'P0'), (INF, 'tb3_1', 'P1')], None),
    ]
    for bids, exp in cases:
        got = law.choose_winner(list(bids))
        if (got is None) != (exp is None) or (exp is not None and tuple(got) != exp):
            return False, f'pujas={bids}: {got!r}, se esperaba {exp!r}'
    return True, 'puja mínima, empates e infinitos correctos'


def _brute_force(cost):
    R, T = cost.shape
    best = INF
    if R <= T:
        for cols in itertools.permutations(range(T), R):
            best = min(best, sum(cost[i, j] for i, j in enumerate(cols)))
    else:
        for rows in itertools.permutations(range(R), T):
            best = min(best, sum(cost[i, j] for j, i in enumerate(rows)))
    return best


def check_hungarian(law=auction_law):
    rng = np.random.default_rng(3)
    mats = [np.array([[4, 1, 3], [2, 0, 5], [3, 2, 2]], float),
            np.array([[1, 2, 3], [2, 4, 6]], float), np.array([[5, 1], [1, 5], [2, 2]], float)]
    mats += [rng.uniform(0, 3, size=s) for s in ((3, 3), (3, 6), (4, 4), (3, 2))]
    for C in mats:
        pairs = law.hungarian(C.tolist())
        if not isinstance(pairs, list) or len(pairs) != min(C.shape):
            return False, f'matriz {C.shape}: se esperaban {min(C.shape)} pares, {pairs!r}'
        if any(not isinstance(i, int) or not isinstance(j, int) for i, j in pairs):
            return False, 'los pares deben ser enteros de Python (usad int(...))'
        if [i for i, _ in pairs] != sorted(i for i, _ in pairs):
            return False, 'la lista debe ir ordenada por robot (i)'
        if len({i for i, _ in pairs}) != len(pairs) or len({j for _, j in pairs}) != len(pairs):
            return False, 'un robot o una tarea aparecen dos veces'
        total = sum(C[i, j] for i, j in pairs)
        if not _close(total, _brute_force(C), 1e-9):
            return False, f'matriz {C.shape}: coste {total:.3f}, el mínimo es {_brute_force(C):.3f}'
    return True, 'asignación óptima en matrices cuadradas y rectangulares'


def _starts(n=3):
    return {f'tb3_{i}': control_point(*START_POSES[i]) for i in range(n)}


def check_auction(law=auction_law, verbose=True):
    lines, ok_all = [], True
    for lay in ('A', 'B', 'C'):
        T = tasks.get(lay)
        for method in ('suma', 'max', 'hungaro'):
            r = auction_core.hungarian_rounds(_starts(), T, law) if method == 'hungaro' else \
                auction_core.ssi(_starts(), T, method, law)
            exp = EXPECTED[(lay, method)]
            ok = _close(r['sum'], exp[0], 2e-3) and _close(r['max'], exp[1], 2e-3)
            ok_all &= ok
            routes = '  '.join(f'{k[-1]}:{"-".join(v) or "·"}' for k, v in r['routes'].items())
            lines.append(f'  {"✔" if ok else "✘"} {lay} {r["method"]:20s} total={r["sum"]:.2f} m '
                         f'máx={r["max"]:.2f} m   {routes}'
                         + ('' if ok else f'   (se esperaba {exp[0]:.2f} / {exp[1]:.2f})'))
    if verbose:
        print('\n'.join(lines))
    return ok_all, 'subasta y húngaro por rondas dan las asignaciones esperadas'


def check_execution(law=auction_law, verbose=True):
    lines, ok_all = [], True
    for lay in ('A', 'B', 'C'):
        for crit in auction_core.CRITERIA:
            auc, ex, _ = kinsim_p2.simulate(lay, crit, 3, law=law)
            ok = ex['completed'] == ex['total'] and not ex['collision']
            ok_all &= ok
            mk = f'{ex["makespan"]:5.1f} s' if ex['makespan'] is not None else ' NO TERMINA'
            lines.append(f'  {"✔" if ok else "✘"} {lay} {crit:4s} {ex["completed"]}/{ex["total"]} '
                         f'tareas, tiempo total {mk}, distancia mínima {ex["min_dist"]:.2f} m')
    if verbose:
        print('\n'.join(lines))
    return ok_all, 'los robots completan todas las tareas sin chocar'


def main():
    print('Práctica 2 — comprobación de auction_law.py\n')
    results = []
    for name, fn in (('route_length', check_route_length),
                     ('best_insertion', check_best_insertion),
                     ('choose_winner', check_choose_winner), ('hungarian', check_hungarian)):
        try:
            ok, msg = fn()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] {name}: {msg}')
    if all(results):
        print('\nSubasta completa (sin ROS), 3 robots en sus marcas:')
        try:
            ok, msg = check_auction()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] asignación: {msg}')
        print('\nEjecución cinemática (kinsim_p2):')
        ok, msg = check_execution()
        results.append(ok)
        print(f'[{OK if ok else FAIL}] ejecución: {msg}')
    else:
        results += [False, False]
        print(f'[{FAIL}] asignación y ejecución: no se comprueban hasta que pasen las cuatro '
              'funciones')
    print(f'\nResultado: {sum(results)}/{len(results)} bloques correctos')
    return 0 if all(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
