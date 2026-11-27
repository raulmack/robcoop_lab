"""Comprobación de la Práctica 3 (la misma que usa el profesor para evaluar).

    ros2 run robcoop_lab check_p3          # o:  python3 -m robcoop_lab.check_p3

Comprueba las cuatro funciones de coverage_law.py, repite con ellas el algoritmo de Lloyd
ideal desde las dos salidas y simula el despliegue con robots diferenciales (kinsim_p3),
también con una zona de interés y con la avería de un robot. No necesita ROS ni Gazebo.
"""
import numpy as np

from . import coverage_core as core
from . import coverage_law, kinsim_p3
from .kinematics import control_point

OK, FAIL = '\033[1;32mPASA\033[0m', '\033[1;31mFALLA\033[0m'

# Coste final H (m⁴) del algoritmo de Lloyd ideal: (salida, robots, densidad)
EXPECTED = {
    ('marcas', 3, 'uniforme'): 5.3597, ('agrupados', 3, 'uniforme'): 5.3597,
    ('marcas', 4, 'uniforme'): 3.3713, ('agrupados', 4, 'uniforme'): 3.3713,
    ('marcas', 3, 'zona'): 10.4439, ('marcas', 2, 'uniforme'): 8.4338,
}
# Rejilla pequeña para los casos hechos a mano: 4 casillas en fila sobre el eje x
LINE = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])


def _random_cases(n_cases=6, seed=5):
    """Rejilla real + posiciones al azar de 1 a 5 robots (sin empates en la práctica)."""
    rng = np.random.default_rng(seed)
    points, cell_area = core.make_grid()
    for c in range(n_cases):
        robots = rng.uniform(-1.4, 1.4, size=(1 + c % 5, 2))
        weights = core.density('zona' if c % 2 else 'uniforme', points)
        yield points, weights, cell_area, robots


def check_mask(law=coverage_law):
    cases = [((0.0, 0.0), [], [True, True, True, True]),
             ((0.0, 0.0), [(3.0, 0.0)], [True, True, False, False]),
             ((3.0, 0.0), [(0.0, 0.0)], [False, False, True, True]),
             ((1.4, 0.5), [(0.0, 0.0), (3.0, -1.0)], [False, True, True, False]),
             ((0.0, 0.0), [(2.0, 0.0)], [True, True, False, False])]     # (1, 0): empate
    for p_i, others, exp in cases:
        got = law.voronoi_mask(LINE.copy(), np.array(p_i), [np.array(o) for o in others])
        got = np.asarray(got)
        if got.shape != (4,) or got.dtype != bool:
            return False, f'debe devolver un array booleano (M,); devuelve {got!r}'
        if got.tolist() != exp:
            return False, (f'casillas (0,0) (1,0) (2,0) (3,0), p_i={p_i}, otros={others}: '
                           f'{got.tolist()}, se esperaba {exp}')
    for points, _, _, robots in _random_cases():
        for i in range(len(robots)):
            others = [robots[j] for j in range(len(robots)) if j != i]
            got = np.asarray(law.voronoi_mask(points, robots[i], others), dtype=bool)
            if got.shape != (len(points),):
                return False, f'en la rejilla real debe devolver {len(points)} valores'
            bad = int(np.sum(got != core.ref_mask(points, robots[i], others)))
            if bad:
                return False, (f'{len(robots)} robots al azar: {bad} casillas mal asignadas al '
                               f'robot {i}')
    return True, 'celdas correctas (sin vecinos, con uno, con varios y con empates)'


def check_centroid(law=coverage_law):
    ones = np.ones(4)
    T, F = True, False
    cases = [(ones, [T, T, T, T], (1.5, 0.0)), (ones, [T, T, F, F], (0.5, 0.0)),
             (np.array([1.0, 0.0, 0.0, 3.0]), [T, T, T, T], (2.25, 0.0)),
             (np.array([1.0, 3.0, 5.0, 5.0]), [T, T, F, F], (0.75, 0.0)),
             (ones, [F, F, F, F], None), (np.zeros(4), [T, T, T, T], None)]
    for w, mask, exp in cases:
        got = law.centroid(LINE.copy(), w.copy(), np.array(mask))
        txt = f'pesos={w.tolist()}, celda={mask}'
        if exp is None:
            if got is not None:
                return False, f'{txt}: sin masa debe devolver None, devuelve {got!r}'
            continue
        if got is None:
            return False, f'{txt}: devuelve None, se esperaba {exp}'
        got = np.asarray(got, dtype=float)
        if got.shape != (2,) or not np.allclose(got, exp, atol=1e-9):
            return False, f'{txt}: {got.tolist()}, se esperaba {list(exp)}'
    for points, weights, _, robots in _random_cases():
        C, _ = core.ref_centroids(points, weights, robots)
        for i in range(len(robots)):
            mask = core.ref_mask(points, robots[i],
                                 [robots[j] for j in range(len(robots)) if j != i])
            got = law.centroid(points, weights, mask)
            if got is None or not np.allclose(np.asarray(got, dtype=float), C[i], atol=1e-9):
                return False, f'rejilla real, robot {i} de {len(robots)}: centroide incorrecto'
    return True, 'centroide correcto (uniforme, con densidad y celdas sin masa)'


def check_velocity(law=coverage_law):
    cases = [(([0, 0], [0, 0], 0.5), [0, 0]), (([0, 0], [1, 0], 0.5), [0.5, 0]),
             (([1, 1], [0, 3], 0.2), [-0.2, 0.4]), (([-1, 2], [0.5, -1], 1.0), [1.5, -3.0])]
    for (p, c, k), exp in cases:
        got = np.asarray(law.coverage_velocity(np.array(p, float), np.array(c, float), k), float)
        if got.shape != (2,) or not np.allclose(got, exp):
            return False, f'p_i={p}, C_i={c}, k={k}: u={got.tolist()}, se esperaba {exp}'
    return True, 'u_i = k (C_i − p_i) en los casos de prueba'


def check_cost(law=coverage_law):
    cases = [(np.ones(4), [(0.0, 0.0)], 1.0, 14.0),              # 0 + 1 + 4 + 9
             (np.ones(4), [(0.0, 0.0), (3.0, 0.0)], 1.0, 2.0),   # 0 + 1 + 1 + 0
             (np.array([1.0, 2.0, 0.0, 4.0]), [(1.0, 0.0)], 0.5, 8.5),   # (1 + 0 + 0 + 16)/2
             (np.ones(4), [(1.0, 1.0), (2.0, -1.0)], 0.25, 1.5)]         # (2 + 1 + 1 + 2)/4
    for w, robots, area, exp in cases:
        got = law.coverage_cost(LINE.copy(), w.copy(), [np.array(r) for r in robots], area)
        if not np.isscalar(got) or abs(float(got) - exp) > 1e-9:
            return False, (f'pesos={w.tolist()}, robots={robots}, ΔA={area}: H={got!r}, '
                           f'se esperaba {exp}')
    for points, weights, area, robots in _random_cases():
        got = float(law.coverage_cost(points, weights, [r for r in robots], area))
        exp = core.ref_cost(points, weights, robots, area)
        if abs(got - exp) > 1e-6 * max(exp, 1.0):
            return False, f'rejilla real, {len(robots)} robots: H={got:.4f}, se esperaba {exp:.4f}'
    return True, 'coste correcto (uniforme, con densidad y con varios robots)'


def check_lloyd(law=coverage_law, verbose=True):
    points, cell_area = core.make_grid()
    lines, ok_all = [], True
    for (start, n, dens), exp in EXPECTED.items():
        weights = core.density(dens, points)
        P0 = [control_point(*p) for p in core.start_poses(start, n)]
        P, costs = core.lloyd(P0, points, weights, cell_area, law)
        ok = abs(costs[-1] - exp) <= 0.002 * exp and \
            all(b <= a + 1e-9 for a, b in zip(costs, costs[1:]))
        ok_all &= ok
        pos = '  '.join(f'({x:+.2f},{y:+.2f})' for x, y in P)
        lines.append(f'  {"✔" if ok else "✘"} {start:9s} {n} robots {dens:8s} '
                     f'H: {costs[0]:6.2f} -> {costs[-1]:5.2f} m⁴ en {len(costs) - 1:2d} '
                     f'iteraciones   {pos}' + ('' if ok else f'   (se esperaba {exp:.2f})'))
    if verbose:
        print('\n'.join(lines))
    return ok_all, 'Lloyd llega a las configuraciones esperadas y H nunca sube'


def check_execution(law=coverage_law, verbose=True):
    cases = [('marcas', 3, 'uniforme', None), ('agrupados', 3, 'uniforme', None),
             ('marcas', 3, 'zona', None), ('marcas', 4, 'uniforme', None),
             ('marcas', 3, 'uniforme', 1)]
    lines, ok_all = [], True
    for start, n, dens, fail in cases:
        r = kinsim_p3.simulate(start, n, dens, law=law, fail=fail)
        exp = EXPECTED[('marcas', 2, 'uniforme')] if fail is not None else \
            EXPECTED[('marcas' if dens == 'zona' else start, n, dens)]
        ok = r['converged'] and not r['collision'] and r['final_cost'] <= 1.02 * exp
        ok_all &= ok
        tc = f'{r["t_conv"]:5.1f} s' if r['t_conv'] is not None else ' NO   '
        extra = ''
        if fail is not None:
            tr = f'{r["t_reconv"]:.1f} s' if r['t_reconv'] is not None else 'NO'
            extra = f'   avería de tb3_{fail}: recolocados en {tr}'
        lines.append(f'  {"✔" if ok else "✘"} {start:9s} {n} robots {dens:8s} desplegados en '
                     f'{tc}  H final {r["final_cost"]:5.2f} m⁴  distancia mínima '
                     f'{r["min_dist"]:.2f} m' + extra)
    if verbose:
        print('\n'.join(lines))
    return ok_all, 'los robots se despliegan sin chocar y llegan al coste esperado'


def main():
    print('Práctica 3 — comprobación de coverage_law.py\n')
    results = []
    for name, fn in (('voronoi_mask', check_mask), ('centroid', check_centroid),
                     ('coverage_velocity', check_velocity), ('coverage_cost', check_cost)):
        try:
            ok, msg = fn()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] {name}: {msg}')
    if all(results):
        print('\nAlgoritmo de Lloyd ideal (sin ROS), con vuestras funciones:')
        try:
            ok, msg = check_lloyd()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] despliegue ideal: {msg}')
        print('\nDespliegue con robots diferenciales (kinsim_p3):')
        try:
            ok, msg = check_execution()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] ejecución: {msg}')
    else:
        results += [False, False]
        print(f'[{FAIL}] despliegue ideal y ejecución: no se comprueban hasta que pasen las '
              'cuatro funciones')
    print(f'\nResultado: {sum(results)}/{len(results)} bloques correctos')
    return 0 if all(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
