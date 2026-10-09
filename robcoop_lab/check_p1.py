"""Comprobación de la Práctica 1 (la misma que usa el profesor para evaluar).

    ros2 run robcoop_lab check_p1          # o:  python3 -m robcoop_lab.check_p1

Comprueba las tres funciones de consensus_law.py y simula (kinsim) todas las
combinaciones de topología y formación con 3 y 4 robots. No necesita ROS ni Gazebo.
"""
import math

import numpy as np

from . import consensus_law, formations, graphs, kinsim

OK, FAIL = '\033[1;32mPASA\033[0m', '\033[1;31mFALLA\033[0m'


def lambda2_expected(topology, n):
    return {'completo': float(n), 'cadena': 2 * (1 - math.cos(math.pi / n)),
            'anillo': 2 * (1 - math.cos(2 * math.pi / n)) if n > 3 else 3.0,
            'estrella': 1.0}[topology]


def check_laplacian(law=consensus_law):
    for n in (3, 4, 5):
        for top in graphs.TOPOLOGIES:
            A = graphs.adjacency(top, n)
            L = np.asarray(law.laplacian(A), dtype=float)
            if L.shape != (n, n) or not np.allclose(L, np.diag(A.sum(1)) - A):
                return False, f'{top} con {n} robots: L incorrecto'
            if not np.allclose(L.sum(axis=1), 0):
                return False, 'las filas de L deben sumar 0'
    return True, 'L = D - A en todas las topologías'


def check_lambda2(law=consensus_law):
    for n in (3, 4, 5):
        for top in graphs.TOPOLOGIES:
            L = np.diag(graphs.adjacency(top, n).sum(1)) - graphs.adjacency(top, n)
            got = float(law.algebraic_connectivity(L))
            exp = lambda2_expected(top, n)
            if abs(got - exp) > 1e-6:
                return False, f'{top} con {n} robots: λ2 = {got:.4f}, se esperaba {exp:.4f}'
    return True, 'λ2 correcto en todas las topologías'


def check_velocity(law=consensus_law):
    cases = [
        (([0, 0], [], 0.1), [0, 0]),
        (([0, 0], [[1, 0]], 0.5), [0.5, 0]),
        (([1, 1], [[0, 1], [1, 0]], 0.2), [-0.2, -0.2]),
        (([2, -1], [[0, 0], [1, 1], [-1, 2]], 0.1), [-0.6, 0.6]),
    ]
    for (xi, nb, k), exp in cases:
        got = np.asarray(law.consensus_velocity(np.array(xi, float),
                                                [np.array(v, float) for v in nb], k), float)
        if got.shape != (2,) or not np.allclose(got, exp):
            return False, f'ξ_i={xi}, vecinos={nb}, k={k}: u={got.tolist()}, se esperaba {exp}'
    return True, 'u_i = -k Σ(ξ_i - ξ_j) en los casos de prueba'


def check_simulation(law=consensus_law, k=0.1, verbose=True):
    total, passed, lines = 0, 0, []
    for n in (3, 4):
        for form in formations.FORMATIONS:
            for top in graphs.TOPOLOGIES:
                if n == 3 and top == 'anillo':
                    continue            # con 3 robots anillo = completo
                total += 1
                r = kinsim.simulate(top, form, k, n, law=law)
                ok = r['converged']
                passed += ok
                tc = f'{r["t_conv"]:5.1f} s' if ok else ' --- '
                lines.append(f'  {"✔" if ok else "✘"} n={n} {top:9s} {form:9s} '
                             f'λ2={r["lambda2"]:.2f}  t_conv={tc}')
    if verbose:
        print('\n'.join(lines))
    return passed == total, f'{passed}/{total} simulaciones convergen (k={k})'


def main():
    print('Práctica 1 — comprobación de consensus_law.py\n')
    results = []
    for name, fn in (('laplaciano', check_laplacian), ('λ2', check_lambda2),
                     ('ley de consenso', check_velocity)):
        try:
            ok, msg = fn()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] {name}: {msg}')
    if results[2]:
        print('\nSimulación cinemática (sin ROS):')
        ok, msg = check_simulation()
        results.append(ok)
        print(f'[{OK if ok else FAIL}] simulación: {msg}')
    else:
        results.append(False)
        print(f'[{FAIL}] simulación: no se ejecuta hasta que la ley de consenso pase')
    print(f'\nResultado: {sum(results)}/{len(results)} bloques correctos')
    return 0 if all(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
