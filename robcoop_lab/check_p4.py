"""Comprobación del reto final (la misma que usa el profesor para evaluar).

    ros2 run robcoop_lab check_p4          # o:  python3 -m robcoop_lab.check_p4

Comprueba que reto_config.py es válido y está justificado, que vuestras tres leyes siguen
pasando las comprobaciones de las prácticas 1, 2 y 3, y juega cuatro misiones completas sin
ROS (kinsim_p4) con puntos de inspección distintos y averiando cada vez un robot distinto.
No necesita ROS ni Gazebo. Tarda unos 15 s.
"""
from . import check_p1, check_p2, check_p3, kinsim_p4
from . import mission_core as mc

OK, FAIL = '\033[1;32mPASA\033[0m', '\033[1;31mFALLA\033[0m'

# (disposición, semilla, robot que se avería)
MISSIONS = (('A', 0, 'tb3_0'), ('B', 0, 'tb3_1'), ('C', 0, 'tb3_2'), ('aleatoria', 4, 'tb3_1'))

LAWS = (
    ('consensus_law.py (P1)', (check_p1.check_laplacian, check_p1.check_lambda2,
                               check_p1.check_velocity)),
    ('auction_law.py (P2)', (check_p2.check_route_length, check_p2.check_best_insertion,
                             check_p2.check_choose_winner, check_p2.check_hungarian)),
    ('coverage_law.py (P3)', (check_p3.check_mask, check_p3.check_centroid,
                              check_p3.check_velocity, check_p3.check_cost)),
)


def check_config(n=3):
    try:
        cfg = mc.load_config(n=n)
    except ValueError as e:
        return False, str(e), None
    return True, mc.describe(cfg), cfg


def check_why(cfg):
    pending = mc.unjustified(cfg)
    if pending:
        return False, 'falta justificar en POR_QUE: ' + ', '.join(pending)
    return True, 'las cinco decisiones tienen su justificación'


def check_law(functions):
    for fn in functions:
        try:
            ok, msg = fn()
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        if not ok:
            return False, f'{fn.__name__[6:]}: {msg}'
    return True, 'sus funciones pasan las comprobaciones de la práctica'


def check_missions(cfg, verbose=True, missions=MISSIONS):
    lines, ok_all, times = [], True, []
    for lay, seed, fail in missions:
        r = kinsim_p4.simulate(cfg, lay, '', seed, 3, fail)
        b = r['scoreboard']
        ok = b['total'] >= 100.0 and b['complete'] and not r['collision']
        ok_all &= ok
        times.append(b['time'])
        name = lay + (f' {seed}' if lay == 'aleatoria' else '')
        tt = '  '.join(f'{mc.TITLES[n][2:5]} ' + (f'{r["phases"][n]["t"]:5.1f}'
                                                 if r['phases'][n]['ok'] else '  NO ')
                       for n in mc.PHASES)
        lines.append(f'  {"✔" if ok else "✘"} puntos {name:12s} avería {fail}   {tt}   '
                     f'marcador {b["total"]:5.1f}   tiempo {b["time"]:6.1f} s'
                     + ('   ¡CHOQUE!' if r['collision'] else ''))
    if verbose:
        print('\n'.join(lines))
    mean = sum(times) / len(times)
    if ok_all:
        return True, f'las {len(missions)} misiones completas; tiempo medio {mean:.1f} s', mean
    return False, 'alguna misión no llega a 100 puntos: mirad qué fase falla', mean


def main():
    print('Reto final — comprobación de la misión\n')
    results = []
    ok, msg, cfg = check_config()
    results.append(ok)
    print(f'[{OK if ok else FAIL}] reto_config.py: {msg}')
    if cfg is not None:
        ok, msg = check_why(cfg)
    else:
        ok, msg = False, 'no se comprueba hasta que la configuración sea válida'
    results.append(ok)
    print(f'[{OK if ok else FAIL}] justificación: {msg}')
    laws_ok = True
    for name, functions in LAWS:
        ok, msg = check_law(functions)
        laws_ok &= ok
        results.append(ok)
        print(f'[{OK if ok else FAIL}] {name}: {msg}')
    if laws_ok and cfg is not None:
        print('\nMisiones completas sin ROS (kinsim_p4), tiempo de cada fase en s:')
        try:
            ok, msg, _ = check_missions(cfg)
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f'error: {e!r}'
        results.append(ok)
        print(f'[{OK if ok else FAIL}] misiones: {msg}')
    else:
        results.append(False)
        print(f'[{FAIL}] misiones: no se juegan hasta que pasen la configuración y las tres leyes')
    print(f'\nResultado: {sum(results)}/{len(results)} bloques correctos')
    return 0 if all(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
