"""Reto final (Práctica 4) sin ROS: fases, reglas y marcador (proporcionado, no se modifica).

La misión encadena las tres prácticas con los mismos robots, sin recolocarlos:

    1. formacion    desde las marcas, el equipo forma una figura por consenso        (P1)
    2. inspeccion   se subastan los puntos de inspección y cada robot visita los suyos (P2)
    3. cobertura    el equipo se despliega para cubrir la zona de 3 x 3 m             (P3)
    4. averia       un robot se para y deja de publicar; los demás se recolocan       (P3)

Cada fase tiene un tiempo límite. Si se agota, la misión SIGUE con la fase siguiente (con los
puntos que correspondan): un fallo en una fase no deja al equipo sin intentar las demás.

Aquí están las reglas (límites y puntos), la lectura de reto_config.py y el cálculo del
marcador. Lo usan igual el árbitro de ROS (mission_node.py) y el simulador sin ROS
(kinsim_p4.py), así que los dos puntúan lo mismo.
"""
import importlib
import math
import os
import random

from . import auction_core, formations, graphs

PHASES = ('formacion', 'inspeccion', 'cobertura', 'averia')
TITLES = {'formacion': '1 Formación', 'inspeccion': '2 Inspección',
          'cobertura': '3 Cobertura', 'averia': '4 Avería', 'seguridad': 'Seguridad'}

# Tiempo límite de cada fase (s) y puntos que vale (100 en total)
T_LIMIT = {'formacion': 60.0, 'inspeccion': 150.0, 'cobertura': 90.0, 'averia': 90.0}
POINTS = {'formacion': 20, 'inspeccion': 30, 'cobertura': 20, 'averia': 20, 'seguridad': 10}
PARTIAL = 0.75            # una fase a medias vale como mucho esta fracción de sus puntos

TOL_FORMATION = 0.05      # m: formados cuando el error de consenso baja de 5 cm ... (como P1)
HOLD_FORMATION = 3.0      # s: ... durante 3 s seguidos
PAUSE = 1.5               # s parados entre una fase y la siguiente
FAIL_DELAY = 2.0          # s entre el despliegue y la avería (averia:=auto o un robot dado)
T_WAIT_FAILURE = 45.0     # s que se espera la avería del profesor (averia:=profesor)
CONTACT_DIST = 0.21       # m entre centros de dos robots: por debajo se han tocado
SAFETY_MIN = 45.0         # puntos de las cuatro fases (de 90) para que cuente la seguridad
MARK_TOL = 0.15           # m: margen para dar por buena la salida desde las marcas

K_CONSENSUS_MAX = 1.0
K_COVERAGE_MAX = 3.0
FAILURES = ('auto', 'profesor', 'ninguna')      # o el nombre de un robot (tb3_1)

FIELDS = ('TOPOLOGIA', 'FORMACION', 'K_CONSENSO', 'CRITERIO', 'K_COBERTURA')


def default_out_dir():
    """Carpeta de resultados del reto: resultados/p4 del repositorio de la pareja."""
    repo = os.path.expanduser('~/ws/src/robcoop_lab')
    if os.path.isdir(repo):
        return os.path.join(repo, 'resultados', 'p4')
    return os.path.expanduser('~/ws/resultados/p4')


# --- Configuración de la pareja (reto_config.py) ---------------------------------------------
def load_config(module=None, n=3):
    """Lee y valida reto_config.py. Devuelve un diccionario:

        topology, formation, k_consensus, criterion, k_coverage, why {decisión: texto}

    module: módulo ya importado (por defecto, robcoop_lab.reto_config, recargado del disco).
    n: número de robots (la formación tiene que estar definida para ese número).
    Lanza ValueError con un mensaje que dice qué hay que corregir.
    """
    if module is None:
        from . import reto_config as module
        module = importlib.reload(module)
    missing = [f for f in FIELDS if not hasattr(module, f)]
    if missing:
        raise ValueError('reto_config.py: falta ' + ', '.join(missing))
    top, form = str(module.TOPOLOGIA).strip(), str(module.FORMACION).strip()
    crit = str(module.CRITERIO).strip()
    if top not in graphs.TOPOLOGIES:
        raise ValueError(f'reto_config.py: TOPOLOGIA = {top!r} no existe. Opciones: '
                         + ', '.join(graphs.TOPOLOGIES))
    try:
        formations.offsets(form, n)
    except ValueError as e:
        raise ValueError(f'reto_config.py: FORMACION = {form!r}: {e}') from None
    if crit not in auction_core.CRITERIA:
        raise ValueError(f'reto_config.py: CRITERIO = {crit!r} no existe. Opciones: '
                         + ', '.join(auction_core.CRITERIA))
    gains = {}
    for name, top_value in (('K_CONSENSO', K_CONSENSUS_MAX), ('K_COBERTURA', K_COVERAGE_MAX)):
        try:
            k = float(getattr(module, name))
        except (TypeError, ValueError):
            raise ValueError(f'reto_config.py: {name} debe ser un número') from None
        if not (math.isfinite(k) and 0.0 < k <= top_value):
            raise ValueError(f'reto_config.py: {name} = {k} debe ser mayor que 0 y como mucho '
                             f'{top_value}')
        gains[name] = k
    why = getattr(module, 'POR_QUE', {})
    why = {f: str(why.get(f, '')).strip() for f in FIELDS} if isinstance(why, dict) else \
        {f: '' for f in FIELDS}
    return {'topology': top, 'formation': form, 'k_consensus': gains['K_CONSENSO'],
            'criterion': crit, 'k_coverage': gains['K_COBERTURA'], 'why': why}


def describe(cfg):
    """La configuración en una línea."""
    return (f'formación {cfg["formation"]} con grafo {cfg["topology"]} (k={cfg["k_consensus"]}), '
            f'subasta {cfg["criterion"]}, cobertura k={cfg["k_coverage"]}')


def unjustified(cfg, min_chars=25):
    """Decisiones de reto_config.py que aún no tienen justificación."""
    return [f for f in FIELDS if len(cfg['why'].get(f, '')) < min_chars]


# --- Avería ----------------------------------------------------------------------------------
def pick_failure(failure, robots, seed=None):
    """Robot que se avería en la fase 4, o None si no lo elige el árbitro.

    failure: 'auto' (uno al azar: no se sabe cuál hasta que ocurre), el nombre de un robot,
             'profesor' (lo avería el profesor con «ros2 run robcoop_lab averia tb3_i») o
             'ninguna' (misión sin fase 4).
    """
    failure = failure.strip().strip('/')
    if failure in ('profesor', 'ninguna'):
        return None
    if failure == 'auto':
        return random.Random(seed).choice(list(robots))
    if failure not in robots:
        raise ValueError(f'averia debe ser {", ".join(FAILURES)} o uno de los robots: '
                         + ', '.join(robots))
    return failure


# --- Marcador --------------------------------------------------------------------------------
def progress_points(done, start, end, target, max_points):
    """Puntos de una fase que consiste en llevar una magnitud de 'start' hasta 'target'.

    Completada (done): todos los puntos. A medias: en proporción a lo que se ha avanzado,
    hasta PARTIAL · max_points. Vale para el error de formación (objetivo: la tolerancia) y
    para el coste de cobertura H (objetivo: el del algoritmo de Lloyd ideal).
    """
    if done:
        return float(max_points)
    if start is None or end is None or target is None or start - target <= 1e-9:
        return 0.0
    frac = max(0.0, min(1.0, (start - end) / (start - target)))
    return round(PARTIAL * max_points * frac, 1)


def scoreboard(phases, min_dist, emergency=False):
    """Marcador de una misión.

    phases: {fase: {'played', 'ok', 't', ...}} con lo medido en cada fase:
        formacion   e0, e_end                       (error de consenso al empezar y al acabar)
        inspeccion  done, total                     (puntos visitados)
        cobertura   h0, h_end, h_lloyd              (coste de cobertura)
        averia      h0, h_end, h_lloyd              (h0: coste justo después de la avería)
    min_dist: menor distancia entre los centros de dos robots en toda la misión (m).
    Seguridad: 10 puntos si los robots no se han tocado (min_dist >= CONTACT_DIST) ni ha
    habido parada de emergencia, y solo si el equipo suma SAFETY_MIN puntos en las fases.
    Devuelve {'points': {fase: puntos}, 'total', 'time', 'complete'}.
      time      suma de lo que ha durado cada fase jugada (el límite si no se ha completado;
                lo que llevaba, si la interrumpe una avería antes de tiempo)
      complete  True si se han completado las cuatro fases (solo entonces vale el tiempo)
    """
    pts = {}
    f = phases.get('formacion', {})
    pts['formacion'] = progress_points(f.get('ok'), f.get('e0'), f.get('e_end'), TOL_FORMATION,
                                       POINTS['formacion']) if f.get('played') else 0.0
    f = phases.get('inspeccion', {})
    pts['inspeccion'] = round(POINTS['inspeccion'] * f.get('done', 0) / f['total'], 1) \
        if f.get('played') and f.get('total') else 0.0
    for name in ('cobertura', 'averia'):
        f = phases.get(name, {})
        pts[name] = progress_points(f.get('ok'), f.get('h0'), f.get('h_end'), f.get('h_lloyd'),
                                    POINTS[name]) if f.get('played') else 0.0
    contact = min_dist is not None and math.isfinite(min_dist) and min_dist < CONTACT_DIST
    # no moverse no es ser seguro: hace falta la mitad de los puntos de las fases
    enough = sum(pts.values()) >= SAFETY_MIN
    pts['seguridad'] = float(POINTS['seguridad']) if enough and not (contact or emergency) \
        else 0.0
    time = 0.0
    for name in PHASES:
        f = phases.get(name, {})
        if f.get('played'):
            time += f['t'] if f.get('ok') and f.get('t') is not None else \
                f.get('cut', T_LIMIT[name])
    complete = all(phases.get(n, {}).get('ok') for n in PHASES) and not emergency
    return {'points': pts, 'total': round(sum(pts.values()), 1), 'time': round(time, 1),
            'complete': bool(complete)}


def phase_note(name, f):
    """Texto corto con lo que ha pasado en una fase (para el marcador)."""
    if not f.get('played'):
        return 'no jugada'
    if f.get('ok') and f.get('t') is not None:
        t = f'{f["t"]:.1f} s'
    elif f.get('cut') is not None:
        t = f'interrumpida por una avería a los {f["cut"]:.1f} s'
    else:
        t = f'no completada en {T_LIMIT[name]:.0f} s'
    if name == 'formacion':
        e = f.get('e_end')
        return f'{t}; error final {100 * e:.1f} cm' if e is not None else t
    if name == 'inspeccion':
        return f'{f.get("done", 0)}/{f.get("total", 0)} puntos; {t}'
    extra = f' ({f["robot"]})' if name == 'averia' and f.get('robot') else ''
    if f.get('h0') is None or f.get('h_end') is None:
        return t + extra
    lloyd = f', Lloyd ideal {f["h_lloyd"]:.2f}' if f.get('h_lloyd') is not None else ''
    return f'{t}{extra}; H {f["h0"]:.2f} -> {f["h_end"]:.2f} m⁴{lloyd}'


def format_scoreboard(phases, board, min_dist=None):
    """El marcador como texto, para la terminal."""
    lines = ['  fase            puntos   resultado']
    for name in PHASES:
        lines.append(f'  {TITLES[name]:14s} {board["points"][name]:5.1f}/{POINTS[name]:<3d} '
                     f'{phase_note(name, phases.get(name, {}))}')
    seg = f'distancia mínima entre robots {min_dist:.2f} m' if min_dist is not None and \
        math.isfinite(min_dist) else ''
    if board['points']['seguridad'] == 0 and seg and min_dist >= CONTACT_DIST:
        seg += f' (no cuenta: menos de {SAFETY_MIN:.0f} puntos en las fases o parada)'
    lines.append(f'  {TITLES["seguridad"]:14s} {board["points"]["seguridad"]:5.1f}/'
                 f'{POINTS["seguridad"]:<3d} {seg}')
    lines.append(f'  MARCADOR: {board["total"]:.1f} / 100      tiempo de misión: '
                 f'{board["time"]:.1f} s' + ('' if board['complete'] else
                                             '  (misión incompleta: el tiempo no puntúa)'))
    return '\n'.join(lines)
