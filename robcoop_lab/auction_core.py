"""Algoritmos de asignación de la Práctica 2, sin ROS (proporcionado).

Usan las funciones de auction_law.py (las vuestras) y sirven para:
  - calcular las pujas de un robot (lo usa auction_node.py, el nodo de cada robot);
  - reproducir la subasta completa sin ROS (compare_p2, check_p2, kinsim_p2);
  - las referencias centralizadas: húngaro por rondas y óptimo por fuerza bruta.

Criterios de puja:
    suma   coste marginal: lo que se alarga MI ruta si me quedo la tarea
           (busca la menor distancia total del equipo)
    max    longitud que tendría MI ruta con la tarea
           (busca que la ruta más larga sea corta: reparte la carga)
"""
import itertools
import math

CRITERIA = ('suma', 'max')


def _length(start, route):
    """Longitud de referencia (no depende del código del alumno; se usa para medir)."""
    total, prev = 0.0, start
    for p in route:
        total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total


# --- Pujas de un robot -----------------------------------------------------------------
def bid_cost(start, route, task, criterion, law):
    """(coste de la puja, posición de inserción) de una tarea para un robot."""
    extra, k = law.best_insertion(start, list(route), task)
    if criterion == 'suma':
        return float(extra), int(k)
    if criterion == 'max':
        return float(law.route_length(start, list(route)) + extra), int(k)
    raise ValueError(f'Criterio desconocido: {criterion}. Opciones: {", ".join(CRITERIA)}')


def robot_bids(robot, start, route, tasks, criterion, law):
    """Pujas de 'robot' por todas las tareas anunciadas: [(coste, robot, tarea), ...].

    route: lista de (x, y) de las tareas que ya tiene; tasks: {nombre: (x, y)}.
    """
    return [(bid_cost(start, route, xy, criterion, law)[0], robot, name)
            for name, xy in tasks.items()]


# --- Resumen de una asignación ---------------------------------------------------------
def summarize(method, starts, tasks, routes, messages=None, extra=None):
    """Diccionario con las rutas (nombres de tareas) y sus longitudes."""
    lengths = {r: _length(starts[r], [tasks[t] for t in routes[r]]) for r in starts}
    out = {'method': method, 'routes': {r: list(routes[r]) for r in starts},
           'lengths': {r: round(v, 4) for r, v in lengths.items()},
           'sum': round(sum(lengths.values()), 4), 'max': round(max(lengths.values()), 4),
           'messages': messages}
    if extra:
        out.update(extra)
    return out


# --- Subasta secuencial de un solo ítem (SSI), sin ROS ---------------------------------
def ssi(starts, tasks, criterion, law):
    """Reproduce el protocolo de los nodos: en cada ronda se anuncian TODAS las tareas
    pendientes, cada robot puja por cada una y se adjudica la puja más baja de todas.

    starts: {robot: (x, y)}; tasks: {nombre: (x, y)}.
    """
    routes = {r: [] for r in starts}
    remaining = dict(tasks)
    rounds, messages = [], 0
    while remaining:
        bids = []
        for r in sorted(starts):
            bids += robot_bids(r, starts[r], [tasks[t] for t in routes[r]], remaining,
                               criterion, law)
        win = law.choose_winner(bids)
        if win is None:
            raise RuntimeError('choose_winner no ha devuelto ninguna puja')
        cost, robot, task = win
        _, k = bid_cost(starts[robot], [tasks[t] for t in routes[robot]], tasks[task],
                        criterion, law)
        routes[robot].insert(k, task)
        del remaining[task]
        messages += 1 + len(starts) + 1          # anuncio + una puja por robot + adjudicación
        rounds.append({'round': len(rounds) + 1, 'winner': robot, 'task': task,
                       'cost': round(float(cost), 4), 'position': k,
                       'bids': [[round(float(c), 4), r, t] for c, r, t in bids]})
    return summarize(f'subasta ({criterion})', starts, tasks, routes, messages,
                     {'rounds': rounds, 'criterion': criterion})


# --- Húngaro por rondas (centralizado) -------------------------------------------------
def hungarian_rounds(starts, tasks, law):
    """Asignación centralizada: en cada ronda el húngaro da UNA tarea a cada robot,
    con el coste = distancia desde donde termina su ruta. Se repite hasta acabar.

    Mensajes: cada robot envía su posición y recibe su ruta (2 por robot).
    """
    robots = sorted(starts)
    pos = {r: starts[r] for r in robots}
    routes = {r: [] for r in robots}
    remaining = sorted(tasks)
    while remaining:
        cost = [[math.dist(pos[r], tasks[t]) for t in remaining] for r in robots]
        pairs = law.hungarian(cost)
        if not pairs:
            raise RuntimeError('hungarian no ha devuelto ninguna pareja')
        used = []
        for i, j in pairs:
            t = remaining[j]
            routes[robots[i]].append(t)
            pos[robots[i]] = tasks[t]
            used.append(t)
        remaining = [t for t in remaining if t not in used]
    return summarize('húngaro por rondas', starts, tasks, routes, 2 * len(robots))


# --- Óptimo por fuerza bruta (referencia) ----------------------------------------------
def optimal(starts, tasks, criterion):
    """Mejor asignación posible probando todos los repartos y órdenes (hasta 8 tareas)."""
    robots, names = sorted(starts), sorted(tasks)
    if len(names) > 8:
        raise ValueError('El óptimo por fuerza bruta solo es razonable hasta 8 tareas')
    best_path = {}                       # (robot, subconjunto) -> (longitud, orden)
    for r in robots:
        for size in range(len(names) + 1):
            for sub in itertools.combinations(names, size):
                best = (math.inf, ())
                for perm in itertools.permutations(sub):
                    L = _length(starts[r], [tasks[t] for t in perm])
                    if L < best[0]:
                        best = (L, perm)
                best_path[(r, frozenset(sub))] = best
    best_key, subs = None, None
    for owners in itertools.product(range(len(robots)), repeat=len(names)):
        cand = [frozenset(n for n, o in zip(names, owners) if o == i) for i in range(len(robots))]
        lens = [best_path[(r, s)][0] for r, s in zip(robots, cand)]
        # suma: menor distancia total; max: menor ruta más larga (y, a igualdad, menor total)
        key = (round(sum(lens), 9),) if criterion == 'suma' else \
            (round(max(lens), 9), round(sum(lens), 9))
        if best_key is None or key < best_key:
            best_key, subs = key, cand
    routes = {r: list(best_path[(r, s)][1]) for r, s in zip(robots, subs)}
    return summarize(f'óptimo ({criterion})', starts, tasks, routes, None)


def compare(starts, tasks, law):
    """Todas las asignaciones para la misma situación (lo que muestra compare_p2)."""
    return [ssi(starts, tasks, 'suma', law), ssi(starts, tasks, 'max', law),
            hungarian_rounds(starts, tasks, law),
            optimal(starts, tasks, 'suma'), optimal(starts, tasks, 'max')]
