"""Árbitro del reto final (proporcionado).

Dirige la misión, la registra y la puntúa. NO controla ningún robot: solo activa en cada fase
el comportamiento que toca (los calcula el agente de cada robot con vuestras leyes) y mide.

    espera      todos los robots con pose, con sus tres nodos vivos y en sus marcas
    formacion   /consensus/enable = True hasta que el error de consenso baja de 5 cm (3 s)
    subasta     subasta secuencial de los puntos de inspección (como el subastador de la P2)
    inspeccion  /auction/start: cada robot recorre su ruta; se anotan los puntos visitados
    cobertura   /coverage/enable = True hasta que cada robot está en su centroide (2 cm, 3 s)
    averia      un robot se para y calla (/tb3_i/coverage/fail); los demás se recolocan

Cada fase tiene un tiempo límite (mission_core.T_LIMIT); si se agota, se pasa a la siguiente.
Al terminar guarda
    <out_dir>/<nombre>.csv    trayectorias y métricas a 10 Hz, con la fase en cada fila
    <out_dir>/<nombre>.json   configuración, lo medido en cada fase y el MARCADOR
y escribe el marcador en la terminal. Publica /reto/markers para RViz.
«ros2 run robcoop_lab parar» (/reto/stop) termina la misión en cualquier momento.
"""
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from . import auction_core, auction_law, formations
from . import coverage_core as core
from . import mission_core as mc
from .arena import START_POSES
from .auction_node import AUCTION_QOS, to_msg
from .consensus_node import ENABLE_QOS
from .kinematics import control_point
from .robot_api import RobotInterface

COLORS = [(0.90, 0.10, 0.29), (0.24, 0.71, 0.29), (0.00, 0.51, 0.78), (0.96, 0.51, 0.19)]
MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
MARKER_RES = 0.10                       # m: casillas con las que se dibujan las celdas en RViz
BID_TIMEOUT = 2.0                       # s de espera de las pujas de una ronda
DT = 0.1                                # s: periodo del árbitro


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameter
        p('robots', ['tb3_0', 'tb3_1', 'tb3_2'])
        p('topology', 'completo')
        p('formation', 'poligono')
        p('k_consensus', 0.1)
        p('criterion', 'suma')
        p('k_coverage', core.K_DEFAULT)
        p('why', '{}')                      # justificación de reto_config.py (JSON)
        p('task_names', ['P0'])
        p('task_xy', [0.0, 0.0])            # x0, y0, x1, y1, ...
        p('layout', '')
        p('density', 'uniforme')
        p('center', list(core.ZONE_CENTER))
        p('sigma', core.ZONE_SIGMA)
        p('peak', core.ZONE_PEAK)
        p('failure', 'auto')                # auto | profesor | ninguna | tb3_i
        p('fail_seed', -1)                  # < 0: al azar de verdad
        p('mode', 'sim')
        p('name', '')
        p('out_dir', '')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.robots = list(g('robots'))
        n = len(self.robots)
        self.cfg = {'topology': g('topology'), 'formation': g('formation'),
                    'k_consensus': float(g('k_consensus')), 'criterion': g('criterion'),
                    'k_coverage': float(g('k_coverage'))}
        try:
            self.cfg['why'] = json.loads(g('why'))
        except ValueError:
            self.cfg['why'] = {}
        self.delta = np.array(formations.offsets(self.cfg['formation'], n))
        names, xy = list(g('task_names')), list(g('task_xy'))
        if len(xy) != 2 * len(names):
            raise ValueError('task_xy debe tener 2 valores por tarea')
        self.tasks = {t: (float(xy[2 * i]), float(xy[2 * i + 1])) for i, t in enumerate(names)}
        self.mode = g('mode')
        self.failure = g('failure').strip().strip('/')
        seed = int(g('fail_seed'))
        self.fail_seed = seed if seed >= 0 else int(time.time() * 1000) % 100000
        self.fail_robot = mc.pick_failure(self.failure, self.robots, self.fail_seed)
        if self.fail_robot and n < 2:
            raise ValueError('Para averiar un robot hacen falta al menos dos')
        self.points, self.cell_area = core.make_grid()
        zone = (tuple(float(v) for v in g('center')), float(g('sigma')), float(g('peak')))
        self.density = g('density')
        self.weights = core.density(self.density, self.points, *zone)
        self.mpoints, _ = core.make_grid(res=MARKER_RES)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.name = g('name') or f'{self.mode}_reto_{g("layout") or "puntos"}_{stamp}'
        self.out_dir = g('out_dir') or mc.default_out_dir()
        self.meta = {'robots': self.robots, 'mode': self.mode, 'config': self.cfg,
                     'layout': g('layout'), 'tasks': {k: list(v) for k, v in self.tasks.items()},
                     'density': self.density, 'failure': self.failure,
                     'limits': mc.T_LIMIT, 'limit': core.LIMIT, 'res': core.RES}
        if self.density == 'zona':
            self.meta.update({'center': list(zone[0]), 'sigma': zone[1], 'peak': zone[2]})

        # --- Comunicaciones ----------------------------------------------------------------
        self.ifaces = [RobotInterface(self, r, command=False) for r in self.robots]
        self.cmd_pubs = [self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in self.robots]
        self.seen = {r: set() for r in self.robots}      # nodos vivos de cada robot
        self.status = {}                                 # estado del nodo de subasta
        self.last_state = {}                             # último latido de cobertura
        self.fail_pubs = {}
        for r in self.robots:
            self.create_subscription(PointStamped, f'/{r}/consensus/state',
                                     lambda m, r=r: self.seen[r].add('consenso'), 10)
            self.create_subscription(String, f'/{r}/auction/status',
                                     lambda m, r=r: self._on_status(r, m), 10)
            self.create_subscription(PointStamped, f'/{r}/coverage/state',
                                     lambda m, r=r: self._on_state(r), 10)
            self.fail_pubs[r] = self.create_publisher(Bool, f'/{r}/coverage/fail', 10)
        self.consensus_pub = self.create_publisher(Bool, '/consensus/enable', ENABLE_QOS)
        self.coverage_pub = self.create_publisher(Bool, '/coverage/enable', ENABLE_QOS)
        self.ann_pub = self.create_publisher(String, '/auction/announce', AUCTION_QOS)
        self.award_pub = self.create_publisher(String, '/auction/award', AUCTION_QOS)
        self.start_pub = self.create_publisher(String, '/auction/start', AUCTION_QOS)
        self.stop_pub = self.create_publisher(Bool, '/auction/stop', AUCTION_QOS)
        self.marker_pub = self.create_publisher(MarkerArray, '/reto/markers', MARKER_QOS)
        self.create_subscription(String, '/auction/bid', self._on_bid, AUCTION_QOS)
        self.create_subscription(String, '/auction/reached', self._on_reached, AUCTION_QOS)
        self.create_subscription(Bool, '/reto/stop', self._on_stop, AUCTION_QOS)

        # --- Estado ------------------------------------------------------------------------
        self.phase = 'espera'
        self.phases = {name: {'played': False, 'ok': False, 't': None} for name in mc.PHASES}
        self.ready_since = None
        self.t0 = None                      # inicio de la misión
        self.t_phase = None                 # inicio de la fase en curso
        self.stable = 0.0
        self.first = True                   # primer ciclo de la fase (se anota el valor inicial)
        self.wait_until, self.wait_fn = None, None
        self.min_dist = math.inf
        self.emergency = False
        self.rows = []
        self.last_poses = None
        self.err_form = None
        # subasta
        self.auction_id = f'{self.name}_{int(time.time())}'
        self.owner, self.award_order = {}, {r: [] for r in self.robots}
        self.rounds, self.round_bids, self.round_t, self.retries = [], {}, None, 0
        self.reached = {}
        self.start_q = {}
        # cobertura
        self.broken = set()
        self.cover = None                   # (q, activos, H, error) para los marcadores
        self.h_lloyd = None
        self.done = False

        self.consensus_pub.publish(Bool(data=False))
        self.coverage_pub.publish(Bool(data=False))
        self.create_timer(DT, self._tick)
        self.create_timer(0.5, self._publish_markers)
        fail = {'auto': 'un robot al azar', 'profesor': 'la decide el profesor',
                'ninguna': 'ninguna'}.get(self.failure, self.failure)
        self.get_logger().info(
            f'RETO "{self.name}": robots {self.robots}, {len(self.tasks)} puntos de inspección, '
            f'avería: {fail}.\n  Configuración: {mc.describe(self.cfg)}\n'
            '  Esperando a los robots...')

    # --- Utilidades ---------------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _t(self):
        return self._now() - self.t0 if self.t0 is not None else 0.0

    def _on_status(self, r, msg):
        self.seen[r].add('subasta')
        self.status[r] = json.loads(msg.data)

    def _on_state(self, r):
        self.seen[r].add('cobertura')
        self.last_state[r] = self._now()

    def _on_stop(self, msg):
        if msg.data and not self.done and self.phase != 'espera':
            self.get_logger().warn('PARADA recibida: se termina la misión')
            self.emergency = True
            self._finish('parada de emergencia')

    def _after(self, delay, fn):
        """Pausa: durante 'delay' s no se hace nada y después se llama a fn()."""
        self.wait_until, self.wait_fn = self._now() + delay, fn

    def _enter(self, phase):
        self.phase = phase
        self.t_phase = self._now()
        self.stable = 0.0
        self.first = True

    def _score(self):
        return mc.scoreboard(self.phases, self.min_dist, self.emergency)

    # --- Salida desde las marcas --------------------------------------------------------------
    def _off_marks(self, poses):
        off = []
        for r, p in zip(self.robots, poses):
            try:
                i = int(r.rsplit('_', 1)[1])
            except (IndexError, ValueError):
                continue
            if not 0 <= i < len(START_POSES):
                continue
            d = math.hypot(p[0] - START_POSES[i][0], p[1] - START_POSES[i][1])
            if d > mc.MARK_TOL:
                off.append((r, d))
        return off

    def _abort_off_marks(self, off):
        self.done = True
        lista = ', '.join(f'{r} a {d:.2f} m' for r, d in off)
        if self.mode == 'real':
            fix = ('Colocad los robots en sus marcas, Ctrl+C en la terminal de p4_real y '
                   'relanzadla (el anclaje se hace al arrancar).')
        else:
            fix = ('Probablemente siguen donde acabó la misión anterior: Ctrl+C en la terminal '
                   'de p4_sim, relanzadla y repetid la misión.')
        self.get_logger().error(
            f'Los robots no están en sus marcas de salida ({lista}). No se juega la misión ni '
            f'se guarda nada. {fix}')

    # --- Bucle principal ----------------------------------------------------------------------
    def _tick(self):
        if self.done:
            return
        poses = [i.pose() for i in self.ifaces]
        now = self._now()

        if self.phase == 'espera':
            missing = [r for r, p in zip(self.robots, poses)
                       if p is None or len(self.seen[r]) < 3]
            if missing:
                self.ready_since = None
                self.get_logger().info(f'Esperando a: {", ".join(missing)}',
                                       throttle_duration_sec=3.0)
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 1.0:
                return
            off = self._off_marks(poses)
            if off:
                self._abort_off_marks(off)
                return
            os.makedirs(self.out_dir, exist_ok=True)
            self.t0 = now
            self._start_formation()

        if any(p is None for p in poses):
            self.get_logger().warn('Se ha perdido la TF de algún robot', throttle_duration_sec=2.0)
            if self.last_poses is None:
                return
            poses = [p if p is not None else lp for p, lp in zip(poses, self.last_poses)]
        self.last_poses = poses
        q = {r: control_point(*p) for r, p in zip(self.robots, poses)}
        for i in range(len(poses)):
            for j in range(i + 1, len(poses)):
                self.min_dist = min(self.min_dist, math.dist(poses[i][:2], poses[j][:2]))

        # Medidas de este ciclo (según la fase)
        xi = np.array([q[r] for r in self.robots]) - self.delta
        self.err_form = float(np.max(np.linalg.norm(xi - xi.mean(axis=0), axis=1)))
        active, cost, err_cov = list(self.robots), None, None
        if self.phase in ('cobertura', 'espera_averia', 'averia') or \
           (self.wait_fn is not None and self.cover is not None):
            active = [r for r in self.robots if r not in self.broken
                      and now - self.last_state.get(r, -math.inf) < core.STALE + 0.5]
            if active:
                qa = [q[r] for r in active]
                cost = core.ref_cost(self.points, self.weights, qa, self.cell_area)
                err_cov = core.ref_error(self.points, self.weights, qa)
                self.cover = (q, active, cost, err_cov)
        fmt = lambda v, nd: '' if v is None else round(v, nd)  # noqa: E731
        row = [round(self._t(), 3), self.phase if self.wait_fn is None else 'pausa',
               round(self.err_form, 4), len(self.reached), fmt(cost, 4), fmt(err_cov, 4),
               len(active)]
        for r, p in zip(self.robots, poses):
            row += [round(p[0], 4), round(p[1], 4), round(p[2], 4), int(r in active)]
        self.rows.append(row)

        if self.wait_fn is not None:                    # pausa entre fases
            if now >= self.wait_until:
                fn, self.wait_fn = self.wait_fn, None
                fn()
            return
        t = now - self.t_phase
        step = getattr(self, '_step_' + self.phase)
        step(now, t, q, active, cost, err_cov)
        self.first = False

    # --- Fase 1 · formación -------------------------------------------------------------------
    def _start_formation(self):
        self._enter('formacion')
        self.consensus_pub.publish(Bool(data=True))
        self.get_logger().info(f'>>> FASE 1 · formación ({self.cfg["formation"]}, grafo '
                               f'{self.cfg["topology"]}, k={self.cfg["k_consensus"]})')

    def _step_formacion(self, now, t, q, active, cost, err_cov):
        f = self.phases['formacion']
        if self.first:
            f.update({'played': True, 'e0': round(self.err_form, 4)})
        f['e_end'] = round(self.err_form, 4)
        self.stable = self.stable + DT if self.err_form < mc.TOL_FORMATION else 0.0
        ok = self.stable >= mc.HOLD_FORMATION
        if not ok and t <= mc.T_LIMIT['formacion']:
            return
        self.consensus_pub.publish(Bool(data=False))
        f.update({'ok': ok, 't': round(t - self.stable + DT, 2) if ok else None})
        self.get_logger().info('>>> ' + ('Formados: ' if ok else 'FASE 1 NO completada: ')
                               + mc.phase_note('formacion', f))
        self._enter('pausa')
        self._after(mc.PAUSE, self._start_auction)

    # --- Fase 2 · subasta e inspección --------------------------------------------------------
    def _task(self, name):
        return {'id': name, 'x': self.tasks[name][0], 'y': self.tasks[name][1]}

    def _remaining(self):
        return [name for name in self.tasks if name not in self.owner]

    def _start_auction(self):
        self._enter('subasta')
        self.phases['inspeccion'].update({'played': True, 'done': 0, 'total': len(self.tasks)})
        self.start_q = {r: control_point(*p) for r, p in zip(self.robots, self.last_poses)}
        self.get_logger().info(f'>>> FASE 2 · subasta de {len(self.tasks)} puntos (criterio '
                               f'{self.cfg["criterion"]})')
        self._announce()

    def _announce(self):
        self.round_bids = {}
        self.round_t = self._now()
        self.ann_pub.publish(to_msg({
            'auction': self.auction_id, 'round': len(self.rounds) + 1,
            'criterion': self.cfg['criterion'],
            'tasks': [self._task(name) for name in self._remaining()],
            'awarded': {r: [self._task(name) for name in self.award_order[r]]
                        for r in self.robots}}))

    def _on_bid(self, msg):
        data = json.loads(msg.data)
        if self.phase != 'subasta' or data.get('auction') != self.auction_id or \
           data.get('round') != len(self.rounds) + 1 or data.get('robot') not in self.robots:
            return
        self.round_bids[data['robot']] = [
            (math.inf if b['cost'] is None else float(b['cost']), data['robot'], b['task'])
            for b in data['bids'] if b['task'] in self._remaining()]

    def _close_round(self):
        """Adjudica la ronda. Devuelve False si la subasta no puede seguir."""
        bids = [b for r in sorted(self.round_bids) for b in self.round_bids[r]]
        try:
            win = auction_law.choose_winner(bids)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'choose_winner da error: {e!r}')
            return False
        k = len(self.rounds) + 1
        if win is None or win[1] not in self.robots or win[2] not in self._remaining():
            self.get_logger().error(f'Ronda {k}: choose_winner no devuelve una puja válida '
                                    f'({win!r}). Revisad auction_law.py con check_p2.')
            return False
        cost, robot, task = win
        self.owner[task] = robot
        self.award_order[robot].append(task)
        self.award_pub.publish(to_msg({'auction': self.auction_id, 'round': k, 'robot': robot,
                                       'cost': float(cost), 'task': self._task(task)}))
        self.rounds.append({'round': k, 'winner': robot, 'task': task,
                            'cost': round(float(cost), 4)})
        self.get_logger().info(f'Ronda {k}: {task} -> {robot} (puja {cost:.2f})')
        return True

    def _step_subasta(self, now, t, q, active, cost, err_cov):
        if len(self.round_bids) == len(self.robots) or \
           (now - self.round_t > BID_TIMEOUT and self.round_bids):
            if not self._close_round():
                self._end_inspection(False, 'la subasta ha fallado')
            elif self._remaining():
                self._announce()
            else:
                self.phases['inspeccion']['t_auction'] = round(t, 2)
                self._enter('confirmacion')
        elif now - self.round_t > BID_TIMEOUT:
            self.retries += 1
            if self.retries > 3:
                self._end_inspection(False, 'ningún robot puja')
                return
            self.get_logger().warn('Ninguna puja en esta ronda: se repite el anuncio')
            self._announce()

    def _step_confirmacion(self, now, t, q, active, cost, err_cov):
        ok = all(set(self.status.get(r, {}).get('route', [])) ==
                 {tk for tk, o in self.owner.items() if o == r} for r in self.robots)
        if not ok and t < 3.0:
            return
        if not ok:
            self.get_logger().warn('Algún robot no confirma su ruta; se sigue igualmente')
        routes = {r: self.status.get(r, {}).get('route', []) for r in self.robots}
        planned = auction_core.summarize('subasta', self.start_q, self.tasks, routes)
        self.phases['inspeccion'].update({'routes': routes, 'planned_sum': planned['sum'],
                                          'planned_max': planned['max']})
        self._enter('inspeccion')
        self.start_pub.publish(to_msg({'auction': self.auction_id}))
        self.get_logger().info('>>> Salida: ' + '   '.join(
            f'{r}: {" -> ".join(routes[r]) or "(sin puntos)"}' for r in self.robots))

    def _on_reached(self, msg):
        data = json.loads(msg.data)
        if data.get('auction') != self.auction_id or self.phase != 'inspeccion':
            return
        task = data['task']
        if task in self.tasks and task not in self.reached:
            self.reached[task] = (data['robot'], round(self._now() - self.t_phase, 2))
            self.phases['inspeccion']['done'] = len(self.reached)
            self.get_logger().info(f'{data["robot"]} alcanza {task} '
                                   f'({len(self.reached)}/{len(self.tasks)}) '
                                   f'a los {self.reached[task][1]:.1f} s')

    def _step_inspeccion(self, now, t, q, active, cost, err_cov):
        if len(self.reached) == len(self.tasks):
            self._end_inspection(True)
        elif t > mc.T_LIMIT['inspeccion']:
            self._end_inspection(False, 'tiempo agotado')

    def _end_inspection(self, ok, why=''):
        f = self.phases['inspeccion']
        f.update({'ok': ok, 'done': len(self.reached),
                  't': max(v[1] for v in self.reached.values()) if ok else None,
                  'reached': {k: list(v) for k, v in self.reached.items()},
                  'rounds': self.rounds})
        if why:
            f['error'] = why
        for _ in range(3):                  # ningún robot debe seguir con su ruta
            self.stop_pub.publish(Bool(data=True))
        self.get_logger().info('>>> ' + ('Inspección completa: ' if ok else
                                         f'FASE 2 NO completada ({why}): ')
                               + mc.phase_note('inspeccion', f))
        self._enter('pausa')
        self._after(mc.PAUSE, self._start_coverage)

    # --- Fase 3 · cobertura -------------------------------------------------------------------
    def _start_coverage(self):
        self._enter('cobertura')
        q0 = [control_point(*p) for p in self.last_poses]
        self.h_lloyd = core.lloyd(q0, self.points, self.weights, self.cell_area)[1][-1]
        self.coverage_pub.publish(Bool(data=True))
        self.get_logger().info(f'>>> FASE 3 · cobertura (densidad {self.density}, '
                               f'k={self.cfg["k_coverage"]})')

    def _lost(self, active):
        """Robots que han dejado de publicar su posición sin que los haya averiado el árbitro."""
        return [r for r in self.robots if r not in active and r not in self.broken]

    def _step_cobertura(self, now, t, q, active, cost, err_cov):
        f = self.phases['cobertura']
        if cost is None:
            self.get_logger().warn('Ningún robot publica su posición', throttle_duration_sec=2.0)
            if t > mc.T_LIMIT['cobertura']:
                f.update({'played': True, 'ok': False})
                self._finish('sin robots en la cobertura')
            return
        if self.first:
            f.update({'played': True, 'h0': round(cost, 4), 'h_lloyd': round(self.h_lloyd, 4)})
        lost = self._lost(active)
        if not lost:
            f['h_end'] = round(cost, 4)
            self.stable = self.stable + DT if err_cov < core.TOL else 0.0
        ok = self.stable >= core.HOLD and not lost
        if not ok and not lost and t <= mc.T_LIMIT['cobertura']:
            return
        f.update({'ok': ok, 't': round(t - self.stable + DT, 2) if ok else None})
        if lost:                                        # la avería ha llegado antes de tiempo
            f['cut'] = round(t, 2)
        self.get_logger().info('>>> ' + ('Desplegados: ' if ok else 'FASE 3 NO completada: ')
                               + mc.phase_note('cobertura', f))
        if lost:                                        # avería antes de acabar el despliegue
            self._begin_recovery(lost[0], q, [r for r in active])
        elif self.failure == 'ninguna':
            self._finish('misión sin avería')
        elif self.fail_robot is not None:
            self._enter('pausa')
            self._after(mc.FAIL_DELAY, self._inject_failure)
        else:
            self._enter('espera_averia')
            self.get_logger().info(
                '>>> Esperando la avería del profesor (ros2 run robcoop_lab averia tb3_i)...')

    def _step_espera_averia(self, now, t, q, active, cost, err_cov):
        lost = self._lost(active)
        if lost:
            self._begin_recovery(lost[0], q, active)
        elif t > mc.T_WAIT_FAILURE:
            self.get_logger().warn(f'Nadie ha averiado un robot en {mc.T_WAIT_FAILURE:.0f} s: '
                                   'la misión termina sin la fase 4')
            self._finish('sin avería')

    # --- Fase 4 · avería ----------------------------------------------------------------------
    def _inject_failure(self):
        r = self.fail_robot
        self.broken.add(r)
        self.fail_pubs[r].publish(Bool(data=True))
        q = {rb: control_point(*p) for rb, p in zip(self.robots, self.last_poses)}
        self._begin_recovery(r, q, [rb for rb in self.robots if rb not in self.broken])

    def _begin_recovery(self, robot, q, active):
        self.broken.add(robot)
        rest = [q[r] for r in active if r != robot]
        self._enter('averia')
        f = self.phases['averia']
        f.update({'played': True, 'robot': robot, 't_mission': round(self._t(), 2)})
        if rest:
            h0 = core.ref_cost(self.points, self.weights, rest, self.cell_area)
            f.update({'h0': round(h0, 4), 'h_end': round(h0, 4), 'h_lloyd': round(
                core.lloyd(rest, self.points, self.weights, self.cell_area)[1][-1], 4)})
        self.get_logger().info(f'>>> FASE 4 · AVERÍA de {robot}: los demás tienen que '
                               'darse cuenta solos y recolocarse')

    def _step_averia(self, now, t, q, active, cost, err_cov):
        f = self.phases['averia']
        for r in self.broken:                           # se repite por si se pierde
            if len(self.rows) % 10 == 0 and r == self.fail_robot:
                self.fail_pubs[r].publish(Bool(data=True))
        for r in self._lost(active):
            self.broken.add(r)
            self.get_logger().warn(f'{r} también ha dejado de publicar su posición')
        if cost is not None:
            f['h_end'] = round(cost, 4)
            self.stable = self.stable + DT if err_cov < core.TOL else 0.0
        ok = self.stable >= core.HOLD
        if not ok and t <= mc.T_LIMIT['averia']:
            return
        f.update({'ok': ok, 't': round(t - self.stable + DT, 2) if ok else None})
        self.get_logger().info('>>> ' + ('Recolocados: ' if ok else 'FASE 4 NO completada: ')
                               + mc.phase_note('averia', f))
        self._finish('misión terminada')

    def _step_pausa(self, *args):
        pass

    # --- Marcadores para RViz -----------------------------------------------------------------
    def _publish_markers(self):
        if self.done or self.last_poses is None:
            return
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        def marker(mid, mtype, ns, rgb=(1.0, 1.0, 1.0), a=1.0):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], a
            return m

        q = {r: control_point(*p) for r, p in zip(self.robots, self.last_poses)}
        phase = self.phase
        coverage = self.cover is not None and (phase in ('cobertura', 'espera_averia', 'averia')
                                               or self.phases['cobertura']['played'])
        if phase == 'formacion':                        # dónde tiene que acabar cada robot
            center = (np.array([q[r] for r in self.robots]) - self.delta).mean(axis=0)
            for i, r in enumerate(self.robots):
                m = marker(i, Marker.CYLINDER, 'formacion', COLORS[i % len(COLORS)], 0.5)
                m.pose.position.x = float(center[0] + self.delta[i][0])
                m.pose.position.y = float(center[1] + self.delta[i][1])
                m.scale.x = m.scale.y = 0.14
                m.scale.z = 0.01
                arr.markers.append(m)
        if not coverage:                                # puntos de inspección y rutas
            for i, (name, (x, y)) in enumerate(sorted(self.tasks.items())):
                owner = self.owner.get(name)
                col = COLORS[self.robots.index(owner) % len(COLORS)] if owner else (0.6, 0.6, 0.6)
                m = marker(i, Marker.CYLINDER, 'puntos', col, 0.35 if name in self.reached else 0.9)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.005
                m.scale.x = m.scale.y = 0.16
                m.scale.z = 0.01
                arr.markers.append(m)
                txt = marker(i, Marker.TEXT_VIEW_FACING, 'nombres')
                txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y + 0.15, 0.1
                txt.scale.z = 0.12
                txt.text = name + (' ✓' if name in self.reached else '')
                arr.markers.append(txt)
            for i, r in enumerate(self.robots):
                route = self.status.get(r, {}).get('route', []) if self.owner else []
                pts = [self.start_q[r]] if r in self.start_q else []
                pts += [self.tasks[tk] for tk in route if tk in self.tasks]
                if len(pts) < 2:
                    continue
                line = marker(i, Marker.LINE_STRIP, 'rutas', COLORS[i % len(COLORS)], 0.8)
                line.scale.x = 0.02
                for x, y in pts:
                    line.points.append(Point(x=float(x), y=float(y), z=0.02))
                arr.markers.append(line)
        else:                                           # celdas de Voronoi y centroides
            cq, active, _, _ = self.cover
            active = [r for r in active if r not in self.broken]
            if active:
                qa = [cq[r] for r in active]
                colors = [COLORS[self.robots.index(r) % len(COLORS)] for r in active]
                owner = core.ref_owner(self.mpoints, qa)
                cells = marker(0, Marker.CUBE_LIST, 'celdas')
                cells.scale.x = cells.scale.y = MARKER_RES
                cells.scale.z = 0.005
                for (x, y), o in zip(self.mpoints, owner):
                    cells.points.append(Point(x=float(x), y=float(y), z=0.0))
                    c = colors[int(o)]
                    cells.colors.append(ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.35))
                arr.markers.append(cells)
                C, _ = core.ref_centroids(self.points, self.weights, qa)
                for i, r in enumerate(active):
                    if not np.all(np.isfinite(C[i])):
                        continue
                    m = marker(self.robots.index(r), Marker.SPHERE, 'centroides', colors[i])
                    m.pose.position.x, m.pose.position.y = float(C[i][0]), float(C[i][1])
                    m.pose.position.z = 0.03
                    m.scale.x = m.scale.y = m.scale.z = 0.09
                    arr.markers.append(m)
            L = core.LIMIT
            border = marker(0, Marker.LINE_STRIP, 'zona a cubrir')
            border.scale.x = 0.02
            for x, y in ((-L, -L), (L, -L), (L, L), (-L, L), (-L, -L)):
                border.points.append(Point(x=x, y=y, z=0.01))
            arr.markers.append(border)
            if self.density == 'zona':
                (cx, cy), s = self.meta['center'], self.meta['sigma']
                ring = marker(0, Marker.LINE_STRIP, 'zona de interes', (1.0, 0.9, 0.0))
                ring.scale.x = 0.03
                for a in np.linspace(0.0, 2 * math.pi, 41):
                    ring.points.append(Point(x=cx + s * math.cos(a), y=cy + s * math.sin(a),
                                             z=0.02))
                arr.markers.append(ring)
            for r in self.broken:                       # el robot averiado
                m = marker(self.robots.index(r), Marker.TEXT_VIEW_FACING, 'averiado',
                           (1.0, 0.2, 0.2))
                m.pose.position.x, m.pose.position.y, m.pose.position.z = q[r][0], q[r][1], 0.35
                m.scale.z = 0.14
                m.text = 'AVERIADO'
                arr.markers.append(m)

        label = {'espera': 'espera', 'formacion': 'FASE 1 · formación',
                 'subasta': 'FASE 2 · subasta', 'confirmacion': 'FASE 2 · subasta',
                 'inspeccion': 'FASE 2 · inspección', 'cobertura': 'FASE 3 · cobertura',
                 'espera_averia': 'esperando la avería', 'averia': 'FASE 4 · avería',
                 'pausa': 'pausa'}.get(phase, phase)
        if phase == 'formacion' and self.err_form is not None:
            label += f'   e = {100 * self.err_form:.0f} cm'
        elif phase == 'inspeccion':
            label += f'   {len(self.reached)}/{len(self.tasks)} puntos'
        elif coverage:
            label += f'   H = {self.cover[2]:.2f} m4   e = {100 * self.cover[3]:.0f} cm'
        txt = marker(0, Marker.TEXT_VIEW_FACING, 'texto')
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = 0.0, 1.95, 0.1
        txt.scale.z = 0.16
        txt.text = f'{label}   [{self._score()["total"]:.0f} puntos · {self._t():.0f} s]'
        arr.markers.append(txt)
        self.marker_pub.publish(arr)

    # --- Final --------------------------------------------------------------------------------
    def stop_all(self):
        self.consensus_pub.publish(Bool(data=False))
        self.coverage_pub.publish(Bool(data=False))
        self.stop_pub.publish(Bool(data=True))
        for pub in self.cmd_pubs:
            pub.publish(Twist())

    def _travelled(self):
        dist = {r: 0.0 for r in self.robots}
        prev = {}
        for row in self.rows:
            for i, r in enumerate(self.robots):
                x, y, th = row[7 + 4 * i:10 + 4 * i]
                qq = control_point(x, y, th)
                if r in prev:
                    dist[r] += math.dist(prev[r], qq)
                prev[r] = qq
        return {r: round(v, 3) for r, v in dist.items()}

    def _finish(self, reason):
        self.done = True
        self.stop_all()
        if not self.rows:
            self.get_logger().warn(f'Fin ({reason}) sin datos: no se guarda nada')
            return
        board = self._score()
        q = {r: control_point(*p) for r, p in zip(self.robots, self.last_poses)}
        self.meta.update({
            'name': self.name, 'end_reason': reason, 'date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration': self.rows[-1][0], 'phases': self.phases, 'scoreboard': board,
            'min_dist': round(self.min_dist, 3) if math.isfinite(self.min_dist) else None,
            'emergency': self.emergency, 'failed': sorted(self.broken),
            'fail_seed': self.fail_seed if self.failure == 'auto' else None,
            'start_q': {r: [round(v, 4) for v in p] for r, p in self.start_q.items()},
            'final_q': {r: [round(v, 4) for v in q[r]] for r in self.robots},
            'travelled': self._travelled()})
        csv_path = os.path.join(self.out_dir, self.name + '.csv')
        header = ['t', 'fase', 'e_formacion', 'puntos', 'coste', 'e_cobertura', 'activos']
        for r in self.robots:
            header += [f'{r}_x', f'{r}_y', f'{r}_theta', f'{r}_activo']
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)
        with open(os.path.join(self.out_dir, self.name + '.json'), 'w') as f:
            json.dump(self.meta, f, indent=2, ensure_ascii=False)
        lines = [f'\n==== FIN DEL RETO ({reason}) ====',
                 '  configuración: ' + mc.describe(self.cfg),
                 mc.format_scoreboard(self.phases, board, self.min_dist),
                 f'  datos: {csv_path}',
                 f'  gráficas: ros2 run robcoop_lab plot_p4 {csv_path}']
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):              # repetir la parada por si algún mensaje se pierde
            node.stop_all()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        try:
            node.stop_all()
            if not node.done and node.rows:
                node._finish('interrumpida (Ctrl+C)')
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
