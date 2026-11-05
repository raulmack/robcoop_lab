"""Subastador y registro de un experimento de la Práctica 2 (proporcionado).

Hace de SUBASTADOR (coordina, pero no decide: las pujas las calculan los robots con su
información) y de OBSERVADOR que registra el experimento:

  1. espera a que todos los robots tengan pose y su nodo de subasta esté vivo;
  2. subasta secuencial de un solo ítem (SSI). En cada ronda:
        /auction/announce  anuncia TODAS las tareas pendientes
        /auction/bid       cada robot responde con una puja por cada tarea
        /auction/award     gana la puja más baja (choose_winner de auction_law.py)
     hasta adjudicar todas las tareas;
  3. comprueba que cada robot tiene su ruta y da la salida (/auction/start);
  4. registra a 10 Hz la pose de cada robot y las tareas alcanzadas (/auction/reached);
  5. al completar todas las tareas o agotar 'timeout', para todo y guarda
        <out_dir>/<nombre>.csv    trayectorias
        <out_dir>/<nombre>.json   asignación, pujas de cada ronda, métricas y comparación
                                  con el húngaro y el óptimo para la misma situación.
Publica además /auction/markers (RViz): tareas con el color del robot que las tiene y rutas.
"""
import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from . import auction_core, auction_law
from .auction_node import AUCTION_QOS, to_msg
from .kinematics import control_point
from .robot_api import RobotInterface
from .tasks import default_out_dir

COLORS = [(0.90, 0.10, 0.29), (0.24, 0.71, 0.29), (0.00, 0.51, 0.78), (0.96, 0.51, 0.19)]
MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AuctioneerNode(Node):

    def __init__(self):
        super().__init__('auctioneer')
        p = self.declare_parameter
        p('robots', ['tb3_0', 'tb3_1', 'tb3_2'])
        p('task_names', ['P0'])
        p('task_xy', [0.0, 0.0])            # x0, y0, x1, y1, ...
        p('layout', '')
        p('criterion', 'suma')
        p('mode', 'sim')
        p('name', '')
        p('out_dir', '')
        p('timeout', 300.0)                 # s de ejecución como máximo
        p('bid_timeout', 2.0)               # s de espera de las pujas de una ronda
        p('execute', True)                  # False: solo subasta, sin mover los robots
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.robots = list(g('robots'))
        names = list(g('task_names'))
        xy = list(g('task_xy'))
        if len(xy) != 2 * len(names):
            raise ValueError('task_xy debe tener 2 valores por tarea')
        self.tasks = {n: (float(xy[2 * i]), float(xy[2 * i + 1])) for i, n in enumerate(names)}
        self.criterion = g('criterion')
        self.timeout = float(g('timeout'))
        self.bid_timeout = float(g('bid_timeout'))
        self.execute = bool(g('execute'))
        self.meta = {'robots': self.robots, 'tasks': {k: list(v) for k, v in self.tasks.items()},
                     'layout': g('layout'), 'criterion': self.criterion, 'mode': g('mode')}
        stamp = time.strftime('%Y%m%d_%H%M%S')
        lay = g('layout') or 'puntos'
        self.name = g('name') or f'{g("mode")}_{lay}_{self.criterion}_{stamp}'
        self.out_dir = g('out_dir') or default_out_dir()
        os.makedirs(self.out_dir, exist_ok=True)
        self.auction_id = f'{self.name}_{int(time.time())}'

        self.ifaces = [RobotInterface(self, r, command=False) for r in self.robots]
        self.status = {}
        for r in self.robots:
            self.create_subscription(String, f'/{r}/auction/status',
                                     lambda m, r=r: self._on_status(r, m), 10)
        self.ann_pub = self.create_publisher(String, '/auction/announce', AUCTION_QOS)
        self.award_pub = self.create_publisher(String, '/auction/award', AUCTION_QOS)
        self.start_pub = self.create_publisher(String, '/auction/start', AUCTION_QOS)
        self.stop_pub = self.create_publisher(Bool, '/auction/stop', AUCTION_QOS)
        self.marker_pub = self.create_publisher(MarkerArray, '/auction/markers', MARKER_QOS)
        self.create_subscription(String, '/auction/bid', self._on_bid, AUCTION_QOS)
        self.create_subscription(String, '/auction/reached', self._on_reached, AUCTION_QOS)
        self.create_subscription(Bool, '/auction/stop', self._on_stop, AUCTION_QOS)

        self.phase = 'espera'
        self.ready_since = None
        self.t0 = None                      # inicio de la subasta
        self.t_start = None                 # salida de los robots (s desde t0)
        self.owner = {}                     # tarea -> robot
        self.award_order = {r: [] for r in self.robots}
        self.rounds = []
        self.round_bids = {}
        self.round_t = None
        self.retries = 0
        self.messages = 0
        self.reached = {}                   # tarea -> (robot, t desde la salida)
        self.rows = []
        self.done = False
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(
            f'Experimento "{self.name}": {len(self.tasks)} tareas, robots {self.robots}, '
            f'criterio {self.criterion}. Esperando a los robots...')

    # --- Utilidades -----------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _t(self):
        return self._now() - self.t0 if self.t0 is not None else 0.0

    def _on_status(self, r, msg):
        self.status[r] = json.loads(msg.data)

    def _task(self, n):
        return {'id': n, 'x': self.tasks[n][0], 'y': self.tasks[n][1]}

    def _remaining(self):
        return [n for n in self.tasks if n not in self.owner]

    # --- Subasta --------------------------------------------------------------------------
    def _announce(self):
        k = len(self.rounds) + 1
        self.round_bids = {}
        self.round_t = self._now()
        self.ann_pub.publish(to_msg({
            'auction': self.auction_id, 'round': k, 'criterion': self.criterion,
            'tasks': [self._task(n) for n in self._remaining()],
            # lo adjudicado a cada robot, en orden (por si alguna adjudicación llega tarde)
            'awarded': {r: [self._task(n) for n in self.award_order[r]] for r in self.robots}}))
        self.messages += 1

    def _on_bid(self, msg):
        data = json.loads(msg.data)
        if self.phase != 'subasta' or data.get('auction') != self.auction_id or \
           data.get('round') != len(self.rounds) + 1 or data.get('robot') not in self.robots:
            return
        self.messages += 1
        if set(data.get('route', [])) != set(self.award_order[data['robot']]):
            self.get_logger().warn(f'{data["robot"]} puja con una ruta desactualizada: '
                                   f'{data.get("route")}')
        self.round_bids[data['robot']] = [
            (math.inf if b['cost'] is None else float(b['cost']), data['robot'], b['task'])
            for b in data['bids'] if b['task'] in self._remaining()]

    def _close_round(self):
        bids = [b for r in sorted(self.round_bids) for b in self.round_bids[r]]
        win = auction_law.choose_winner(bids)
        k = len(self.rounds) + 1
        if win is None:
            self.get_logger().error(f'Ronda {k}: choose_winner no devuelve ninguna puja '
                                    '(¿está programada?). Se cancela la subasta.')
            self._finish('subasta fallida')
            return
        cost, robot, task = win
        if robot not in self.robots or task not in self._remaining():
            self.get_logger().error(f'Ronda {k}: ganadora no válida {win}')
            self._finish('subasta fallida')
            return
        self.owner[task] = robot
        self.award_order[robot].append(task)
        self.award_pub.publish(to_msg({
            'auction': self.auction_id, 'round': k, 'robot': robot, 'cost': float(cost),
            'task': self._task(task)}))
        self.messages += 1
        missing = [r for r in self.robots if r not in self.round_bids]
        self.rounds.append({'round': k, 'winner': robot, 'task': task,
                            'cost': round(float(cost), 4), 'missing': missing,
                            'bids': [[round(c, 4) if math.isfinite(c) else None, r, t]
                                     for c, r, t in bids]})
        self.get_logger().info(f'Ronda {k}: {task} -> {robot} (puja {cost:.2f})'
                               + (f'  sin puja de {missing}' if missing else ''))

    # --- Bucle principal ------------------------------------------------------------------
    def _tick(self):
        if self.done:
            return
        poses = [i.pose() for i in self.ifaces]
        now = self._now()

        if self.phase == 'espera':
            missing = [r for r, p in zip(self.robots, poses) if p is None or r not in self.status]
            if missing:
                self.ready_since = None
                self.get_logger().info(f'Esperando a: {", ".join(missing)}',
                                       throttle_duration_sec=3.0)
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 1.0:
                return
            self.t0 = now
            self.start_poses = {r: p for r, p in zip(self.robots, poses)}
            self.phase = 'subasta'
            self.get_logger().info('>>> Empieza la subasta')
            self._announce()

        self._log(poses)

        if self.phase == 'subasta':
            if len(self.round_bids) == len(self.robots) or \
               (now - self.round_t > self.bid_timeout and self.round_bids):
                self._close_round()
                if self.done:
                    return
                if self._remaining():
                    self._announce()
                else:
                    self.t_auction = round(self._t(), 3)
                    self.phase = 'confirmacion'
                    self.confirm_t = now
                    self.get_logger().info(f'>>> Subasta terminada en {self.t_auction:.2f} s')
            elif now - self.round_t > self.bid_timeout:
                self.retries += 1
                if self.retries > 3:
                    self._finish('sin pujas')
                    return
                self.get_logger().warn('Ninguna puja en esta ronda: se repite el anuncio')
                self._announce()
            return

        if self.phase == 'confirmacion':
            ok = all(set(self.status.get(r, {}).get('route', [])) ==
                     {t for t, o in self.owner.items() if o == r} for r in self.robots)
            if not ok and now - self.confirm_t < 3.0:
                return
            if not ok:
                self.get_logger().warn('Algún robot no confirma su ruta; se sigue igualmente')
            if not self.execute:
                self._finish('solo subasta')
                return
            self.phase = 'ejecucion'
            self.t_start = round(self._t(), 3)
            self.start_pub.publish(to_msg({'auction': self.auction_id}))
            self.get_logger().info('>>> Salida: los robots recorren sus rutas')
            return

        if self.phase == 'ejecucion':
            t_exec = self._t() - self.t_start
            if len(self.reached) == len(self.tasks):
                # se espera a que el último robot termine su parada en el punto
                if not hasattr(self, 't_all'):
                    self.t_all = now
                if now - self.t_all > 1.5:
                    self._finish('completado')
            elif t_exec > self.timeout:
                self._finish('tiempo agotado')

    def _on_reached(self, msg):
        data = json.loads(msg.data)
        if data.get('auction') != self.auction_id or self.t_start is None:
            return
        task = data['task']
        if task not in self.reached:
            self.reached[task] = (data['robot'], round(self._t() - self.t_start, 2))
            self.get_logger().info(f'{data["robot"]} alcanza {task} '
                                   f'({len(self.reached)}/{len(self.tasks)}) '
                                   f'a los {self.reached[task][1]:.1f} s')

    def _on_stop(self, msg):
        if msg.data and not self.done and self.phase != 'espera':
            self.get_logger().warn('PARADA recibida: se termina el experimento')
            self._finish('parada de emergencia')

    def _log(self, poses):
        row = [round(self._t(), 3), self.phase]
        for p in poses:
            row += [round(p[0], 4), round(p[1], 4), round(p[2], 4)] if p else ['', '', '']
        self.rows.append(row)

    # --- Marcadores para RViz -------------------------------------------------------------
    def _publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def marker(mid, mtype, ns):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        for i, (name, (x, y)) in enumerate(sorted(self.tasks.items())):
            owner = self.owner.get(name)
            col = COLORS[self.robots.index(owner) % len(COLORS)] if owner else (0.6, 0.6, 0.6)
            m = marker(i, Marker.CYLINDER, 'tareas')
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.005
            m.scale.x = m.scale.y = 0.16
            m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b = col
            m.color.a = 0.35 if name in self.reached else 0.9
            arr.markers.append(m)
            txt = marker(i, Marker.TEXT_VIEW_FACING, 'nombres')
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y + 0.15, 0.1
            txt.scale.z = 0.12
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = name + (' ✓' if name in self.reached else '')
            arr.markers.append(txt)
        for i, r in enumerate(self.robots):
            st = self.status.get(r, {})
            line = marker(i, Marker.LINE_STRIP, 'rutas')
            line.scale.x = 0.02
            line.color.r, line.color.g, line.color.b = COLORS[i % len(COLORS)]
            line.color.a = 0.8
            sp = getattr(self, 'start_poses', {}).get(r)
            pts = [control_point(*sp)[:2]] if sp else []
            pts += [self.tasks[t] for t in st.get('route', []) if t in self.tasks]
            for x, y in pts:
                line.points.append(Point(x=float(x), y=float(y), z=0.02))
            if len(line.points) < 2:
                line.action = Marker.DELETE
            arr.markers.append(line)
        self.marker_pub.publish(arr)

    # --- Final ----------------------------------------------------------------------------
    def stop_all(self):
        self.stop_pub.publish(Bool(data=True))

    def _travelled(self):
        dist = {r: 0.0 for r in self.robots}
        prev = {}
        for row in self.rows:
            for i, r in enumerate(self.robots):
                x, y, th = row[2 + 3 * i:5 + 3 * i]
                if x == '' or row[1] != 'ejecucion':
                    continue
                q = control_point(x, y, th)          # se mide igual que lo planificado
                if r in prev:
                    dist[r] += math.dist(prev[r], q)
                prev[r] = q
        return {r: round(v, 3) for r, v in dist.items()}

    def _finish(self, reason):
        self.done = True
        self.stop_all()
        starts = {r: control_point(*p) for r, p in getattr(self, 'start_poses', {}).items()}
        routes = {r: self.status.get(r, {}).get('route', []) for r in self.robots}
        planned = auction_core.summarize('subasta', starts, self.tasks, routes,
                                         self.messages) if starts else {}
        finish = {}
        for r in self.robots:
            mine = [self.reached[t][1] for t in routes[r] if t in self.reached]
            finish[r] = max(mine) if mine and len(mine) == len(routes[r]) else \
                (0.0 if not routes[r] else None)
        all_done = finish and all(v is not None for v in finish.values())
        makespan = max(finish.values()) if all_done else None
        comparison = []
        if starts:
            try:
                comparison = [{k: v for k, v in c.items() if k != 'rounds'}
                              for c in auction_core.compare(starts, self.tasks, auction_law)]
            except Exception as e:  # noqa: BLE001  (p. ej. hungarian aún sin programar)
                self.get_logger().warn(f'No se ha podido calcular la comparación: {e!r}')
        self.meta.update({
            'name': self.name, 'end_reason': reason, 'date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'start_poses': {r: [round(v, 4) for v in p]
                            for r, p in getattr(self, 'start_poses', {}).items()},
            'start_q': {r: [round(v, 4) for v in q] for r, q in starts.items()},
            'routes': routes, 'planned_lengths': planned.get('lengths'),
            'planned_sum': planned.get('sum'), 'planned_max': planned.get('max'),
            'messages': self.messages, 'rounds': self.rounds,
            't_auction': getattr(self, 't_auction', None), 't_start': self.t_start,
            'reached': {t: list(v) for t, v in self.reached.items()},
            'completed': len(self.reached), 'finish_time': finish, 'makespan': makespan,
            'travelled': self._travelled(),
            'nav': {r: {k: self.status.get(r, {}).get(k) for k in ('detours', 'backups')}
                    for r in self.robots},
            'comparison': comparison})
        csv_path = os.path.join(self.out_dir, self.name + '.csv')
        header = ['t', 'fase']
        for r in self.robots:
            header += [f'{r}_x', f'{r}_y', f'{r}_theta']
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)
        with open(os.path.join(self.out_dir, self.name + '.json'), 'w') as f:
            json.dump(self.meta, f, indent=2, ensure_ascii=False)
        lines = [f'\n==== FIN ({reason}) ====',
                 f'  criterio {self.criterion}: {self.messages} mensajes, '
                 f'subasta {self.meta["t_auction"]} s']
        for r in self.robots:
            lines.append(f'  {r}: {" -> ".join(routes[r]) or "(sin tareas)"}   '
                         f'planificado {planned.get("lengths", {}).get(r, 0):.2f} m, '
                         f'recorrido {self.meta["travelled"][r]:.2f} m, '
                         'termina ' + (f'{finish[r]:.1f} s' if finish[r] is not None else 'NO'))
        mk = f'{makespan:.1f} s' if makespan is not None else 'NO'
        lines.append(f'  tareas {len(self.reached)}/{len(self.tasks)}   tiempo total {mk}   '
                     f'distancia planificada: total {planned.get("sum")} m, '
                     f'máx {planned.get("max")} m')
        lines.append(f'  datos: {csv_path}')
        lines.append(f'  gráficas: ros2 run robcoop_lab plot_p2 {csv_path}')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = AuctioneerNode()
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
                node._finish('interrumpido (Ctrl+C)')
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
