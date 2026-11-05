"""Nodo de subasta de UN robot (proporcionado; las pujas usan auction_law.py).

Se lanza uno por robot, en su namespace. Cada nodo es un PARTICIPANTE de la subasta:
  - guarda su ruta (lista ordenada de tareas que ha ganado);
  - cuando el subastador anuncia tareas (/auction/announce), calcula una puja por cada una
    con SU posición y SU ruta, y las envía (/auction/bid);
  - cuando se adjudica una tarea a este robot (/auction/award), la inserta en su ruta en la
    mejor posición (best_insertion);
  - con /auction/start recorre su ruta con go_to_point + evitación y avisa de cada tarea
    alcanzada (/auction/reached). Al terminar se queda parado donde está.
  - /auction/stop (True) para el robot en cualquier momento (ros2 run robcoop_lab parar).
  - publica su estado en /<robot>/auction/status (2 Hz) y un latido en /<robot>/heartbeat.

Todos los mensajes son std_msgs/String con JSON (se leen con  ros2 topic echo /auction/bid).
Parámetros: criterion ('suma' | 'max'), rate, dwell (s parado en cada punto), priority.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Header, String

from . import auction_core, auction_law
from .kinematics import control_point
from .navigation import GOAL_TOL, GoToPoint, scan_points
from .robot_api import RobotInterface

AUCTION_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)


def to_msg(data):
    return String(data=json.dumps(data, ensure_ascii=False))


class AuctionNode(Node):

    def __init__(self):
        super().__init__('auction_node')
        self.declare_parameter('criterion', 'suma')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('dwell', 1.0)          # s parado en cada punto («inspección»)
        self.declare_parameter('priority', 0)         # desempate en bloqueos (0 = máxima)
        self.ns = self.get_namespace().strip('/')
        self.criterion = self.get_parameter('criterion').value
        if self.criterion not in auction_core.CRITERIA:
            raise ValueError(f'criterion debe ser uno de {auction_core.CRITERIA}')
        self.dwell = float(self.get_parameter('dwell').value)

        self.robot = RobotInterface(self, self.ns)
        self.nav = GoToPoint(priority=int(self.get_parameter('priority').value))
        self._reset(None)

        self.bid_pub = self.create_publisher(String, '/auction/bid', AUCTION_QOS)
        self.reached_pub = self.create_publisher(String, '/auction/reached', AUCTION_QOS)
        self.status_pub = self.create_publisher(String, 'auction/status', 10)
        self.hb_pub = self.create_publisher(Header, 'heartbeat', 10)
        self.create_subscription(String, '/auction/announce', self._on_announce, AUCTION_QOS)
        self.create_subscription(String, '/auction/award', self._on_award, AUCTION_QOS)
        self.create_subscription(String, '/auction/start', self._on_start, AUCTION_QOS)
        self.create_subscription(Bool, '/auction/stop', self._on_stop, AUCTION_QOS)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._step)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f'{self.ns}: participante de la subasta (criterio {self.criterion})')

    # --- Estado ---------------------------------------------------------------------------
    def _reset(self, auction_id):
        self.auction_id = auction_id
        self.route = []              # [{'id', 'x', 'y'}] en el orden en que se visitarán
        self.done = []
        self.phase = 'espera'        # espera | subasta | ejecucion | terminado | parado
        self.start_q = None          # punto de control al empezar la subasta
        self.wait_until = None

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _xy_route(self):
        return [(t['x'], t['y']) for t in self.route]

    # --- Subasta --------------------------------------------------------------------------
    def _on_announce(self, msg):
        data = json.loads(msg.data)
        if data['auction'] != self.auction_id:              # subasta nueva: se empieza de cero
            self.robot.stop()
            self._reset(data['auction'])
        if self.phase not in ('espera', 'subasta'):
            return
        pose = self.robot.pose()
        if pose is None:
            self.get_logger().warn('Anuncio recibido sin pose (¿TF?): no pujo')
            return
        if self.start_q is None:
            self.start_q = control_point(*pose)
        self.phase = 'subasta'
        # El anuncio trae lo adjudicado hasta ahora: si la adjudicación de la ronda anterior
        # aún no ha llegado (los tópicos no garantizan el orden entre sí), se aplica aquí.
        for t in data.get('awarded', {}).get(self.ns, []):
            self._insert(t)
        tasks = {t['id']: (t['x'], t['y']) for t in data['tasks']}
        bids = auction_core.robot_bids(self.ns, self.start_q, self._xy_route(), tasks,
                                       self.criterion, auction_law)
        self.bid_pub.publish(to_msg({
            'auction': self.auction_id, 'round': data['round'], 'robot': self.ns,
            'route': [t['id'] for t in self.route],
            'bids': [{'task': t, 'cost': (round(c, 4) if math.isfinite(c) else None)}
                     for c, _, t in bids]}))

    def _on_award(self, msg):
        data = json.loads(msg.data)
        if data['auction'] != self.auction_id or data['robot'] != self.ns:
            return
        if self._insert(data['task']):
            self.get_logger().info(f'Gano {data["task"]["id"]} (puja {data["cost"]:.2f}); '
                                   'ruta: ' + ' -> '.join(x['id'] for x in self.route))

    def _insert(self, t):
        """Inserta la tarea ganada en la mejor posición de la ruta (si no estaba ya)."""
        if any(x['id'] == t['id'] for x in self.route) or self.start_q is None:
            return False
        _, k = auction_core.bid_cost(self.start_q, self._xy_route(), (t['x'], t['y']),
                                     self.criterion, auction_law)
        self.route.insert(k, {'id': t['id'], 'x': t['x'], 'y': t['y']})
        return True

    def _on_start(self, msg):
        data = json.loads(msg.data)
        if data['auction'] != self.auction_id or self.phase not in ('espera', 'subasta'):
            return
        self.phase = 'ejecucion'
        self._next_goal()
        ruta = ' -> '.join(x['id'] for x in self.route) or 'sin tareas'
        self.get_logger().info('>>> En marcha: ' + ruta)

    def _on_stop(self, msg):
        if msg.data and self.phase != 'parado':
            self.phase = 'parado'
            self.robot.stop()
            self.get_logger().warn('PARADA recibida')

    # --- Ejecución de la ruta -------------------------------------------------------------
    def _pending(self):
        return [t for t in self.route if t['id'] not in self.done]

    def _next_goal(self):
        pend = self._pending()
        if pend:
            self.nav.set_goal((pend[0]['x'], pend[0]['y']))

    def _step(self):
        if self.phase != 'ejecucion':
            return
        pose = self.robot.pose()
        if pose is None:
            self.robot.stop()
            self.get_logger().warn('Sin TF del robot: parado', throttle_duration_sec=2.0)
            return
        now = self._now()
        if self.wait_until is not None:                    # parado en el punto
            if now < self.wait_until:
                self.robot.stop()
                return
            self.wait_until = None
            self._next_goal()
        pend = self._pending()
        if not pend:
            self.robot.stop()
            self.phase = 'terminado'
            self.get_logger().info('>>> Ruta terminada')
            return
        obstacles = scan_points(self.robot.safety.scan)
        v, w, dist = self.nav.command(pose, obstacles, now)
        if dist < GOAL_TOL:
            self.robot.stop()
            task = pend[0]
            self.done.append(task['id'])
            self.wait_until = now + self.dwell
            self.reached_pub.publish(to_msg({'auction': self.auction_id, 'robot': self.ns,
                                             'task': task['id']}))
            self.get_logger().info(f'Alcanzado {task["id"]} ({len(self.done)}/{len(self.route)})')
            return
        self.robot.send(v, w)
        if self.robot.safety.blocked:
            self.get_logger().warn('Filtro de seguridad: obstáculo delante, avance anulado',
                                   throttle_duration_sec=2.0)

    def _publish_status(self):
        pend = self._pending()
        self.status_pub.publish(to_msg({
            'robot': self.ns, 'auction': self.auction_id, 'phase': self.phase,
            'criterion': self.criterion, 'route': [t['id'] for t in self.route],
            'done': list(self.done), 'current': pend[0]['id'] if pend else None,
            'nav': self.nav.state, 'detours': self.nav.detours, 'backups': self.nav.backups,
            'blocked': bool(self.robot.safety.blocked)}))
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = self.ns
        self.hb_pub.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = AuctionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001  (al cerrar con Ctrl+C a veces salta un error interno)
        if rclpy.ok():
            raise
    finally:
        try:
            node.robot.stop()       # el robot real NO se para solo si deja de recibir órdenes
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
