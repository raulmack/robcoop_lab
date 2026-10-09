"""Nodo de consenso de UN robot (proporcionado; la ley está en consensus_law.py).

Se lanza uno por robot, en su namespace. Cada nodo:
  - lee la pose de SU robot (TF) y calcula su variable de consenso ξ_i = q_i - δ_i;
  - la publica en /<robot>/consensus/state;
  - se suscribe SOLO a /<vecino>/consensus/state de sus vecinos (eso es el grafo);
  - cuando /consensus/enable es True, calcula u_i con consensus_law.consensus_velocity,
    la convierte en (v, ω) y la envía al robot a través del filtro de seguridad.

Parámetros: neighbors, offset [dx, dy], k, rate, stale_time, deadband.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray

from . import consensus_law
from .kinematics import control_point, integrator_to_unicycle, saturate
from .robot_api import RobotInterface

ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ConsensusNode(Node):

    def __init__(self):
        super().__init__('consensus_node')
        self.declare_parameter('neighbors', [''])
        self.declare_parameter('offset', [0.0, 0.0])
        self.declare_parameter('k', 0.1)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('stale_time', 1.0)     # s sin noticias -> vecino ignorado
        self.declare_parameter('deadband', 0.001)     # m/s, por debajo no se mueve

        self.ns = self.get_namespace().strip('/')
        self.neighbors = [n for n in self.get_parameter('neighbors').value if n]
        self.delta = np.array(self.get_parameter('offset').value, dtype=float)
        self.k = float(self.get_parameter('k').value)
        self.stale = Duration(seconds=float(self.get_parameter('stale_time').value))
        self.deadband = float(self.get_parameter('deadband').value)

        self.robot = RobotInterface(self, self.ns)
        self.enabled = False
        self.xi_nb = {}                       # vecino -> (ξ_j, instante de recepción)
        self.disturb_until = None             # perturbación activa (prueba de robustez)
        self.disturb_u = np.zeros(2)

        self.state_pub = self.create_publisher(PointStamped, 'consensus/state', 10)
        for nb in self.neighbors:
            self.create_subscription(PointStamped, f'/{nb}/consensus/state',
                                     lambda msg, nb=nb: self._on_neighbor(nb, msg), 10)
        self.create_subscription(Bool, '/consensus/enable', self._on_enable, ENABLE_QOS)
        self.create_subscription(Float64MultiArray, 'consensus/disturb', self._on_disturb, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._step)

        self.get_logger().info(f'{self.ns}: vecinos={self.neighbors} offset={self.delta.tolist()} '
                               f'k={self.k}')

    # --- Callbacks --------------------------------------------------------------
    def _on_neighbor(self, nb, msg):
        self.xi_nb[nb] = (np.array([msg.point.x, msg.point.y]), self.get_clock().now())

    def _on_enable(self, msg):
        if msg.data == self.enabled:
            return
        if not msg.data:
            self.robot.stop()
        self.enabled = msg.data
        self.get_logger().info('consenso ACTIVADO' if msg.data else 'consenso PARADO')

    def _on_disturb(self, msg):
        """[u_x, u_y, duración]: durante 'duración' s el robot ignora el consenso."""
        ux, uy, dur = msg.data[:3]
        self.disturb_u = np.array([ux, uy])
        self.disturb_until = self.get_clock().now() + Duration(seconds=dur)
        self.get_logger().warn(f'PERTURBACIÓN: u=({ux:.2f}, {uy:.2f}) durante {dur:.1f} s')

    # --- Bucle de control -------------------------------------------------------
    def _step(self):
        pose = self.robot.pose()
        if pose is None:
            self.get_logger().warn('Esperando TF map -> ' + self.robot.base_frame,
                                   throttle_duration_sec=5.0)
            return
        x, y, th = pose
        now = self.get_clock().now()
        xi_i = np.array(control_point(x, y, th)) - self.delta

        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y = float(xi_i[0]), float(xi_i[1])
        self.state_pub.publish(msg)

        if not self.enabled:
            return

        if self.disturb_until is not None and now < self.disturb_until:
            u = self.disturb_u
        else:
            fresh = [xi for xi, t in self.xi_nb.values() if now - t < self.stale]
            if len(fresh) < len(self.neighbors):
                self.get_logger().warn(f'Solo {len(fresh)} de {len(self.neighbors)} vecinos '
                                       'con datos recientes', throttle_duration_sec=2.0)
            u = np.asarray(consensus_law.consensus_velocity(xi_i, fresh, self.k), dtype=float)

        ux, uy = saturate(float(u[0]), float(u[1]))
        if np.hypot(ux, uy) < self.deadband:
            self.robot.stop()
            return
        v, w = integrator_to_unicycle(ux, uy, th)
        self.robot.send(v, w)
        if self.robot.safety.blocked:
            self.get_logger().warn('Filtro de seguridad: obstáculo delante, avance anulado',
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
