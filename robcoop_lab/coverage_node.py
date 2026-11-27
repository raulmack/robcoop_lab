"""Nodo de cobertura de UN robot (proporcionado; la ley está en coverage_law.py).

Se lanza uno por robot, en su namespace. Cada nodo:
  - lee la pose de SU robot (TF) y publica su posición (el punto de control) en
    /<robot>/coverage/state. Ese mensaje hace también de LATIDO: mientras llega, el robot
    está activo;
  - escucha /<otro>/coverage/state de los demás. Si de uno no llega nada durante
    'stale_time' segundos, lo da por caído y deja de contar con él;
  - cuando /coverage/enable es True calcula, con SU posición y la de los robots activos, su
    celda de Voronoi, el centroide y u_i = k (C_i − p_i) (coverage_law.py), lo convierte en
    (v, ω) y lo envía al robot a través del filtro de seguridad;
  - publica su centroide en /<robot>/coverage/centroid;
  - /<robot>/coverage/fail (True) simula una avería: el robot se para y deja de publicar;
    con False vuelve a funcionar.

Nadie le dice a un robot cuál es su celda: la calcula él con lo que oye. Es distribuido.
Parámetros: others, k, rate, stale_time, density, center, sigma, peak.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from . import coverage_core as core
from . import coverage_law
from .kinematics import control_point
from .robot_api import RobotInterface

ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CoverageNode(Node):

    def __init__(self):
        super().__init__('coverage_node')
        p = self.declare_parameter
        p('others', [''])                       # los demás robots del equipo
        p('k', core.K_DEFAULT)
        p('rate', 10.0)
        p('stale_time', core.STALE)             # s sin noticias -> robot caído
        p('density', 'uniforme')
        p('center', list(core.ZONE_CENTER))
        p('sigma', core.ZONE_SIGMA)
        p('peak', core.ZONE_PEAK)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.ns = self.get_namespace().strip('/')
        self.others = [n for n in g('others') if n and n != self.ns]
        self.k = float(g('k'))
        self.stale = Duration(seconds=float(g('stale_time')))
        self.points, self.cell_area = core.make_grid()
        self.weights = core.density(g('density'), self.points, tuple(g('center')),
                                    float(g('sigma')), float(g('peak')))

        self.robot = RobotInterface(self, self.ns)
        self.enabled = False
        self.failed = False
        self.q_other = {}                       # robot -> (posición, instante de recepción)
        self.active = None                      # robots con los que cuenta ahora mismo

        self.state_pub = self.create_publisher(PointStamped, 'coverage/state', 10)
        self.centroid_pub = self.create_publisher(PointStamped, 'coverage/centroid', 10)
        for other in self.others:
            self.create_subscription(PointStamped, f'/{other}/coverage/state',
                                     lambda msg, r=other: self._on_other(r, msg), 10)
        self.create_subscription(Bool, '/coverage/enable', self._on_enable, ENABLE_QOS)
        self.create_subscription(Bool, 'coverage/fail', self._on_fail, 10)
        self.create_timer(1.0 / float(g('rate')), self._step)
        self.get_logger().info(f'{self.ns}: cobertura con {self.others}, densidad '
                               f'{g("density")}, k={self.k}')

    # --- Callbacks ------------------------------------------------------------------------
    def _on_other(self, r, msg):
        self.q_other[r] = (np.array([msg.point.x, msg.point.y]), self.get_clock().now())

    def _on_enable(self, msg):
        if msg.data == self.enabled:
            return
        if not msg.data:
            self.robot.stop()
        self.enabled = msg.data
        self.get_logger().info('cobertura ACTIVADA' if msg.data else 'cobertura PARADA')

    def _on_fail(self, msg):
        if msg.data == self.failed:
            return
        self.failed = msg.data
        self.robot.stop()
        if self.failed:
            self.get_logger().error('AVERÍA simulada: me paro y dejo de publicar mi posición')
        else:
            self.get_logger().warn('Avería reparada: vuelvo al equipo')

    def _point(self, xy, stamp):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y = float(xy[0]), float(xy[1])
        return msg

    # --- Bucle de control -----------------------------------------------------------------
    def _step(self):
        if self.failed:
            return
        pose = self.robot.pose()
        if pose is None:
            self.get_logger().warn('Esperando TF map -> ' + self.robot.base_frame,
                                   throttle_duration_sec=5.0)
            return
        now = self.get_clock().now()
        stamp = now.to_msg()
        self.state_pub.publish(self._point(control_point(*pose), stamp))   # posición y latido
        if not self.enabled:
            return

        # Solo se cuenta con los robots de los que hay noticias recientes
        fresh = {r: q for r, (q, t) in self.q_other.items() if now - t < self.stale}
        active = sorted(fresh)
        if active != self.active:
            if self.active is not None:
                for r in sorted(set(self.active) - set(active)):
                    self.get_logger().warn(f'{r} no responde: lo excluyo y recalculo mi celda')
                for r in sorted(set(active) - set(self.active)):
                    self.get_logger().info(f'{r} vuelve a responder: cuento con él')
            self.active = active

        v, w, c, _ = core.robot_command(coverage_law, self.points, self.weights, pose,
                                        [fresh[r] for r in active], self.k)
        if c is None:
            self.robot.stop()
            self.get_logger().warn('Sin centroide (¿centroid sin programar o celda vacía?): '
                                   'parado', throttle_duration_sec=5.0)
            return
        self.centroid_pub.publish(self._point(c, stamp))
        if v == 0.0 and w == 0.0:
            self.robot.stop()
            return
        self.robot.send(v, w)
        if self.robot.safety.blocked:
            self.get_logger().warn('Filtro de seguridad: obstáculo delante, avance anulado',
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001  (al cerrar con Ctrl+C a veces salta un error interno)
        if rclpy.ok():
            raise
    finally:
        try:
            node.robot.stop()       # el robot real NO se para solo si deja de recibir órdenes
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
