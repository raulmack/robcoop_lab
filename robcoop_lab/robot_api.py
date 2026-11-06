"""Interfaz con un robot, igual en simulación y en real (proporcionado).

    robot = RobotInterface(node, 'tb3_0')
    pose = robot.pose()          # (x, y, θ) en el marco 'map', o None si aún no hay TF
    robot.send(v, w)             # pasa por el filtro de seguridad y publica /tb3_0/cmd_vel
    robot.stop()

La pose sale de TF:  map -> tb3_i/odom -> tb3_i/base_footprint
  - simulación: map -> tb3_i/odom lo publica multi_robot.launch.py (pose de aparición)
  - real:       lo publica anchor_node (p1_real.launch.py) a partir de las marcas del suelo
"""
import math

from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .safety import SafetyFilter, scan_is_degenerate

MAP_FRAME = 'map'


def _tf_buffer(node):
    """Un único buffer de TF por nodo, aunque se creen varias interfaces."""
    if not hasattr(node, '_robcoop_tf_buffer'):
        node._robcoop_tf_buffer = Buffer()
        node._robcoop_tf_listener = TransformListener(node._robcoop_tf_buffer, node)
    return node._robcoop_tf_buffer


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotInterface:

    def __init__(self, node, ns, command=True):
        """command=False: solo lectura de pose (para observadores como el registro)."""
        self.node = node
        self.ns = ns.strip('/')
        self.base_frame = f'{self.ns}/base_footprint'
        self.tf = _tf_buffer(node)
        self.safety = SafetyFilter()
        self._cmd_pub = None
        if command:
            self._cmd_pub = node.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
            node.create_subscription(LaserScan, f'/{self.ns}/scan', self._on_scan,
                                     qos_profile_sensor_data)

    def _on_scan(self, msg):
        self.safety.update_scan(msg)
        if scan_is_degenerate(msg):
            self.node.get_logger().warn(
                f'{self.ns}: el LiDAR marca {msg.range_min:.2f} m en todos los rayos, es decir, '
                'NO está midiendo (no se esquivarán obstáculos). En simulación: cerrad la '
                'simulación y relanzadla con render:=software. Con el robot real: avisad al '
                'profesor.', throttle_duration_sec=10.0)

    def pose(self):
        """(x, y, θ) del robot en 'map' con la última TF disponible, o None."""
        try:
            t = self.tf.lookup_transform(MAP_FRAME, self.base_frame, Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)

    def send(self, v, w):
        """Envía (v, ω) tras el filtro de seguridad. Devuelve lo que se ha enviado."""
        v, w = self.safety.filter(v, w)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)
        return v, w

    def stop(self):
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())
