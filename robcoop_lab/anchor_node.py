"""Ancla los robots REALES al marco común 'map' (proporcionado).

Los robots reales solo tienen odometría: cada uno mide su pose en su propio marco
tb3_i/odom, cuyo origen es donde estaba al arrancar. Para que todos compartan el
marco 'map' (el de la arena), se colocan sobre sus marcas del suelo y este nodo:

  1. lee la odometría actual de cada robot  (pose O en tb3_i/odom);
  2. conoce la pose de su marca              (pose M en map, de config/p1.yaml);
  3. publica la TF estática map -> tb3_i/odom = M · O⁻¹

Así no hace falta reiniciar los robots: basta con volver a colocarlos en sus marcas
y relanzar p1_real.launch.py. Desde ese momento la pose en 'map' se va degradando con
la deriva de la odometría (unos centímetros por minuto).
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster

from .robot_api import yaw_from_quaternion


class AnchorNode(Node):

    def __init__(self):
        super().__init__('anchor_node')
        self.declare_parameter('robots', ['tb3_0'])
        self.declare_parameter('marks', [0.0, 0.0, 0.0])     # x0, y0, yaw0, x1, ...
        self.robots = list(self.get_parameter('robots').value)
        marks = list(self.get_parameter('marks').value)
        if len(marks) < 3 * len(self.robots):
            raise ValueError('Faltan marcas: hacen falta x, y, yaw por robot')
        self.marks = {r: marks[3 * i:3 * i + 3] for i, r in enumerate(self.robots)}
        self.broadcaster = StaticTransformBroadcaster(self)
        self.anchored = {}
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom',
                                     lambda m, r=r: self._on_odom(r, m), qos_profile_sensor_data)
        self.create_timer(3.0, self._report)
        self.get_logger().info('Coloca cada robot en su marca. Esperando odometría de: '
                               + ', '.join(self.robots))

    def _on_odom(self, r, msg):
        if r in self.anchored:
            return
        p = msg.pose.pose
        xo, yo, tho = p.position.x, p.position.y, yaw_from_quaternion(p.orientation)
        xm, ym, thm = self.marks[r]
        th = thm - tho
        c, s = math.cos(th), math.sin(th)
        tx = xm - (c * xo - s * yo)
        ty = ym - (s * xo + c * yo)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{r}/odom'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.rotation.z = math.sin(th / 2)
        t.transform.rotation.w = math.cos(th / 2)
        self.anchored[r] = t
        # se reenvían todas juntas: /tf_static guarda el último mensaje de cada publicador
        self.broadcaster.sendTransform(list(self.anchored.values()))
        self.get_logger().info(f'{r} anclado en la marca ({xm:.2f}, {ym:.2f}, '
                               f'{math.degrees(thm):.0f}°)')

    def _report(self):
        missing = [r for r in self.robots if r not in self.anchored]
        if missing:
            self.get_logger().warn('Sin odometría todavía: ' + ', '.join(missing)
                                   + '  (¿robot_up.sh lanzado? ¿modo_real?)')


def main(args=None):
    rclpy.init(args=args)
    node = AnchorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
