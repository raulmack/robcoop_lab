"""Parada de emergencia por software (proporcionado).

    ros2 run robcoop_lab parar                      # tb3_0..tb3_3
    ros2 run robcoop_lab parar tb3_0 tb3_2

Desactiva el consenso, para la subasta y envía velocidad cero a los robots durante 1 s.
OJO: un TurtleBot3 real sigue con la última velocidad recibida si su nodo de control
muere. Si algo va mal, esto primero; si no basta, levantar el robot o apagarlo.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .auction_node import AUCTION_QOS
from .consensus_node import ENABLE_QOS


def main():
    robots = [a for a in sys.argv[1:] if not a.startswith('-')] or \
        ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    rclpy.init()
    node = rclpy.create_node('robcoop_stop_all')
    enable = node.create_publisher(Bool, '/consensus/enable', ENABLE_QOS)
    auction = node.create_publisher(Bool, '/auction/stop', AUCTION_QOS)
    pubs = [node.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in robots]
    end = time.time() + 1.0
    while time.time() < end:
        enable.publish(Bool(data=False))
        auction.publish(Bool(data=True))
        for p in pubs:
            p.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.1)
    print('Parada enviada a: ' + ', '.join(robots))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
