"""Parada de emergencia por software (proporcionado).

    ros2 run robcoop_lab parar                      # tb3_0..tb3_3
    ros2 run robcoop_lab parar tb3_0 tb3_2

Desactiva el consenso y la cobertura, para la subasta, termina la misión del reto y envía
velocidad cero a los robots durante al menos 2 s.
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
from .coverage_node import ENABLE_QOS as COVERAGE_QOS


def main():
    robots = [a for a in sys.argv[1:] if not a.startswith('-')] or \
        ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    rclpy.init()
    node = rclpy.create_node('robcoop_stop_all')
    enable = node.create_publisher(Bool, '/consensus/enable', ENABLE_QOS)
    auction = node.create_publisher(Bool, '/auction/stop', AUCTION_QOS)
    coverage = node.create_publisher(Bool, '/coverage/enable', COVERAGE_QOS)
    mission = node.create_publisher(Bool, '/reto/stop', AUCTION_QOS)
    pubs = [node.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in robots]
    # Se publica al menos 2 s y, si siguen apareciendo nodos que escuchan (con el ordenador
    # cargado tardan en descubrir a este), hasta 1 s después del último; como mucho 6 s.
    everyone = [enable, auction, coverage, mission] + pubs
    start = last_change = time.time()
    heard = -1
    while True:
        now = time.time()
        count = sum(p.get_subscription_count() for p in everyone)
        if count != heard:
            heard, last_change = count, now
        if now - start > 6.0 or (now - start > 2.0 and now - last_change > 1.0):
            break
        enable.publish(Bool(data=False))
        auction.publish(Bool(data=True))
        coverage.publish(Bool(data=False))
        mission.publish(Bool(data=True))
        for p in pubs:
            p.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    print('Parada enviada a: ' + ', '.join(robots))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
