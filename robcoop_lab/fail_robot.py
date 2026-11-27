"""Avería simulada de un robot durante la cobertura (proporcionado; lo usa el profesor).

    ros2 run robcoop_lab averia tb3_1             # tb3_1 se para y deja de publicar su posición
    ros2 run robcoop_lab averia tb3_1 --reparar   # vuelve a funcionar

El robot no avisa a nadie: los demás tienen que darse cuenta porque dejan de recibir su
latido (/tb3_1/coverage/state) y recolocarse. Es lo que hace el árbitro del reto en la fase 4
(con averia:=profesor espera a que alguien ejecute esto).
"""
import argparse
import time

import rclpy
from std_msgs.msg import Bool


def main():
    p = argparse.ArgumentParser(description='Avería simulada de un robot (cobertura)')
    p.add_argument('robot', help='robot que se avería, p. ej. tb3_1')
    p.add_argument('--reparar', action='store_true', help='el robot vuelve a funcionar')
    a = p.parse_args()
    robot = a.robot.strip().strip('/')
    rclpy.init()
    node = rclpy.create_node('robcoop_averia')
    pub = node.create_publisher(Bool, f'/{robot}/coverage/fail', 10)
    end = time.time() + 1.5
    while time.time() < end:
        pub.publish(Bool(data=not a.reparar))
        rclpy.spin_once(node, timeout_sec=0.1)
    heard = pub.get_subscription_count()
    print(f'{robot}: ' + ('reparado' if a.reparar else 'AVERIADO')
          + ('' if heard else '   (OJO: nadie escucha ese tópico: ¿nombre del robot? '
                              '¿modo_sim / modo_real?)'))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
