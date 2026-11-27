"""Agente de UN robot para el reto final (proporcionado).

En las prácticas 1 a 3 cada robot tenía un nodo con un solo comportamiento. En el reto el
mismo robot tiene que hacer las tres cosas, una detrás de otra, así que este programa arranca
en un único proceso los tres nodos de las prácticas, SIN CAMBIARLOS:

    consensus_node   formación por consenso        (usa vuestro consensus_law.py)
    auction_node     pujas y recorrido de la ruta  (usa vuestro auction_law.py)
    coverage_node    despliegue de cobertura       (usa vuestro coverage_law.py)

Los tres están siempre vivos y publican su estado (y sus latidos), pero solo mueve el robot
el comportamiento que el árbitro de la misión (mission_node.py) tiene activado en cada fase:

    fase 1  /consensus/enable = True
    fase 2  /auction/announce ... /auction/start
    fases 3 y 4  /coverage/enable = True

Se lanza uno por robot, en su namespace, desde p4_reto.launch.py, que le pasa los parámetros
de cada nodo a partir de reto_config.py.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from .auction_node import AuctionNode
from .consensus_node import ConsensusNode
from .coverage_node import CoverageNode


def main(args=None):
    rclpy.init(args=args)
    nodes = [ConsensusNode(), AuctionNode(), CoverageNode()]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001  (al cerrar con Ctrl+C a veces salta un error interno)
        time.sleep(0.3)             # si era el cierre, el contexto ya está apagado
        if rclpy.ok():
            raise
    finally:
        for node in nodes:
            try:
                node.robot.stop()   # el robot real NO se para solo si deja de recibir órdenes
            except Exception:
                pass
        for node in nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
