"""PRÁCTICA 2 — Subasta de tareas.  >>> ESTE ES EL FICHERO QUE PROGRAMÁIS <<<

Solo hay que completar las cuatro funciones marcadas con TODO. El resto del paquete
(nodos ROS, protocolo de mensajes, navegación, registro y gráficas) ya está hecho y las usa.

Comprobación rápida, sin ROS ni Gazebo:
    cd ~/ws/src/robcoop_lab
    python3 -m pytest test/test_auction_law.py -v
    ros2 run robcoop_lab check_p2

Notación (ver el guion):
    punto, start   tupla o array (x, y) en metros, en el marco 'map'
    ruta           lista ordenada de puntos que el robot visitará uno tras otro
    puja           (coste, robot, tarea): coste que declara 'robot' por quedarse 'tarea'
"""
import numpy as np


def route_length(start, route):
    """Longitud del camino que sale de 'start' y visita los puntos de 'route' en orden.

        L = |start - r_0| + |r_0 - r_1| + ... + |r_{n-2} - r_{n-1}|

    start: (x, y)
    route: lista de (x, y) (puede estar vacía: entonces L = 0)
    Devuelve: float, en metros. El robot NO vuelve al inicio.
    """
    # TODO (1): calcular y devolver la longitud del camino
    return 0.0


def best_insertion(start, route, task):
    """Mejor sitio de la ruta para insertar 'task' y cuánto se alarga la ruta.

    Se prueban todas las posiciones k = 0, 1, ..., len(route) (k = len(route) es añadirla al
    final) y se devuelve la que menos alarga el camino:

        coste_extra(k) = L(route con task insertada en k) - L(route)

    Devuelve: (coste_extra, k). Si hay empate, la k más pequeña.
    Pista: route[:k] + [task] + route[k:] es la ruta con la tarea insertada en k.
    """
    # TODO (2): probar todas las posiciones y devolver (coste_extra mínimo, k)
    return 0.0, len(route)


def choose_winner(bids):
    """Puja ganadora de una ronda: la de MENOR coste.

    bids: lista de tuplas (coste, robot, tarea), p. ej. [(1.2, 'tb3_0', 'P3'), ...]
          Las pujas con coste infinito (math.inf) no son válidas.
    Devuelve: la tupla ganadora, o None si no hay ninguna válida.
    Empates: gana el robot con el nombre menor y, si sigue el empate, la tarea con el
    nombre menor (así todos los nodos llegarían al mismo resultado).
    Pista: Python compara tuplas elemento a elemento.
    """
    # TODO (3): devolver la puja ganadora
    return None


def hungarian(cost):
    """Asignación óptima centralizada de UNA tarea por robot (algoritmo húngaro).

    cost: matriz R x T (lista de listas o array), cost[i][j] = coste de que el robot i
          haga la tarea j. Puede no ser cuadrada (más tareas que robots o al revés).
    Devuelve: lista de pares (i, j) ordenada por i, con la asignación que minimiza
              la suma de costes (cada robot y cada tarea aparecen como mucho una vez).
    Pista: scipy.optimize.linear_sum_assignment hace el trabajo; hay que convertir su
           resultado en una lista de pares de enteros de Python.
    """
    cost = np.asarray(cost, dtype=float)
    # TODO (4): resolver con linear_sum_assignment y devolver la lista de pares
    return []
