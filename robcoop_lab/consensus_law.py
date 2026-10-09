"""PRÁCTICA 1 — Ley de consenso.  >>> ESTE ES EL FICHERO QUE PROGRAMÁIS <<<

Solo hay que completar las tres funciones marcadas con TODO. El resto del paquete
(nodos ROS, cinemática, seguridad, registro y gráficas) ya está hecho y las usa.

Comprobación rápida, sin ROS ni Gazebo:
    cd ~/ws/src/robcoop_lab
    python3 -m pytest test/test_consensus_law.py -v

Notación (ver el guion):
    A    matriz de adyacencia (N x N), A[i][j] = 1 si i y j son vecinos
    L    laplaciano del grafo
    ξ_i  variable de consenso del robot i:  ξ_i = q_i - δ_i
         (q_i: punto de control del robot, δ_i: su offset en la formación)
"""
import numpy as np


def laplacian(A):
    """Devuelve el laplaciano L = D - A, siendo D la matriz diagonal de grados.

    A: array de numpy N x N (simétrica, ceros en la diagonal).
    Devuelve: array de numpy N x N de tipo float.
    """
    A = np.asarray(A, dtype=float)
    # TODO (1): calcular y devolver L
    return np.zeros_like(A)


def algebraic_connectivity(L):
    """Devuelve λ2, el segundo autovalor más pequeño del laplaciano.

    Pista: L es simétrica, así que np.linalg.eigvalsh devuelve autovalores reales
    ordenados de menor a mayor.
    """
    L = np.asarray(L, dtype=float)
    # TODO (2): calcular y devolver λ2 como float
    return 0.0


def consensus_velocity(xi_i, xi_neighbors, k):
    """Velocidad deseada del robot i según la ley de consenso:

        u_i = -k · Σ_{j ∈ N_i} (ξ_i - ξ_j)

    xi_i:          array (2,) con la variable de consenso propia [x, y]
    xi_neighbors:  lista de arrays (2,) con las de los vecinos (puede estar vacía)
    k:             ganancia (> 0)
    Devuelve:      array (2,) con [u_x, u_y] en m/s (sin saturar: eso ya se hace fuera)
    """
    xi_i = np.asarray(xi_i, dtype=float)
    u = np.zeros(2)
    # TODO (3): calcular u a partir de xi_i, xi_neighbors y k
    return u
