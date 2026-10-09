"""Grafos de comunicación del equipo (proporcionado, no hay que modificarlo).

Un grafo se representa por su matriz de adyacencia A (N x N, simétrica, 0/1):
A[i][j] = 1 si el robot i recibe la información del robot j (y viceversa).
"""
import numpy as np

TOPOLOGIES = ('completo', 'anillo', 'cadena', 'estrella')


def adjacency(topology, n):
    """Matriz de adyacencia de una topología con n robots.

    completo : todos con todos
    anillo   : 0-1-2-...-(n-1)-0
    cadena   : 0-1-2-...-(n-1)
    estrella : el robot 0 en el centro, conectado con todos los demás
    """
    if n < 2:
        raise ValueError('Hacen falta al menos 2 robots')
    A = np.zeros((n, n), dtype=int)
    if topology == 'completo':
        A = np.ones((n, n), dtype=int) - np.eye(n, dtype=int)
    elif topology == 'cadena':
        for i in range(n - 1):
            A[i, i + 1] = A[i + 1, i] = 1
    elif topology == 'anillo':
        for i in range(n):
            j = (i + 1) % n
            A[i, j] = A[j, i] = 1
    elif topology == 'estrella':
        A[0, 1:] = 1
        A[1:, 0] = 1
    else:
        raise ValueError(f'Topología desconocida: {topology}. Opciones: {", ".join(TOPOLOGIES)}')
    return A


def neighbors(A, names):
    """Diccionario {robot: [vecinos]} a partir de la matriz de adyacencia."""
    return {names[i]: [names[j] for j in range(len(names)) if A[i, j]]
            for i in range(len(names))}


def lambda2_reference(A):
    """λ2 del laplaciano (lo usa el registro de experimentos; el alumno programa el suyo)."""
    L = np.diag(A.sum(axis=1)) - A
    return float(np.sort(np.linalg.eigvalsh(L))[1])


def describe(topology, n):
    """Texto con la matriz de adyacencia y los vecinos (para 'ros2 run robcoop_lab graph_info')."""
    A = adjacency(topology, n)
    names = [f'tb3_{i}' for i in range(n)]
    lines = [f'Topología {topology} con {n} robots', 'Matriz de adyacencia A:', str(A), 'Vecinos:']
    for k, v in neighbors(A, names).items():
        lines.append(f'  {k}: {", ".join(v)}')
    return '\n'.join(lines)


def main():
    import argparse
    p = argparse.ArgumentParser(description='Muestra la matriz de adyacencia de una topología')
    p.add_argument('topologia', choices=TOPOLOGIES)
    p.add_argument('-n', type=int, default=4, help='número de robots (por defecto 4)')
    a = p.parse_args()
    print(describe(a.topologia, a.n))


if __name__ == '__main__':
    main()
