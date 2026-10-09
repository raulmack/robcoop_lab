"""Disposición de la arena (proporcionado; el profesor la ajusta a la arena real).

Origen de 'map' en el centro de la arena, eje x hacia la pared marcada con «+x».
Las marcas de salida del suelo coinciden con las poses de aparición en simulación,
así los resultados de simulación y real se pueden comparar directamente.
"""
# (x, y, yaw) de la marca de cada robot: tb3_0, tb3_1, tb3_2, tb3_3 (mirando al centro)
START_POSES = [(1.2, 1.0, -2.4), (-1.0, 1.2, -0.9), (-1.2, -1.0, 0.7), (1.0, -1.2, 2.2)]


def poses_arg(n):
    """Cadena para el argumento 'poses' de multi_robot.launch.py."""
    return '; '.join(f'{x},{y},{yaw}' for x, y, yaw in START_POSES[:n])
