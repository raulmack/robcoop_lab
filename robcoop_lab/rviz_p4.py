"""Configuración de RViz para el reto final (proporcionado).

La de turtlebot3_gz_bringup (modelo y scan de cada robot) más los marcadores del árbitro
(/reto/markers): posiciones de la formación, puntos de inspección y rutas, celdas de Voronoi
y centroides, y arriba la fase en curso con el marcador y el tiempo.
"""
from turtlebot3_gz_bringup import sim_utils

MARKERS = (
    '    - Class: rviz_default_plugins/MarkerArray\n'
    '      Name: reto\n'
    '      Enabled: true\n'
    '      Topic:\n'
    '        Value: /reto/markers\n'
    '        Depth: 5\n'
    '        Durability Policy: Transient Local\n'
    '        Reliability Policy: Reliable\n'
    '        History Policy: Keep Last\n')


def rviz_config(robots, fixed_frame='map'):
    path = sim_utils.rviz_config(robots, fixed_frame=fixed_frame)
    with open(path) as f:
        text = f.read()
    text = text.replace('  Enabled: true\n  Global Options:', MARKERS + '  Enabled: true\n'
                        '  Global Options:', 1)
    with open(path, 'w') as f:
        f.write(text)
    return path
