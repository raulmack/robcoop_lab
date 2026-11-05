"""Configuración de RViz para la Práctica 2 (proporcionado).

La de turtlebot3_gz_bringup (modelo y scan de cada robot) más los marcadores de la subasta
(/auction/markers): tareas con el color del robot que las ha ganado y la ruta de cada uno.
"""
from turtlebot3_gz_bringup import sim_utils

MARKERS = (
    '    - Class: rviz_default_plugins/MarkerArray\n'
    '      Name: subasta\n'
    '      Enabled: true\n'
    '      Topic:\n'
    '        Value: /auction/markers\n'
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
