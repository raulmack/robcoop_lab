# robcoop_lab

Paquete base de las prácticas de **Robótica Cooperativa** (Máster en Automática y Robótica, UCLM)
con TurtleBot3 Burger, ROS 2 Humble y Gazebo Fortress.

Es una **plantilla**: cada pareja crea su repositorio privado con *Use this template*, lo clona en
`~/robcoop-docker/ws/src/robcoop_lab` y programa ahí sus prácticas. El entorno (contenedor con
ROS 2 y la simulación multi-robot) está en
[robcoop-docker](https://github.com/raulmack/robcoop-docker).

| Práctica | Guion | Qué se programa |
|---|---|---|
| P1 · Formación por consenso | [docs/P1_consenso.md](docs/P1_consenso.md) | `robcoop_lab/consensus_law.py` |
| P2 · Asignación de tareas por subasta | [docs/P2_subasta.md](docs/P2_subasta.md) | `robcoop_lab/auction_law.py` |
| P3 · Integración y reto final | (próximamente) | |

## Estructura

```
robcoop_lab/          código Python (nodos, ley de consenso, utilidades)
launch/               p1_sim, p1_consensus, p1_real, p2_sim, p2_subasta, p2_real
test/                 pruebas sin ROS:  python3 -m pytest test -v
docs/                 guiones y plantillas de informe
resultados/           CSV, JSON y figuras de vuestros experimentos (se entregan)
```

## Comandos de la P1

```bash
ros2 run robcoop_lab check_p1                                     # comprobación sin ROS
ros2 launch robcoop_lab p1_sim.launch.py                          # 4 robots simulados
ros2 launch robcoop_lab p1_consensus.launch.py topologia:=anillo  # un experimento
ros2 run robcoop_lab plot_p1 resultados/p1/<experimento>.csv      # gráficas
ros2 run robcoop_lab parar                                        # parada de emergencia
```

## Comandos de la P2

```bash
ros2 run robcoop_lab check_p2                                     # comprobación sin ROS
ros2 run robcoop_lab compare_p2                                   # subasta, húngaro y óptimo (sin ROS)
ros2 launch robcoop_lab p2_sim.launch.py                          # 3 robots simulados + RViz
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=B criterio:=max   # un experimento
ros2 run robcoop_lab plot_p2 resultados/p2/<experimento>.csv      # gráficas
```

Si vuestro repositorio se creó antes de la P2, el paso 0 de [docs/P2_subasta.md](docs/P2_subasta.md)
explica cómo traer los ficheros nuevos sin perder vuestro trabajo de la P1.
