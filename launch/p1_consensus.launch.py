#!/usr/bin/env python3
"""Práctica 1 — un experimento de consenso (simulación o robots reales).

Lanza un consensus_node por robot (cada uno solo escucha a sus vecinos) y el nodo que
dirige y registra el experimento. Termina solo al converger o al agotar el tiempo.

    ros2 launch robcoop_lab p1_consensus.launch.py topologia:=anillo
    ros2 launch robcoop_lab p1_consensus.launch.py topologia:=completo formacion:=linea k:=0.2
    ros2 launch robcoop_lab p1_consensus.launch.py formacion:=poligono perturbar:=tb3_1
    ros2 launch robcoop_lab p1_consensus.launch.py modo:=real robots:=tb3_0,tb3_1,tb3_2

Argumentos:
    robots      tb3_0,tb3_1,tb3_2,tb3_3     (3 o 4, en el orden de las marcas)
    topologia   completo | anillo | cadena | estrella
    formacion   reunion | linea | poligono | triangulo
    k           ganancia (0.1)
    modo        sim | real
    nombre      nombre de los ficheros de resultados (por defecto se genera)
    timeout     s máximos del experimento (120)
    perturbar   robot al que se empuja 3 s tras converger ('' = ninguno)
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robcoop_lab import formations, graphs


def setup(context, *args, **kwargs):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    robots = [r.strip().strip('/') for r in arg('robots').split(',') if r.strip()]
    topology, formation, mode = arg('topologia'), arg('formacion'), arg('modo')
    k = float(arg('k'))
    sim = mode == 'sim'
    if mode not in ('sim', 'real'):
        raise ValueError('modo debe ser sim o real')

    A = graphs.adjacency(topology, len(robots))
    nbs = graphs.neighbors(A, robots)
    offs = formations.offsets(formation, len(robots))

    actions = []
    localhost = os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1'
    if sim and not localhost:
        actions.append(LogInfo(msg='\033[1;33m[robcoop] AVISO: simulación SIN modo_sim: '
                                   'tus órdenes pueden llegar a los robots reales del '
                                   'laboratorio. Ejecuta modo_sim.\033[0m'))
    if not sim and localhost:
        actions.append(LogInfo(msg='\033[1;31m[robcoop] modo:=real pero la terminal está en '
                                   'modo_sim: no se verán los robots. Ejecuta modo_real.\033[0m'))

    for i, r in enumerate(robots):
        actions.append(Node(
            package='robcoop_lab', executable='consensus_node', namespace=r,
            name='consensus_node', output='screen',
            parameters=[{'neighbors': nbs[r], 'offset': list(offs[i]), 'k': k,
                         'use_sim_time': sim}]))

    experiment = Node(
        package='robcoop_lab', executable='experiment_node', name='consensus_experiment',
        output='screen',
        parameters=[{'robots': robots, 'offsets': [c for o in offs for c in o],
                     'topology': topology, 'formation': formation, 'k': k, 'mode': mode,
                     'name': arg('nombre'), 'timeout': float(arg('timeout')),
                     'disturb': arg('perturbar'), 'use_sim_time': sim}])
    actions.append(experiment)
    # Cuando el experimento termina, se cierra todo (los nodos paran sus robots al salir)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=experiment, on_exit=[EmitEvent(event=Shutdown(reason='fin'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='tb3_0,tb3_1,tb3_2,tb3_3'),
        DeclareLaunchArgument('topologia', default_value='completo'),
        DeclareLaunchArgument('formacion', default_value='reunion'),
        DeclareLaunchArgument('k', default_value='0.1'),
        DeclareLaunchArgument('modo', default_value='sim'),
        DeclareLaunchArgument('nombre', default_value=''),
        DeclareLaunchArgument('timeout', default_value='120.0'),
        DeclareLaunchArgument('perturbar', default_value=''),
        OpaqueFunction(function=setup),
    ])
