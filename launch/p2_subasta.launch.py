#!/usr/bin/env python3
"""Práctica 2 — un experimento de subasta (simulación o robots reales).

Lanza un auction_node por robot (cada uno calcula sus pujas con su posición y su ruta) y el
subastador, que reparte las tareas, da la salida y registra el experimento. Termina solo al
completar todas las tareas o al agotar el tiempo.

    ros2 launch robcoop_lab p2_subasta.launch.py                          # disposición A, suma
    ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=B criterio:=max
    ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=aleatoria semilla:=4
    ros2 launch robcoop_lab p2_subasta.launch.py puntos:="1.0,0.5; -0.5,1.2; 0,-1"
    ros2 launch robcoop_lab p2_subasta.launch.py modo:=real nombre:=real_A_suma

Argumentos:
    robots       tb3_0,tb3_1,tb3_2      (2 a 4, en el orden de las marcas)
    disposicion  A | B | C | aleatoria  (ver tasks.py)
    puntos       "x,y; x,y; ..."        (sustituye a la disposición)
    semilla      para la disposición aleatoria (0)
    criterio     suma | max
    modo         sim | real
    nombre       nombre de los ficheros de resultados (por defecto se genera)
    timeout      s máximos de ejecución (300)
    ejecutar     true | false  (false: solo la subasta, los robots no se mueven)
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robcoop_lab import auction_core, tasks


def setup(context, *args, **kwargs):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    robots = [r.strip().strip('/') for r in arg('robots').split(',') if r.strip()]
    mode, criterion = arg('modo'), arg('criterio')
    if mode not in ('sim', 'real'):
        raise ValueError('modo debe ser sim o real')
    if criterion not in auction_core.CRITERIA:
        raise ValueError(f'criterio debe ser uno de {auction_core.CRITERIA}')
    sim = mode == 'sim'
    task_xy = tasks.get(arg('disposicion'), arg('puntos'), int(arg('semilla')))
    layout = 'puntos' if arg('puntos').strip() else arg('disposicion')
    if layout == 'aleatoria':
        layout = f'aleatoria{arg("semilla")}'

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
            package='robcoop_lab', executable='auction_node', namespace=r,
            name='auction_node', output='screen',
            parameters=[{'criterion': criterion, 'priority': i, 'use_sim_time': sim}]))

    auctioneer = Node(
        package='robcoop_lab', executable='auctioneer_node', name='auctioneer',
        output='screen',
        parameters=[{'robots': robots, 'task_names': list(task_xy),
                     'task_xy': [float(c) for xy in task_xy.values() for c in xy],
                     'layout': layout, 'criterion': criterion, 'mode': mode,
                     'name': arg('nombre'), 'timeout': float(arg('timeout')),
                     'execute': arg('ejecutar').lower() in ('true', '1'),
                     'use_sim_time': sim}])
    actions.append(auctioneer)
    # Cuando el subastador termina, se cierra todo (los nodos paran sus robots al salir)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=auctioneer, on_exit=[EmitEvent(event=Shutdown(reason='fin'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='tb3_0,tb3_1,tb3_2'),
        DeclareLaunchArgument('disposicion', default_value='A'),
        DeclareLaunchArgument('puntos', default_value=''),
        DeclareLaunchArgument('semilla', default_value='0'),
        DeclareLaunchArgument('criterio', default_value='suma'),
        DeclareLaunchArgument('modo', default_value='sim'),
        DeclareLaunchArgument('nombre', default_value=''),
        DeclareLaunchArgument('timeout', default_value='300.0'),
        DeclareLaunchArgument('ejecutar', default_value='true'),
        OpaqueFunction(function=setup),
    ])
