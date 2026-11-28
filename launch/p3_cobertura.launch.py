#!/usr/bin/env python3
"""Práctica 3 — un experimento de cobertura (simulación o robots reales).

Lanza un coverage_node por robot (cada uno calcula su celda de Voronoi con su posición y la
de los demás) y el nodo que dirige y registra el experimento. Termina solo cuando el equipo
se ha desplegado o al agotar el tiempo.

    ros2 launch robcoop_lab p3_cobertura.launch.py                       # densidad uniforme
    ros2 launch robcoop_lab p3_cobertura.launch.py k:=0.1 nombre:=sim_k01
    ros2 launch robcoop_lab p3_cobertura.launch.py densidad:=zona
    ros2 launch robcoop_lab p3_cobertura.launch.py densidad:=zona centro:="-0.5,0.5" sigma:=0.4
    ros2 launch robcoop_lab p3_cobertura.launch.py densidad:=zona centro:="0.6,-0.6;-0.8,0.8"
    ros2 launch robcoop_lab p3_cobertura.launch.py fallo:=tb3_1
    ros2 launch robcoop_lab p3_cobertura.launch.py modo:=real nombre:=real_uniforme

Argumentos:
    robots      tb3_0,tb3_1,tb3_2     (2 a 4, en el orden de las marcas)
    densidad    uniforme | zona       (zona: hay una zona de interés, ver coverage_core.py)
    centro      "x,y" de la zona de interés (0.6,-0.6); varias: "x1,y1;x2,y2"
    sigma       tamaño de la zona de interés en m (0.5)
    pico        importancia del centro de la zona respecto al resto (12)
    k           ganancia de u_i = k (C_i − p_i) (0.5)
    fallo       robot que se avería cuando el equipo ya se ha desplegado ('' = ninguno)
    modo        sim | real
    salida      auto | libre  (auto: comprueba que salen de las marcas o de 'agrupados';
                               libre: salen de donde estén, sin comprobar)
    nombre      nombre de los ficheros de resultados (por defecto se genera)
    timeout     s máximos del experimento (120)
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robcoop_lab import coverage_core


def setup(context, *args, **kwargs):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    robots = [r.strip().strip('/') for r in arg('robots').split(',') if r.strip()]
    mode, dens, start = arg('modo'), arg('densidad'), arg('salida')
    if mode not in ('sim', 'real'):
        raise ValueError('modo debe ser sim o real')
    if dens not in coverage_core.DENSITIES:
        raise ValueError(f'densidad debe ser una de {coverage_core.DENSITIES}')
    if start not in ('auto', 'libre'):
        raise ValueError('salida debe ser auto o libre (la posición de salida se elige en '
                         'p3_sim o p3_real)')
    fail = arg('fallo').strip().strip('/')
    if fail and fail not in robots:
        raise ValueError(f'fallo debe ser uno de los robots: {", ".join(robots)}')
    center = coverage_core.parse_centers(arg('centro'))          # [x1, y1, x2, y2, ...]
    zone = {'density': dens, 'center': center, 'sigma': float(arg('sigma')),
            'peak': float(arg('pico'))}
    k = float(arg('k'))
    sim = mode == 'sim'

    actions = []
    localhost = os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1'
    if sim and not localhost:
        actions.append(LogInfo(msg='\033[1;33m[robcoop] AVISO: simulación SIN modo_sim: '
                                   'tus órdenes pueden llegar a los robots reales del '
                                   'laboratorio. Ejecuta modo_sim.\033[0m'))
    if not sim and localhost:
        actions.append(LogInfo(msg='\033[1;31m[robcoop] modo:=real pero la terminal está en '
                                   'modo_sim: no se verán los robots. Ejecuta modo_real.\033[0m'))

    for r in robots:
        actions.append(Node(
            package='robcoop_lab', executable='coverage_node', namespace=r,
            name='coverage_node', output='screen',
            parameters=[{'others': [o for o in robots if o != r] or [''], 'k': k,
                         'use_sim_time': sim, **zone}]))

    experiment = Node(
        package='robcoop_lab', executable='coverage_experiment', name='coverage_experiment',
        output='screen',
        parameters=[{'robots': robots, 'k': k, 'mode': mode, 'start': start,
                     'name': arg('nombre'), 'timeout': float(arg('timeout')), 'fail': fail,
                     'use_sim_time': sim, **zone}])
    actions.append(experiment)
    # Cuando el experimento termina, se cierra todo (los nodos paran sus robots al salir)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=experiment, on_exit=[EmitEvent(event=Shutdown(reason='fin'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='tb3_0,tb3_1,tb3_2'),
        DeclareLaunchArgument('densidad', default_value='uniforme'),
        DeclareLaunchArgument('centro', default_value='0.6,-0.6'),
        DeclareLaunchArgument('sigma', default_value='0.5'),
        DeclareLaunchArgument('pico', default_value='12.0'),
        DeclareLaunchArgument('k', default_value='0.5'),
        DeclareLaunchArgument('fallo', default_value=''),
        DeclareLaunchArgument('modo', default_value='sim'),
        DeclareLaunchArgument('salida', default_value='auto'),
        DeclareLaunchArgument('nombre', default_value=''),
        DeclareLaunchArgument('timeout', default_value='120.0'),
        OpaqueFunction(function=setup),
    ])
