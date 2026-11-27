#!/usr/bin/env python3
"""Reto final — una misión completa (simulación o robots reales).

Lanza un agente por robot (agent_node: sus tres comportamientos de las prácticas 1 a 3, con
vuestras leyes) y el árbitro (mission_node), que activa cada fase, registra y puntúa:

    formación  ->  subasta e inspección  ->  cobertura  ->  avería de un robot

La configuración del equipo NO se pasa por aquí: se lee de robcoop_lab/reto_config.py.
Termina solo y escribe el marcador.

    ros2 launch robcoop_lab p4_reto.launch.py nombre:=sim_reto
    ros2 launch robcoop_lab p4_reto.launch.py disposicion:=aleatoria semilla:=7
    ros2 launch robcoop_lab p4_reto.launch.py averia:=tb3_2
    ros2 launch robcoop_lab p4_reto.launch.py modo:=real nombre:=real_reto_1

Argumentos (el día de la demostración los fija el profesor):
    robots       tb3_0,tb3_1,tb3_2      (3 o 4, en el orden de las marcas)
    disposicion  A | B | C | aleatoria  (puntos de inspección, ver tasks.py)
    puntos       "x,y; x,y; ..."        (sustituye a la disposición)
    semilla      para la disposición aleatoria (0)
    num_puntos   puntos de la disposición aleatoria (6)
    averia       auto | profesor | ninguna | tb3_i
                   auto      el árbitro avería un robot al azar tras el despliegue
                   profesor  espera a que el profesor ejecute «ros2 run robcoop_lab averia tb3_i»
                   tb3_i     se avería ese robot (para comparar simulación y real)
    densidad     uniforme | zona        (zona: con zona de interés, como en la P3)
    centro, sigma, pico                 de la zona de interés
    modo         sim | real
    nombre       nombre de los ficheros de resultados (por defecto se genera)
"""
import json
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robcoop_lab import coverage_core, formations, graphs, mission_core, tasks


def setup(context, *args, **kwargs):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    robots = [r.strip().strip('/') for r in arg('robots').split(',') if r.strip()]
    n = len(robots)
    mode, dens, failure = arg('modo'), arg('densidad'), arg('averia').strip().strip('/')
    if mode not in ('sim', 'real'):
        raise ValueError('modo debe ser sim o real')
    if dens not in coverage_core.DENSITIES:
        raise ValueError(f'densidad debe ser una de {coverage_core.DENSITIES}')
    mission_core.pick_failure(failure, robots)          # solo para validar el argumento
    cfg = mission_core.load_config(n=n)                 # vuestro reto_config.py
    center = [float(v) for v in arg('centro').split(',')[:2]]
    if len(center) != 2:
        raise ValueError('centro debe ser "x,y"')
    zone = {'density': dens, 'center': center, 'sigma': float(arg('sigma')),
            'peak': float(arg('pico'))}
    sim = mode == 'sim'
    task_xy = tasks.get(arg('disposicion'), arg('puntos'), int(arg('semilla')),
                        int(arg('num_puntos')))
    layout = 'puntos' if arg('puntos').strip() else arg('disposicion')
    if layout == 'aleatoria':
        layout = f'aleatoria{arg("semilla")}'

    actions = [LogInfo(msg='[robcoop] reto_config.py: ' + mission_core.describe(cfg))]
    pending = mission_core.unjustified(cfg)
    if pending:
        actions.append(LogInfo(msg='\033[1;33m[robcoop] reto_config.py: falta justificar '
                                   + ', '.join(pending) + ' (POR_QUE). Se evalúa.\033[0m'))
    localhost = os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1'
    if sim and not localhost:
        actions.append(LogInfo(msg='\033[1;33m[robcoop] AVISO: simulación SIN modo_sim: '
                                   'tus órdenes pueden llegar a los robots reales del '
                                   'laboratorio. Ejecuta modo_sim.\033[0m'))
    if not sim and localhost:
        actions.append(LogInfo(msg='\033[1;31m[robcoop] modo:=real pero la terminal está en '
                                   'modo_sim: no se verán los robots. Ejecuta modo_real.\033[0m'))

    # Un agente por robot. Sus tres nodos (consenso, subasta y cobertura) reciben los mismos
    # parámetros que en las prácticas 1 a 3, cada uno los suyos, en un fichero por robot.
    nbs = graphs.neighbors(graphs.adjacency(cfg['topology'], n), robots)
    offs = formations.offsets(cfg['formation'], n)
    params_dir = tempfile.mkdtemp(prefix='robcoop_reto_')
    for i, r in enumerate(robots):
        params = {
            f'/{r}/consensus_node': {'ros__parameters': {
                'neighbors': nbs[r] or [''], 'offset': [float(v) for v in offs[i]],
                'k': cfg['k_consensus'], 'use_sim_time': sim}},
            f'/{r}/auction_node': {'ros__parameters': {
                'criterion': cfg['criterion'], 'priority': i, 'use_sim_time': sim}},
            f'/{r}/coverage_node': {'ros__parameters': {
                'others': [o for o in robots if o != r] or [''], 'k': cfg['k_coverage'],
                'use_sim_time': sim, **zone}},
        }
        path = os.path.join(params_dir, f'{r}.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump(params, f)
        actions.append(Node(package='robcoop_lab', executable='agent_node', namespace=r,
                            output='screen', parameters=[path]))

    mission = Node(
        package='robcoop_lab', executable='mission_node', name='mission', output='screen',
        parameters=[{'robots': robots, 'topology': cfg['topology'],
                     'formation': cfg['formation'], 'k_consensus': cfg['k_consensus'],
                     'criterion': cfg['criterion'], 'k_coverage': cfg['k_coverage'],
                     'why': json.dumps(cfg['why'], ensure_ascii=False),
                     'task_names': list(task_xy),
                     'task_xy': [float(c) for xy in task_xy.values() for c in xy],
                     'layout': layout, 'failure': failure,
                     'fail_seed': int(arg('semilla_averia')), 'mode': mode,
                     'name': arg('nombre'), 'use_sim_time': sim, **zone}])
    actions.append(mission)
    # Cuando el árbitro termina, se cierra todo (los agentes paran sus robots al salir)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=mission, on_exit=[EmitEvent(event=Shutdown(reason='fin'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='tb3_0,tb3_1,tb3_2'),
        DeclareLaunchArgument('disposicion', default_value='A'),
        DeclareLaunchArgument('puntos', default_value=''),
        DeclareLaunchArgument('semilla', default_value='0'),
        DeclareLaunchArgument('num_puntos', default_value='6'),
        DeclareLaunchArgument('averia', default_value='auto'),
        DeclareLaunchArgument('semilla_averia', default_value='-1',
                              description='< 0: el robot averiado se elige al azar de verdad'),
        DeclareLaunchArgument('densidad', default_value='uniforme'),
        DeclareLaunchArgument('centro', default_value='0.6,-0.6'),
        DeclareLaunchArgument('sigma', default_value='0.5'),
        DeclareLaunchArgument('pico', default_value='12.0'),
        DeclareLaunchArgument('modo', default_value='sim'),
        DeclareLaunchArgument('nombre', default_value=''),
        OpaqueFunction(function=setup),
    ])
