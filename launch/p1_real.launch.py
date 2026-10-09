#!/usr/bin/env python3
"""Práctica 1 — robots reales: ancla los robots al marco 'map' y abre RViz.

Antes: robots encendidos con robot_up.sh (lo hace el profesor), terminal en modo_real
y cada robot colocado sobre SU marca del suelo.

    ros2 launch robcoop_lab p1_real.launch.py                         # tb3_0..tb3_3
    ros2 launch robcoop_lab p1_real.launch.py robots:=tb3_0,tb3_1,tb3_2

Para repetir un experimento: volver a colocar los robots en sus marcas, Ctrl+C aquí
y lanzar de nuevo (se vuelve a anclar la odometría).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robcoop_lab import arena
from turtlebot3_gz_bringup import sim_utils


def setup(context, *args, **kwargs):
    robots = [r.strip().strip('/') for r in
              LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    marks = [float(v) for pose in arena.START_POSES[:len(robots)] for v in pose]
    actions = []
    if os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1':
        actions.append(LogInfo(msg='\033[1;31m[robcoop] Estás en modo SIMULACIÓN '
                                   '(ROS_LOCALHOST_ONLY=1): no verás los robots. '
                                   'Ejecuta modo_real y abre una terminal nueva.\033[0m'))
    actions.append(Node(package='robcoop_lab', executable='anchor_node', name='anchor_node',
                        output='screen',
                        parameters=[{'robots': robots, 'marks': marks}]))
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', sim_utils.rviz_config(robots, fixed_frame='map')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='tb3_0,tb3_1,tb3_2,tb3_3',
                              description='Robots, en el orden de las marcas'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        OpaqueFunction(function=setup),
    ])
