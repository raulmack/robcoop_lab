#!/usr/bin/env python3
"""Práctica 2 — simulación: N robots en las marcas de salida y RViz con la subasta.

    ros2 launch robcoop_lab p2_sim.launch.py                 # 3 robots, sin ventana de Gazebo
    ros2 launch robcoop_lab p2_sim.launch.py num_robots:=4
    ros2 launch robcoop_lab p2_sim.launch.py gui:=true       # con Gazebo (más lento en WSL)

En RViz se ven las tareas (gris: sin adjudicar; color del robot: adjudicada; tenue: hecha)
y la ruta de cada robot. Para repetir desde las marcas: Ctrl+C y volver a lanzar.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robcoop_lab import arena, rviz_p2


def setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_robots').perform(context))
    bringup = os.path.join(get_package_share_directory('turtlebot3_gz_bringup'),
                           'launch', 'multi_robot.launch.py')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'num_robots': str(n), 'world': 'empty',
                          'poses': arena.poses_arg(n),
                          'gui': LaunchConfiguration('gui').perform(context),
                          'rviz': 'false'}.items())]
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1'):
        robots = [f'tb3_{i}' for i in range(n)]
        actions.append(TimerAction(period=3.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', rviz_p2.rviz_config(robots)],
            parameters=[{'use_sim_time': True}])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3', description='2 a 4'),
        DeclareLaunchArgument('gui', default_value='false', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        OpaqueFunction(function=setup),
    ])
