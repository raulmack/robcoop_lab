#!/usr/bin/env python3
"""Práctica 1 — simulación: N robots en las marcas de salida de la arena.

    ros2 launch robcoop_lab p1_sim.launch.py                 # 4 robots, sin ventana de Gazebo
    ros2 launch robcoop_lab p1_sim.launch.py num_robots:=3
    ros2 launch robcoop_lab p1_sim.launch.py gui:=true       # con Gazebo (más lento en WSL)

Para repetir un experimento desde las marcas: Ctrl+C y volver a lanzar (unos 15 s).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from robcoop_lab import arena


def setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_robots').perform(context))
    bringup = os.path.join(get_package_share_directory('turtlebot3_gz_bringup'),
                           'launch', 'multi_robot.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'num_robots': str(n), 'world': 'empty',
                          'poses': arena.poses_arg(n),
                          'gui': LaunchConfiguration('gui').perform(context),
                          'rviz': LaunchConfiguration('rviz').perform(context)}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='4', description='3 o 4'),
        DeclareLaunchArgument('gui', default_value='false', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        OpaqueFunction(function=setup),
    ])
