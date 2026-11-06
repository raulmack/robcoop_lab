#!/usr/bin/env python3
"""Práctica 1 — simulación: N robots en las marcas de salida de la arena.

    ros2 launch robcoop_lab p1_sim.launch.py                 # 4 robots, sin ventana de Gazebo
    ros2 launch robcoop_lab p1_sim.launch.py num_robots:=3
    ros2 launch robcoop_lab p1_sim.launch.py gui:=true       # con Gazebo (más lento en WSL)
    ros2 launch robcoop_lab p1_sim.launch.py render:=software   # si el LiDAR no mide

Para repetir un experimento desde las marcas: Ctrl+C y volver a lanzar (unos 15 s).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from turtlebot3_gz_bringup import sim_utils

from robcoop_lab.sim_launch import sim_actions


def setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_robots').perform(context))
    return sim_actions(context, n, lambda robots: sim_utils.rviz_config(robots))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='4', description='3 o 4'),
        DeclareLaunchArgument('gui', default_value='false', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('render', default_value='gpu',
                              description='gpu | software (si el LiDAR simulado no mide)'),
        OpaqueFunction(function=setup),
    ])
