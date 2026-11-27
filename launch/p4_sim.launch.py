#!/usr/bin/env python3
"""Reto final — simulación: N robots en las marcas de salida y RViz con la misión.

    ros2 launch robcoop_lab p4_sim.launch.py                 # 3 robots, sin ventana de Gazebo
    ros2 launch robcoop_lab p4_sim.launch.py num_robots:=4
    ros2 launch robcoop_lab p4_sim.launch.py gui:=true       # con Gazebo (más lento en WSL)
    ros2 launch robcoop_lab p4_sim.launch.py render:=software   # si el LiDAR no mide

En RViz se ve lo que toca en cada fase: adónde va cada robot en la formación, los puntos de
inspección y las rutas, y las celdas de Voronoi con sus centroides. Arriba, la fase, los
puntos que lleva el equipo y el tiempo. Para repetir desde las marcas: Ctrl+C y volver a lanzar.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from robcoop_lab import rviz_p4
from robcoop_lab.sim_launch import sim_actions


def setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_robots').perform(context))
    return sim_actions(context, n, rviz_p4.rviz_config)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3', description='3 o 4'),
        DeclareLaunchArgument('gui', default_value='false', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('render', default_value='gpu',
                              description='gpu | software (si el LiDAR simulado no mide)'),
        OpaqueFunction(function=setup),
    ])
