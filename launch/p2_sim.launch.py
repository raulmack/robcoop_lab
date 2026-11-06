#!/usr/bin/env python3
"""Práctica 2 — simulación: N robots en las marcas de salida y RViz con la subasta.

    ros2 launch robcoop_lab p2_sim.launch.py                 # 3 robots, sin ventana de Gazebo
    ros2 launch robcoop_lab p2_sim.launch.py num_robots:=4
    ros2 launch robcoop_lab p2_sim.launch.py gui:=true       # con Gazebo (más lento en WSL)
    ros2 launch robcoop_lab p2_sim.launch.py render:=software   # si el LiDAR no mide

En RViz se ven las tareas (gris: sin adjudicar; color del robot: adjudicada; tenue: hecha)
y la ruta de cada robot. Para repetir desde las marcas: Ctrl+C y volver a lanzar.
Si los robots no avanzan y sale «el LiDAR marca 0.12 m en todos los rayos», relanzad con
render:=software (pasa con algunas GPU).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from robcoop_lab import rviz_p2
from robcoop_lab.sim_launch import sim_actions


def setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_robots').perform(context))
    return sim_actions(context, n, rviz_p2.rviz_config)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3', description='2 a 4'),
        DeclareLaunchArgument('gui', default_value='false', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('render', default_value='gpu',
                              description='gpu | software (si el LiDAR simulado no mide)'),
        OpaqueFunction(function=setup),
    ])
