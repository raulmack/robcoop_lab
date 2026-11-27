#!/usr/bin/env python3
"""Práctica 3 — simulación: N robots en su posición de salida y RViz con la cobertura.

    ros2 launch robcoop_lab p3_sim.launch.py                    # 3 robots en las marcas
    ros2 launch robcoop_lab p3_sim.launch.py salida:=agrupados  # juntos en una esquina
    ros2 launch robcoop_lab p3_sim.launch.py num_robots:=4
    ros2 launch robcoop_lab p3_sim.launch.py gui:=true          # con Gazebo (más lento en WSL)
    ros2 launch robcoop_lab p3_sim.launch.py render:=software   # si el LiDAR no mide

En RViz se ven las celdas de Voronoi (cada una con el color de su robot), el centroide de
cada celda (esfera) y el borde de la zona a cubrir. Para repetir desde la salida: Ctrl+C y
volver a lanzar.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from robcoop_lab import coverage_core, rviz_p3
from robcoop_lab.sim_launch import sim_actions


def setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_robots').perform(context))
    start = LaunchConfiguration('salida').perform(context)
    return sim_actions(context, n, rviz_p3.rviz_config, coverage_core.poses_arg(start, n))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3', description='2 a 4'),
        DeclareLaunchArgument('salida', default_value='marcas',
                              description='marcas | agrupados'),
        DeclareLaunchArgument('gui', default_value='false', description='Ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('render', default_value='gpu',
                              description='gpu | software (si el LiDAR simulado no mide)'),
        OpaqueFunction(function=setup),
    ])
