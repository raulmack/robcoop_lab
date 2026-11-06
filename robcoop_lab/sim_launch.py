"""Piezas comunes de p1_sim y p2_sim (proporcionado).

Gazebo se lanza con turtlebot3_gz_bringup y RViz aparte, para poder elegir el render:

    render:=gpu        (por defecto) la GPU del PC, vía D3D12 en WSL
    render:=software   Gazebo dibuja por software (llvmpipe). Más lento, pero hace falta en
                       algunas GPU en las que el LiDAR simulado no mide: marca 0.12 m en todos
                       los rayos y los robots no avanzan. RViz sigue usando la GPU.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (GroupAction, IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from . import arena

RENDERS = ('gpu', 'software')


def sim_actions(context, n, rviz_config=None):
    """Acciones para lanzar n robots en las marcas (y RViz con rviz_config(robots), si se da)."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    render = arg('render').lower()
    if render not in RENDERS:
        raise ValueError(f'render debe ser uno de {RENDERS}')
    bringup = os.path.join(get_package_share_directory('turtlebot3_gz_bringup'),
                           'launch', 'multi_robot.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={'num_robots': str(n), 'world': 'empty', 'poses': arena.poses_arg(n),
                          'gui': arg('gui'), 'rviz': 'false'}.items())
    actions = []
    if render == 'software':
        actions.append(LogInfo(msg='[robcoop] Gazebo con render por software (render:=software)'))
        # GroupAction aísla la variable: solo la ven Gazebo y lo que lanza multi_robot
        actions.append(GroupAction([SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
                                    gazebo]))
    else:
        actions.append(gazebo)
    if rviz_config is not None and arg('rviz').lower() in ('true', '1'):
        robots = [f'tb3_{i}' for i in range(n)]
        actions.append(TimerAction(period=3.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', rviz_config(robots)], parameters=[{'use_sim_time': True}])]))
    return actions
