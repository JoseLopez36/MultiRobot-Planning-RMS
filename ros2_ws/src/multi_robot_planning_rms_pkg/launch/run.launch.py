from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime
import yaml

def generate_launch_description():
    # Obtener las rutas a los archivos de configuración
    package_dir = get_package_share_directory('multi_robot_planning_rms_pkg')
    mission_config_path = os.path.join(package_dir, 'config', 'mission.yaml')
    nodes_config_path = os.path.join(package_dir, 'config', 'nodes.yaml')
    launch_config_path = os.path.join(package_dir, 'config', 'launch.yaml')

    # Cargar el archivo de configuración del lanzamiento de nodos
    with open(launch_config_path, 'r') as f:
        launch = yaml.safe_load(f)
    
    # Obtener las configuraciones de activación de los nodos
    nodes = {
        'coordinator': launch.get('coordinator', [True, 'info']),
        'darp': launch.get('darp', [True, 'info']),
        'px4_transform': launch.get('px4_transform', [True, 'info'])
    }

    # Inicializar la descripción de la lanzamiento
    ld = LaunchDescription()

    # Condicionalmente agregar el nodo coordinador
    if nodes['coordinator'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='coordinator_node',
                name='coordinator_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['coordinator'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )


    # Condicionalmente agregar el nodo DARP
    if nodes['darp'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='darp_node',
                name='darp_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['darp'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )

    # Condicionalmente agregar el nodo de transformacion de PX4
    if nodes['px4_transform'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='px4_transform_node',
                name='px4_transform_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['px4_transform'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )


    return ld