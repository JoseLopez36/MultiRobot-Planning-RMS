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

    # Cargar configuración de misión (para obtener agents.ids)
    with open(mission_config_path, 'r') as f:
        mission = yaml.safe_load(f)

    # Cargar configuración de parámetros por nodo (para defaults de control_node)
    with open(nodes_config_path, 'r') as f:
        nodes_cfg = yaml.safe_load(f)
    
    # Obtener las configuraciones de activación de los nodos
    nodes = {
        'planning': launch.get('planning', [True, 'info']),
        'darp': launch.get('darp', [True, 'info']),
        'control': launch.get('control', [True, 'info']),
        'px4_transform': launch.get('px4_transform', [True, 'info']),
        'visualization': launch.get('visualization', [True, 'info'])
    }

    # Obtener IDs de agentes desde mission.yaml
    agents_ids = []
    try:
        agents_ids = mission.get('/**', {}).get('ros__parameters', {}).get('agents', {}).get('ids', [])
    except Exception:
        agents_ids = []

    # Defaults de control_node desde nodes.yaml (opcional)
    control_params = nodes_cfg.get('control_node', {}).get('ros__parameters', {}) if isinstance(nodes_cfg, dict) else {}

    # Inicializar la descripción de la lanzamiento
    ld = LaunchDescription()

    # Condicionalmente agregar el nodo de planificación
    if nodes['planning'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='planning_node',
                name='planning_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['planning'][1]],
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

    # Condicionalmente agregar N nodos de control (uno por agente)
    if nodes['control'][0]:
        for agent_id in agents_ids:
            ld.add_action(
                Node(
                    package='multi_robot_planning_rms_pkg',
                    executable='control_node',
                    namespace=agent_id,
                    name='control_node',
                    output='screen',
                    arguments=['--ros-args', '--log-level', nodes['control'][1]],
                    parameters=[mission_config_path, {'agent_id': agent_id}, control_params]
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

    # Condicionalmente agregar el nodo de visualización
    if nodes['visualization'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='visualization_node',
                name='visualization_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['visualization'][1]],
                parameters=[mission_config_path, nodes_config_path]
            )
        )


    return ld