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
    system_path = os.path.join(package_dir, 'config', 'system.yaml')
    drone_control_path = os.path.join(package_dir, 'config', 'drone_control.yaml')
    tf_manager_path = os.path.join(package_dir, 'config', 'tf_manager.yaml')
    visualization_path = os.path.join(package_dir, 'config', 'visualization.yaml')
    EIF_filter_path = os.path.join(package_dir, 'config', 'EIF_filter.yaml')

    rosbags_path = os.path.join(package_dir, 'rosbags')

    # Cargar el archivo de configuración de los nodos
    launch_config_path = os.path.join(package_dir, 'config', 'launch.yaml')
    with open(launch_config_path, 'r') as f:
        launch = yaml.safe_load(f)
    
    # Obtener las configuraciones de activación de los nodos
    # Establecer valores predeterminados si no están presentes en la configuración
    nodes = {
        'drone_control': launch.get('drone_control', [True, 'info']),
        'tf_manager': launch.get('tf_manager', [True, 'info']),
        'visualization': launch.get('visualization', [True, 'info']),
        'EIF_filter': launch.get('EIF_filter', [True, 'info']),
        'EIF_filter_descentralized': launch.get('EIF_filter_descentralized', [True, 'info'])
    }

    # Inicializar la descripción de la lanzamiento
    ld = LaunchDescription()

    # Condicionalmente agregar el nodo Drone Control
    if nodes['drone_control'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='drone_control_node',
                name='drone_control_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['drone_control'][1]],
                parameters=[system_path, drone_control_path]
            )
        )

    # Condicionalmente agregar el nodo TF Manager
    if nodes['tf_manager'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='tf_manager_node',
                name='tf_manager_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['tf_manager'][1]],
                parameters=[system_path, tf_manager_path]
            )
        )

    # Condicionalmente agregar el nodo Visualization
    if nodes['visualization'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='visualization_node',
                name='visualization_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['visualization'][1]],
                parameters=[system_path, visualization_path]
            )
        )

    # Añadir puentes a Gazebo
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            output='screen',
            arguments=[
                '/model/x500_mod_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # Añadir más puentes aquí
            ],
            remappings=[
                ('/model/x500_mod_0/odometry', '/ground_truth/vehicle_odom')
            ]
        )
    )

    # Condicionalmente agregar el nodo EIF Filter
    if nodes['EIF_filter'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='EIF_filter_node',
                #name='EIF_filter_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['EIF_filter'][1]],
                parameters=[system_path, EIF_filter_path, drone_control_path],
            )
        )

    if nodes['EIF_filter_descentralized'][0]:
        ld.add_action(
            Node(
                package='multi_robot_planning_rms_pkg',
                executable='EIF_filter_descentralized_node',
                #name='EIF_filter_node',
                output='screen',
                arguments=['--ros-args', '--log-level', nodes['EIF_filter_descentralized'][1]],
                parameters=[system_path, EIF_filter_path, drone_control_path],
            )
        )

    if launch.get('rosbag', True):
        date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        rosbag_name = os.path.join(rosbags_path,f"experimento_{date}")
        print(rosbag_name)
        ld.add_action(
            ExecuteProcess(
                cmd=['ros2','bag','record','-o',rosbag_name, '/ground_truth/vehicle_odom', '/EIF_filter_node/predicted_position', '/EIF_filter_node/process_stats','/EIF_filter_descentralized_node/predicted_position', '/EIF_filter_descentralized_node/process_stats'],
                output = 'screen'
            )
        )

    return ld