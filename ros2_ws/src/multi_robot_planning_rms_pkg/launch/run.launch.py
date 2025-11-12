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
    drone_control_path = os.path.join(package_dir, 'config', 'drone_control.yaml')
    rosbags_path = os.path.join(package_dir, 'rosbags')

    # Cargar el archivo de configuración de los nodos
    launch_config_path = os.path.join(package_dir, 'config', 'launch.yaml')
    with open(launch_config_path, 'r') as f:
        launch = yaml.safe_load(f)
    
    # Obtener las configuraciones de activación de los nodos
    # Establecer valores predeterminados si no están presentes en la configuración
    nodes = {
        'drone_control': launch.get('drone_control', [True, 'info'])
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
                parameters=[drone_control_path]
            )
        )

    if launch.get('rosbag', True):
        date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        rosbag_name = os.path.join(rosbags_path,f"experimento_{date}")
        print(rosbag_name)
        ld.add_action(
            ExecuteProcess(
                cmd=['ros2','bag','record','-o',rosbag_name, '/ground_truth/vehicle_odom'],
                output = 'screen'
            )
        )

    return ld