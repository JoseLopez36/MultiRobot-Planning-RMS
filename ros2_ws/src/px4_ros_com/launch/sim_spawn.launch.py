import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from launch.actions import ExecuteProcess


def load_config(context, *args, **kwargs):
    config_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'sim_spawn.yaml')
    config_path = os.path.abspath(config_path)
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    vehicle_type = config.get('vehicle_type', 'x500')
    pos = config.get('position', {})
    x = pos.get('x', 0.0)
    y = pos.get('y', 0.0)
    # Ignorar el eje z
    # z = pos.get('z', 0.2)
    yaw = pos.get('yaw', 0.0)

    # Comando para lanzar PX4 SITL con Gazebo y argumentos de posición
    # (Ajusta el comando según tu entorno y modelo)
    px4_cmd = f"make px4_sitl gz_{vehicle_type} gazebo_x:={x} gazebo_y:={y} gazebo_yaw:={yaw}"

    px4_dir = '/home/joseantonio/PX4-Autopilot'
    return [
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            name="microxrce_agent",
            output="screen",
        ),
        ExecuteProcess(
            cmd=["bash", "-c", px4_cmd],
            name="px4_sitl",
            output="screen",
            shell=False,
            cwd=px4_dir,
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_config)
    ])
