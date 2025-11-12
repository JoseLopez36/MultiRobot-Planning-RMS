import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
import yaml

def load_config(context, *args, **kwargs):
    # Ruta absoluta FIJA al archivo de configuración YAML fuente
    config_path = '/home/joseantonio/ws_sensor_combined/src/px4_ros_com/config/sim_spawn_multi.yaml'
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    px4_dir = '/home/joseantonio/PX4-Autopilot'
    world_path = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds', 'default.sdf')
    gz_model_path = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    gz_server_config = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'server.config')
    env_gz = dict(os.environ)
    env_gz['GZ_SIM_RESOURCE_PATH'] = env_gz.get('GZ_SIM_RESOURCE_PATH', '') + (':' if env_gz.get('GZ_SIM_RESOURCE_PATH') else '') + gz_model_path
    env_gz['GZ_SIM_SERVER_CONFIG_PATH'] = gz_server_config
    actions = [
        ExecuteProcess(
            cmd=["gz", "sim", world_path],
            name="gazebo_sim",
            output="screen",
            env=env_gz,
        ),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            name="microxrce_agent",
            output="screen",
        )
    ]

    vehicles = config.get('vehicles', [])
    # Lanzar múltiples UAVs usando el binario px4 directamente, cada uno con su instancia y pose
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    for idx, vehicle in enumerate(vehicles):
        name = vehicle.get('name', f'uav{idx+1}')
        model = vehicle.get('model', 'x500')
        pose = vehicle.get('pose', '0,0')
        autostart = str(vehicle.get('autostart', 4001))
        ns = f'px4_{idx+1}'
        pose_split = pose.split(',')
        x = pose_split[0] if len(pose_split) > 0 else '0'
        y = pose_split[1] if len(pose_split) > 1 else '0'
        yaw = pose_split[3] if len(pose_split) > 3 else '0'
        if model.startswith('gz_'):
            model_base = model
        else:
            model_base = f'gz_{model}'
        env_px4 = dict(os.environ)
        env_px4['PX4_SIM_MODEL'] = model_base
        env_px4['PX4_SIM_INSTANCE'] = str(idx)
        env_px4['PX4_GZ_MODEL_POSE'] = f'{x},{y},0,{yaw}'
        env_px4['PX4_SYS_AUTOSTART'] = autostart
        # Set unique PX4_ROS_NAMESPACE and PX4_CLIENT_KEY for each PX4 instance
        env_px4['PX4_ROS_NAMESPACE'] = ns
        env_px4['PX4_CLIENT_KEY'] = f"{100 + idx}"
        # Set unique MAVLink UDP port for QGC per instance (default 14540, 14541, ...)
        mavlink_udp_port = 14540 + idx
        # Start px4 as a server instance (-i) so multiple instances don't collide, and set MAVLink UDP port
        actions.append(
            ExecuteProcess(
                cmd=[px4_bin, '-i', str(idx)],
                name=f"px4_sitl_{name}",
                output="screen",
                cwd=os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin'),
                env=env_px4,
            )
        )
        actions.append(
            Node(
                package='px4_ros_com',
                executable='offboard_control_multi',
                name=f'offboard_control_multi_{name}',
                namespace=ns,
                output='screen',
            )
        )
    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_config)
    ])
