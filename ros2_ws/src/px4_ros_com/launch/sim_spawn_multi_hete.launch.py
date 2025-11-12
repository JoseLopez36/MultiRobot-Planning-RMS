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
    actions = []

    vehicles = config.get('vehicles', [])
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')

    # Lanzar Gazebo solo una vez
    if vehicles:
        actions.append(
            ExecuteProcess(
                cmd=["gz", "sim", world_path,"--start-paused", "false"],
                name="gazebo_sim",
                output="screen",
                env=env_gz,
            )
        )
    # Lanzar MicroXRCEAgent una vez
    actions.append(
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            name="microxrce_agent",
            output="screen",
        )
    )

    for idx, vehicle in enumerate(vehicles):
        name = vehicle.get('name', f'uav{idx+1}')
        model = vehicle.get('model', 'x500')
        pose = vehicle.get('pose', '0,0')
        autostart = str(vehicle.get('autostart', 4001))
        instance = vehicle.get('instance', idx)  # Usa el valor de 'instance' del YAML
        ns = f'px4_{instance}' if instance > 0 else ''  # Sin namespace para instance 0
        sysid = instance
        pose_split = pose.split(',')
        x = pose_split[0] if len(pose_split) > 0 else '0'
        y = pose_split[1] if len(pose_split) > 1 else '0'
        yaw = pose_split[3] if len(pose_split) > 3 else '0'
        if model.startswith('gz_'):
            model_base = model
        else:
            model_base = f'gz_{model}'
        print(f"\n--- Vehículo {idx} ---")
        print(f"name: {name}")
        print(f"model: {model}")
        print(f"model_base: {model_base}")
        print(f"pose: {pose}")
        print(f"autostart: {autostart}")
        print(f"instance (PX4): {instance}")
        print(f"namespace: {ns}")
        print(f"SYSID_THISMAV: {sysid}")
        print(f"x: {x}, y: {y}, yaw: {yaw}")
        env_px4 = dict(os.environ)
        env_px4['PX4_SIM_MODEL'] = model_base
        env_px4['PX4_SIM_INSTANCE'] = str(instance)
        env_px4['PX4_SYS_AUTOSTART'] = autostart
        if ns:
            env_px4['PX4_ROS_NAMESPACE'] = ns
        env_px4['SYSID_THISMAV'] = str(sysid)
        print(f"PX4_SIM_MODEL: {env_px4['PX4_SIM_MODEL']}")
        print(f"PX4_SIM_INSTANCE: {env_px4['PX4_SIM_INSTANCE']}")
        print(f"PX4_SYS_AUTOSTART: {env_px4['PX4_SYS_AUTOSTART']}")
        if ns:
            print(f"PX4_ROS_NAMESPACE: {env_px4['PX4_ROS_NAMESPACE']}")
        print(f"SYSID_THISMAV: {env_px4['SYSID_THISMAV']}")
        if int(instance) > 1:
            env_px4['PX4_GZ_STANDALONE'] = '1'
        env_px4['PX4_GZ_MODEL_POSE'] = f'{x},{y},0,{yaw}'
        #print(f"PX4_GZ_STANDALONE: {env_px4['PX4_GZ_STANDALONE']}")
        print(f"PX4_GZ_MODEL_POSE: {env_px4['PX4_GZ_MODEL_POSE']}")
        print(f"cmd: {[px4_bin, '-i', str(instance)]}")
        print(f"cwd: {os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin')}")
        actions.append(
            ExecuteProcess(
                cmd=[px4_bin, '-i', str(instance)],
                name=f"px4_sitl_{name}",
                output="screen",
                cwd=os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin'),
                env=env_px4,
            )
        )
        # Asignar puertos UDP/TCP únicos por instancia
        env_px4['PX4_SITL_UDP_PORT'] = str(14560 + instance)
        env_px4['PX4_SITL_TCP_PORT'] = str(4560 + instance)
        print(f"PX4_SITL_UDP_PORT: {env_px4['PX4_SITL_UDP_PORT']}")
        print(f"PX4_SITL_TCP_PORT: {env_px4['PX4_SITL_TCP_PORT']}")
    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_config)
    ])
