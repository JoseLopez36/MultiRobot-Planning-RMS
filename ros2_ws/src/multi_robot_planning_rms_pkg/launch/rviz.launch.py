import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def get_src_path(context):
    pkg_share = FindPackageShare('multi_robot_planning_rms_pkg').perform(context)
    pkg_share_path = Path(pkg_share)
    pkg_src_path = pkg_share_path.parents[3] / 'src' / 'multi_robot_planning_rms_pkg'
    return pkg_src_path

def generate_launch_description():
    # Declare the config file argument
    config_arg = launch.actions.DeclareLaunchArgument(
        'config',
        default_value='default.rviz',
        description='RViz config file name'
    )
    
    def launch_setup(context, *args, **kwargs):
        # Get package src directory
        pkg_src_path = get_src_path(context)
        
        # Get the RViz config file path from launch argument
        config = launch.substitutions.LaunchConfiguration('config').perform(context)
        rviz_config_file = pkg_src_path / 'rviz' / config
        
        return [
            # Launch RViz node
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2', 
                name='rviz2',
                arguments=['-d', str(rviz_config_file)],
                parameters=[{'use_sim_time': True}]
            )
        ]
    
    return LaunchDescription([
        config_arg,
        launch.actions.OpaqueFunction(function=launch_setup)
    ])