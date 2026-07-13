from launch import LaunchDescription
from launch_ros.actions import Node

#opening zed wrapper launch file from ranger2 launch file
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ranger2',
            executable='look_for_gate',
            name='gate_finder',
        ),
        Node(
            package='ranger2',
            executable='go_to_gate',
            name='gate_goerto',
        ),
        Node(
            package='ranger2',
            executable='record',
            name='video_recorder',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py'
                ])
            ),
            launch_arguments={
                'camera_model': 'zedxm',
                'custom_object_detection_config_path': 'best.yaml', 
            }.items()
        )
    

        
    ])