from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable

#opening zed wrapper launch file from ranger2 launch file
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='ZED_SDK_SVO_VERSION',
            value='1'
        ),
        Node(
            package='ranger2',
            executable='look_for_gate',
            namespace='ranger2',
            name='gate_finder'
        ),
        Node(
            package='ranger2',
            executable='go_to_gate',
            namespace='ranger2',
            name='gate_goerto'
        ),
        Node(
            package='ranger2',
            executable='record',
            namespace='ranger2',
            name='video_recorder'
        ),
        Node(
            package='ranger2',
            executable='cube_kill_switch',
            namespace='ranger2',
            name='cube_kill_switch'
        ),
        Node(
            package='ranger2',
            executable='odom_to_vision_pose',
            namespace='ranger2',
            name='mavros_position_override'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
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
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch',
                    'apm.launch'
                ])
            ),
            launch_arguments={
                'fcu_url': '/dev/ttyACM0:115200'
            }.items()
        )

    ])