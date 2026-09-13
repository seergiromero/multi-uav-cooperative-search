import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('search_simulation')

    world = PathJoinSubstitution([
        pkg_share, 'worlds', 'search_room.sdf'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, 'models'),
        },
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) time'),
        gz_sim,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_sim,
                on_exit=[bridge],
            )
        ),
    ])