import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler, TimerAction)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

WORLD_SERVICE = '/gazebo/worlds'


def _gazebo_ready(world_name, timeout_s=60):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            proc = subprocess.run(
                ['gz', 'service', '-s', WORLD_SERVICE,
                 '--reqtype', 'gz.msgs.Empty',
                 '--reptype', 'gz.msgs.StringMsg_V',
                 '--timeout', '2000', '--req', ''],
                capture_output=True, text=True, check=True, timeout=5)
            if world_name in proc.stdout:
                return True
        except (subprocess.TimeoutExpired, FileNotFoundError,
                subprocess.CalledProcessError):
            pass
        time.sleep(0.5)
    raise RuntimeError(
        f'Gazebo server did not advertise world [{world_name}] '
        f'within {timeout_s}s')


def _spawn_robots_when_ready(context, world_name, robots, model_dir):
    _gazebo_ready(world_name)

    spawn_actions = []
    for name, x, y, yaw in robots:
        model_path = os.path.join(model_dir, f'turtlebot3_burger_{name}.sdf')
        spawn_actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-file', model_path,
                '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw),
            ],
            output='screen',
        ))
    return spawn_actions


ROBOTS = {
    'robot_1': (-6.5, -6.5, 0.0),
    'robot_2': (6.5, -7.5, 0.0),
    'robot_3': (2.5, 10.5, 0.0),
}


def _select_robots(context):
    requested = LaunchConfiguration('robot_names').perform(context)
    names = [name.strip() for name in requested.split(',') if name.strip()]
    if not names:
        raise RuntimeError('robot_names must contain at least one robot name')
    unknown = [name for name in names if name not in ROBOTS]
    if unknown:
        raise RuntimeError(
            f'Unknown robot(s) in robot_names: {", ".join(unknown)}. '
            f'Available: {", ".join(ROBOTS)}')
    return [(name, *ROBOTS[name]) for name in names]


def _robot_actions(context, world_name, model_dir, config_dir, robot_desc,
                   use_sim_time, slam_params_path):
    robots = _select_robots(context)
    run_slam = LaunchConfiguration('slam').perform(context).lower() == 'true'

    actions = []
    for name, _, _, _ in robots:
        bridge_config = os.path.join(config_dir, f'bridge_{name}.yaml')

        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ))

        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{name}_state_publisher',
            namespace=name,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': f'{name}/',
            }],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
                ('joint_states', f'/{name}/joint_states'),
            ],
            output='screen',
        ))

        if run_slam:
            slam_node = LifecycleNode(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                namespace=name,
                parameters=[
                    slam_params_path,
                    {
                        'use_sim_time': use_sim_time,
                        'use_lifecycle_manager': False,
                        'map_name': f'/{name}/map',
                        'map_frame': f'{name}/map',
                        'odom_frame': f'{name}/odom',
                        'base_frame': f'{name}/base_footprint',
                        'scan_topic': f'/{name}/scan',
                    },
                ],
                output='screen',
            )
            actions.append(slam_node)
            actions.append(EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ),
            ))
            actions.append(RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=slam_node,
                    start_state='configuring',
                    goal_state='inactive',
                    entities=[
                        LogInfo(msg=f'{name}: activating slam_toolbox'),
                        EmitEvent(event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )),
                    ],
                ),
            ))

    actions.append(TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=lambda context:
            _spawn_robots_when_ready(context, world_name, robots, model_dir))],
    ))

    return actions


def generate_launch_description():
    pkg_sim = get_package_share_directory('search_simulation')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_bringup = get_package_share_directory('search_bringup')

    world = os.path.join(pkg_sim, 'worlds', 'search_room.sdf')
    world_name = 'search_room'
    model_dir = os.path.join(pkg_bringup, 'models')
    config_dir = os.path.join(pkg_bringup, 'config')
    slam_params_path = os.path.join(config_dir, 'slam_toolbox.yaml')

    gz_resources = os.pathsep.join([
        os.path.join(pkg_sim, 'models'),
        os.path.join(pkg_tb3, 'models'),
    ])

    urdf_path = os.path.join(pkg_tb3, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resources},
    )

    clock_and_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) time'),
        DeclareLaunchArgument(
            'robot_names', default_value='robot_1,robot_2,robot_3',
            description='Comma-separated list of robots to spawn '
                        '(e.g. robot_1 or robot_1,robot_3)'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Run slam_toolbox for each spawned robot'),
        gz_sim,
        clock_and_tf_bridge,
        OpaqueFunction(
            function=_robot_actions,
            args=[world_name, model_dir, config_dir, robot_desc,
                  use_sim_time, slam_params_path],
        ),
    ])