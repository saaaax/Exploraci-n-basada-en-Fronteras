from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path al launch del robot
    robot_launch = os.path.join(
        get_package_share_directory('el7009_diff_drive_robot'),
        'launch',
        'robot.launch.py'
    )

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # Lanza SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    static_transform_publisher_basefootprint_baselink = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_transform_basefootprint',
    arguments=[
        '0', '0', '0',
        '0', '0', '0',
        'map', 'odom'
    ]
)


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch)
        ),
        slam_toolbox_launch,
        static_transform_publisher_basefootprint_baselink,
    ])