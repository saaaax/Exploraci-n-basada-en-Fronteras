from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta al paquete y launch del robot
    robot_launch = os.path.join(
        get_package_share_directory('el7009_diff_drive_robot'),
        'launch',
        'robot.launch.py'
    )

    # SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
                

            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(
                get_package_share_directory('el7009_diff_drive_robot'),
                'config',
                'mapper_params_online_async.yaml'
            )
        }.items()
    )

    # Transformación base_footprint -> base_link
    static_transform_publisher_basefootprint_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Navegación (Nav2)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(
                get_package_share_directory('el7009_diff_drive_robot'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch)
        ),
        slam_toolbox_launch,
        static_transform_publisher_basefootprint_baselink,
        nav2,
    ])
