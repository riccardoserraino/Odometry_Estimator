from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')

    rviz_config = os.path.join(
        get_package_share_directory('first_project'),
        'rviz',
        'config.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='../bags/rosbag2_1',
            description='Path of rosbag to play'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock',  '--rate', '1.2'],
            output='screen',
        ),

        Node(
            package='first_project',
            executable='odometer',
            name='sim',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='first_project',
            executable='tf_error',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['bash', '-lc', "pkill -f 'first_project.*(odometer|tf_error)' || true"],
                        output='screen'
                    )
                ]
            )
        ),
    ])