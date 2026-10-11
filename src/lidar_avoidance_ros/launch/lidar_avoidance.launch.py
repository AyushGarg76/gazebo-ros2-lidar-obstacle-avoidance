import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Launch Gazebo Fortress with the obstacle world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r /home/ayush/robotics_ws/src/lidar_world.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Bridge LiDAR: Gazebo -> ROS 2
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # Bridge velocity commands: ROS 2 -> Gazebo
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        output='screen'
    )

    # ROS 2 obstacle-avoidance controller
    avoidance_controller = Node(
        package='lidar_avoidance_ros',
        executable='lidar_avoidance_node',
        name='lidar_avoidance_node',
        output='screen'
    )

    # IMPORTANT: Include gazebo_launch in this list
    return LaunchDescription([
        gazebo_launch,
        lidar_bridge,
        cmd_vel_bridge,
        avoidance_controller
    ])