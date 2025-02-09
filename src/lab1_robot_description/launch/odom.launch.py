from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "lab1_robot_description"
    rviz_file_name = "odom.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )
    model_odom = Node(
            package=package_name,
            executable='model_odom.py',
    )
    single_track = Node(
            package=package_name,
            executable='single_track.py',
    )
    double_track = Node(
            package=package_name,
            executable='double_track.py',
    )
    yaw_rate = Node(
            package=package_name,
            executable='yaw_rate.py',
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )
    return LaunchDescription([
        model_odom,
        single_track,
        double_track,
        yaw_rate,
        rviz
    ])