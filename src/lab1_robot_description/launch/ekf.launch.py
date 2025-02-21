from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "lab1_robot_description"

    ekf_double = Node(package=package_name,
                      namespace='/double_track',
                      executable='ekf_node.py',
                      name='double_track_ekf'
                      )
    
    ekf_single = Node(package=package_name,
                      namespace='/single_track',
                      executable='ekf_node.py',
                      name='single_track_ekf'
                      )

    ekf_yawrate = Node(package=package_name,
                      namespace='/yaw_rate',
                      executable='ekf_node.py',
                      name='yaw_rate_ekf'
                      )
    return LaunchDescription([
        ekf_double,
        ekf_single,
        ekf_yawrate
    ])