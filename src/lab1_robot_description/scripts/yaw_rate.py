#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_from_euler
import yaml

class YawRateNode(Node):
    def __init__(self):
        super().__init__('yaw_rate_node')
        
        self.dt = 0.01

        self.create_timer(self.dt, self.timer_callback)
        
        self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.joint_states_callback, 10)
        self.create_subscription(LinkStates, '/gazebo/link_states', self.link_states_callback, 10)

        self.odom_publisher1 = self.create_publisher(Odometry, "/yaw_rate_odom", 10)

        self.odom = [0,0,0,0,0,0] #x, y, theta, beta, v, w
        self.vel_rear = [0,0] #right, left
        self.direction = [0,0]
        self.w = 0.0

        self.odom_file = open("yaw_rate.yaml", "a")

    def timer_callback(self):
        new_odom = [0,0,0,0,0,0]
        new_odom[0] = self.odom[0] + (self.odom[4] * self.dt * np.cos(self.odom[3] + self.odom[2] + (self.odom[5]*self.dt/2)))
        new_odom[1] = self.odom[1] + (self.odom[4] * self.dt * np.sin(self.odom[3] + self.odom[2] + (self.odom[5]*self.dt/2)))
        new_odom[2] = self.odom[2] + (self.odom[5]*self.dt)
        new_odom[4] = (self.vel_rear[0] + self.vel_rear[1])/2
        new_odom[5] = self.w
        self.odom = new_odom
        self.odom_pub(self.odom)

    def joint_states_callback(self, msg:DynamicJointState):
        index_l, index_r = None, None
        for i in range(len(msg.joint_names)):
            if msg.joint_names[i] == "left_joint_b":
                index_l = i
            elif msg.joint_names[i] == "right_joint_b":
                index_r = i

        if index_l is not None and index_r is not None:
            if msg.interface_values[index_l].values[1] > 0:
                self.direction[1] = 1
            else:
                self.direction[1] = -1

            if msg.interface_values[index_r].values[1] > 0:
                self.direction[0] = 1
            else:
                self.direction[0] = -1

    def link_states_callback(self, msg:LinkStates):
        index_l, index_r, index_c = None, None, None
        for i in range(len(msg.name)):
            if msg.name[i] == "example::left_wheel_b":
                index_l = i
            elif msg.name[i] == "example::right_wheel_b":
                index_r = i
            elif msg.name[i] == "example::base_link":
                index_c = i

        if index_l is not None and index_r is not None:
            self.vel_rear = [np.sqrt(msg.twist[index_r].linear.x**2 + msg.twist[index_r].linear.y**2)*self.direction[0],
                              np.sqrt(msg.twist[index_l].linear.x**2 + msg.twist[index_l].linear.y**2)*self.direction[1]]
        if index_c is not None:
            self.w = msg.twist[index_c].angular.z

    def odom_pub(self, odom):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "double"
        odom_msg.pose.pose.position.x = odom[0]
        odom_msg.pose.pose.position.y = odom[1]

        q = quaternion_from_euler(0 ,0 , odom[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = odom[4]
        odom_msg.twist.twist.angular.z = odom[5]
        self.odom_publisher1.publish(odom_msg)

        # Convert the timestamp to seconds (including nanoseconds).
        stamp = odom_msg.header.stamp
        timestamp_sec = stamp.sec + stamp.nanosec * 1e-9

        # Extract pose and twist information.
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        lin = odom_msg.twist.twist.linear
        ang = odom_msg.twist.twist.angular

         # Create a dictionary to represent the odometry message.
        data = {
            'timestamp': timestamp_sec,
            'position': {
                'x': pos.x,
                'y': pos.y,
                'z': pos.z
            },
            'orientation': {
                'x': ori.x,
                'y': ori.y,
                'z': ori.z,
                'w': ori.w
            },
            'linear': {
                'x': lin.x,
                'y': lin.y,
                'z': lin.z
            },
            'angular': {
                'x': ang.x,
                'y': ang.y,
                'z': ang.z
            }
        }

        # Append the new data as a YAML document.
        self.odom_file.write("---\n")
        yaml.dump(data, self.odom_file)
        self.odom_file.flush()

        


def main(args=None):
    rclpy.init(args=args)
    node = YawRateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
