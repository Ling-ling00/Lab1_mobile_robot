#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class ModelNode(Node):
    def __init__(self):
        super().__init__('model_node')

        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, "/model_odom", 10)

    def model_states_callback(self, msg:ModelStates):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "model"
        for i in range(len(msg.name)):
            if msg.name[i] == "example":
                index = i
        odom_msg.pose.pose = msg.pose[index]
        odom_msg.twist.twist = msg.twist[index]
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
