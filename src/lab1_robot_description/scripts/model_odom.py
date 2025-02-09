#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import yaml

class ModelNode(Node):
    def __init__(self):
        super().__init__('model_node')

        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, "/model_odom", 10)

        self.odom_file = open("odom_data.yaml", "a")


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
    node = ModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
