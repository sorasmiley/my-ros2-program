#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class SmartphoneNode(Node):

    # constructor (OO method recommended)
    def __init__(self): 
        # initialize node
        super().__init__("smartphone") # node name

        # create subscriber inside node
        # topic type, topic name, callback function, keep 10 messages
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone has been started.")

    # create callback to handle message you got
    # callback_topic
    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communication
    node = SmartphoneNode() # create node
    rclpy.spin(node) # pause program and keep node alive
    rclpy.shutdown() # shut down ROS2 communication

if __name__ == "__main__":
    main()