#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    # constructor (OO method recommended)
    def __init__(self): 
        # initialize node
        super().__init__("robot_news_station") # node

        self.robot_name_ = "C3PO"

        # create the publisher inside the node
        # publisher_ object is from create_publisher method
        # publisher with data type and name, keep 10 messages
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # topic

        # create a timer from self(Node object) with period of 2 hertz and callback
        self.timer = self.create_timer(0.5, self.publish_news)

        # print something for debugging
        self.get_logger().info("Robot News Station has been started.")

    # to publish "Hello" on the topic robot_news_station
    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name_) + " from the robot news station."
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communication
    node = RobotNewsStationNode()
    rclpy.spin(node) # pause program and keep node alive
    rclpy.shutdown() # shut down ROS2 communication

if __name__ == "__main__":
    main()