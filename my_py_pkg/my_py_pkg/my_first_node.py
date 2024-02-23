#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    # constructor (OO method recommended)
    def __init__(self): 
        super().__init__("py_test") # call __init__ function from the Node
        self.counter_ = 0 # initialize variable to 0
        self.get_logger().info("Hello ROS2") # directly call logger from self
        self.create_timer(0.5, self.timer_callback) # create timer with 0.5 sec btwn each callback

    def timer_callback(self):
        self.counter_ += 1 # increase counter every time the callback is called
        self.get_logger().info("Hello" + str(self.counter_)) # print Hello counter

def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communication

    # create node
    node = MyNode()
    # node = Node("py_test") # create a node
    # node.get_logger().info("Hello ROS2") # print something
    
    # spin node
    rclpy.spin(node) # pause program and keep node alive
    
    rclpy.shutdown() # shut down ROS2 communication

if __name__ == "__main__":
    main()