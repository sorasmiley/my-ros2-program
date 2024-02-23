#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    // initialize Node class
    // initialize counter when we create Node
    MyNode() : Node("cpp_test"), counter_(0) // create the node called "cpp_test"
    {
        // this b/c get__logger() is part of class
        // print something with logger from the node
        RCLCPP_INFO(this->get_logger(), "Hello Cpp  Node");

        // initialize timer
        // wall timer will call timerCallback function every 1 second
        // 1st argument is duration or every second
        // 2nd argument timer callback function which will be bound to this object
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this)); 
                                        
    }

private:
    void timerCallback()
    {
        counter_++; // increase counter by one
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    // declare a (sharedpointer to) timer as private  member of class
    rclcpp::TimerBase::SharedPtr timer_;
    // declare a counter
    int counter_;
};

// main function will be the same for all node (5 lines)
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize ROS2 communication

    //  create node
    // make_shared creates a shared pointer to a Node
    // do not need to use new or delete manually
    // when shared pointer goes out of scope, it auto destroys node
    // auto node =  std::make_shared<rclcpp::Node>("cpp_test");
    // RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");

    // create shared pointer to node and constructor gets called
    auto node = std::make_shared<MyNode>();

    // spin is executed so pause the program
    rclcpp::spin(node);
    rclcpp::shutdown(); // shutdown ROS2 communication

    return 0;
}