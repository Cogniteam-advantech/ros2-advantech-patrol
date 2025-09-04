/*
 * advantech_patrol_node.cpp
 *
 *  Converted to ROS2 Humble
 */

#include "PatrolManager.hpp"
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;



// Example usage
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PatrolManager>();

    node->run();                      // setup logic
    rclcpp::spin(node);   
    rclcpp::shutdown();     
    
    return 0;
}

