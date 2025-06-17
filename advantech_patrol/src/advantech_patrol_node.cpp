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

// bool readWaypoints(vector<WayPoint>& waypoints, rclcpp::Node::SharedPtr node) {
    
//     // Declare parameter
//     node->declare_parameter("waypoints", std::vector<std::string>());
    
//     vector<string> waypointsList;
    
//     if (node->get_parameter("waypoints", waypointsList) && !waypointsList.empty()) {
        
//         geometry_msgs::msg::PoseStamped pose;
//         int line = 1;
        
//         for (auto waypointString : waypointsList) {
//             double heading = 0;
            
//             auto parsedValues = sscanf(waypointString.c_str(), "%lf,%lf,%lf",
//                     &pose.pose.position.x,
//                     &pose.pose.position.y,
//                     &heading);
            
//             pose.header.frame_id = "map";
//             pose.header.stamp = rclcpp::Time(0);
            
//             // Convert yaw to quaternion using tf2
//             tf2::Quaternion quat;
//             quat.setRPY(0, 0, heading);
//             pose.pose.orientation = tf2::toMsg(quat);
            
//             WayPoint waypoint;
//             waypoint.w_pose_ = pose;
//             waypoints.push_back(waypoint);
            
//             if (parsedValues < 3) {
//                 RCLCPP_ERROR(node->get_logger(), "Failed to parse waypoint (line %i)", line);
//                 return false;
//             }
            
//             RCLCPP_INFO(node->get_logger(), "Loaded waypoint %d: (%.2f, %.2f, %.2f)", 
//                        line, pose.pose.position.x, pose.pose.position.y, heading);
            
//             line++;
//         }
        
//     } else {
//         RCLCPP_ERROR(node->get_logger(), "Error: waypoints parameter does not exist or is empty");
//         return false;
//     }
    
//     return true;
// }



// Example usage
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PatrolManager>();

    node->run2();                      // setup logic
    rclcpp::spin(node);   
    rclcpp::shutdown();     
    
    return 0;
}

// void run(){

// 		//verify amcl works and the robot have location
// 		bool recv_map_odom = false;
// 		cerr<<" waiting for robot's location ... "<<endl;

// 		while( ros::ok() && !recv_map_odom ) {	
			
// 			// cerr<<" waiting for robot's location ... "<<endl;
// 			ros::spinOnce();

// 			recv_map_odom =  checkLocalizationOk();
			
// 		}

// 		while (ros::ok()){

// 			ros::spinOnce();

// 			if ( startPatrolCmd_){
// 				break;
// 			}

// 			updateRobotLocation();


// 			publishPersonAlarm();				

// 		}
		
// 		while (ros::ok())
// 		{
// 			publishWaypointsWithStatus();

// 			for (int i = 0; i < waypoints_.size(); i++) {
				
// 				cerr<<" staring waypoint "<<i<<endl;

// 				publishPersonAlarm();				
				

// 				if (personAlarm_) {

// 					// The robot waits for the person to pass
// 					while(ros::ok()) {


// 						ros::spinOnce();

// 						publishPersonAlarm();

// 						if (personAlarm_ == false) {													

// 							break;
// 						} 
// 					}
// 				}				
				
// 				clearAllCostMaps();

// 				moveBaseController_.navigate(waypoints_[i].w_pose_);

// 				while(ros::ok()) {

// 					ros::spinOnce();

// 					updateRobotLocation();

// 					publishWaypointsWithStatus();

// 					// publishRobotHistoryPath();	

// 					publishPersonAlarm();							

// 					moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));

// 					float distDromRobot =
// 					distanceCalculate(cv::Point2d(waypoints_[i].w_pose_.pose.position.x, waypoints_[i].w_pose_.pose.position.y),
// 								cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));


// 					if (personAlarm_){

// 						moveBaseController_.moveBaseClient_.cancelGoal();

// 						// The robot waits for the person to pass
// 						while(ros::ok()) {
							

// 							ros::spinOnce();

// 							publishPersonAlarm();

// 							if (personAlarm_ == false) {							
								

// 								clearAllCostMaps();

// 								// The robot continues the navigation to the same point it canceled
// 								moveBaseController_.navigate(waypoints_[i].w_pose_);

// 								break;
// 							} 
// 						}


// 					}
// 					// if (distDromRobot < 0.15)
// 					// {
// 					// 	cerr << " cancel the goal !! " << endl;
// 					// 	moveBaseController_.moveBaseClient_.cancelGoal();
// 					// 	break;
// 					// }
					

// 					if ( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED 
// 						||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::ABORTED
// 						||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::REJECTED) {

// 						break;
// 					}
					
// 				}

// 			}

// 			cerr<<"FINSIHED PATROL!!!!! "<<endl;
// 		}



// 	}