/* put here the code to have a client that simply asks user to insert of how much to vary the current bxtr EE pose */

#include "ros/ros.h"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "human_baxter_collaboration/BaxterStopTrajectory.h"
//#include "geometry_msgs/Pose.h"
#include "iostream"


ros::Publisher stop_pub;

bool user_input(){
	std::string cmd;
	ROS_INFO("\n"
					"Stop the current trajectory [y/n]:"
					"\n");
	std::cin >> cmd;
	
	if (cmd == "y" || cmd == "yes" || cmd == "Y" || cmd == "Yes" || cmd == "YES"){
		return true;
	}
	
	return false;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "baxter_stop_interface");
	ros::NodeHandle node_handle;

  stop_pub = node_handle.advertise<human_baxter_collaboration::BaxterStopTrajectory>("/baxter_moveit_stop_trajectory", 1000);
	ros::Rate loop(5);
	human_baxter_collaboration::BaxterStopTrajectory empty_msg;

	while(ros::ok()){
	
		if (user_input()){
			ROS_INFO("Stopping the current execution");
			stop_pub.publish(empty_msg);
		}
		
		ros::spinOnce();
	}

	return 0;
}
