/* put here the code to have a client that simply asks user to insert of how much to vary the current bxtr EE pose */

#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/Pose.h"
#include "iostream"
#include "hrc_01/BaxterCmd.h"
using Pose = geometry_msgs::Pose;

hrc_01::BaxterCmd user_input(){
	double r, p, y;
	hrc_01::BaxterCmd cmd_pose;
	ROS_INFO("\n"
					"Insert the end_effector displacement:"
					"\n");
	printf("X: "); scanf("%lf", &(cmd_pose.request.position.x));
	printf("Y: "); scanf("%lf", &(cmd_pose.request.position.y));
	printf("Z: "); scanf("%lf", &(cmd_pose.request.position.z));
	printf("R: "); scanf("%lf", &(cmd_pose.request.rpy.x));
	printf("P: "); scanf("%lf", &(cmd_pose.request.rpy.y));
	printf("Y: "); scanf("%lf", &(cmd_pose.request.rpy.z));
	
	return cmd_pose;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "baxter_user_interface");
	ros::NodeHandle node_handle;
	
	ros::ServiceClient client_bxtr_ui = node_handle.serviceClient<hrc_01::BaxterCmd>("/user_interface/pose");

	ros::Rate loop(5);
	Pose cmd_pose;
	hrc_01::BaxterCmd cmd_bxtr;

	while(ros::ok()){
	
		// cmd_pose = user_input();
		
		cmd_bxtr = user_input();
		client_bxtr_ui.call(cmd_bxtr);
		
		if (!cmd_bxtr.response.success){
			ROS_INFO("Trying to reach the new pose failed!");
		}
		
		ros::spinOnce();
	}

	return 0;
}
