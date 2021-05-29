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
	char str[16];
	
	ROS_INFO("\n"
					"Insert the end_effector goal pose:"
					"\n");
	printf("ARM: ");	scanf("%s", str);
	cmd_pose.request.arm = std::string(str);
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
	
	ros::ServiceClient client_bxtr_ui_L = node_handle.serviceClient<hrc_01::BaxterCmd>("/user_interface/pose/left");
	ros::ServiceClient client_bxtr_ui_R = node_handle.serviceClient<hrc_01::BaxterCmd>("/user_interface/pose/right");

	std::shared_ptr<ros::ServiceClient> client_bxtr_ui;

	ros::Rate loop(5);
	Pose cmd_pose;
	hrc_01::BaxterCmd cmd_bxtr;

	while(ros::ok()){
	
		// cmd_pose = user_input();
		
		cmd_bxtr = user_input();
		if (cmd_bxtr.request.arm == "left"){
			client_bxtr_ui=std::make_shared<ros::ServiceClient>(client_bxtr_ui_L);
		}
		else if (cmd_bxtr.request.arm == "right"){
			client_bxtr_ui=std::make_shared<ros::ServiceClient>(client_bxtr_ui_R);
		}
		else{
			ROS_WARN("Wrong arm value inserted. Expected 'left' or 'right', got %s", cmd_bxtr.request.arm.c_str());
		}
		
		client_bxtr_ui->call(cmd_bxtr);
		if (!cmd_bxtr.response.success){
			ROS_INFO("Trying to reach the new pose failed!");
		}
		
		ros::spinOnce();
	}

	return 0;
}
