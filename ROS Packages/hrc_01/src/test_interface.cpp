/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "human_baxter_collaboration/BaxterTrajectory.h"
#include "hrc_01/BaxterCmd.h"

ros::Publisher traj_pub;
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// MoveIt operates on sets of joints called "planning groups" and stores them in an object called
// the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
// are used interchangably.
static const std::string ARM = "left";
static const std::string PLANNING_GROUP = ARM+"_arm";
moveit::planning_interface::MoveGroupInterface *move_group_interface;
static const std::string PLANNING_GROUP_2 = "right_hand";
moveit::planning_interface::MoveGroupInterface *mgi_2;
static bool flag_relative_goal = false;


void RPY2Quat(geometry_msgs::Quaternion* orig_orientation, double r_, double p_, double y_){
	tf2::Quaternion q_orig, q_rot, q_new;

	// Get the original orientation of 'commanded_pose'
	tf2::convert(*orig_orientation, q_orig);

	double r=r_*M_PI/180, p=p_*M_PI/180, y=y_*M_PI/180;
	q_rot.setRPY(r, p, y);

	q_new = q_rot*q_orig;  // Calculate the new orientation
	q_new.normalize();

	// Stuff the new rotation back into the pose. This requires conversion into a msg type
	tf2::convert(q_new, *orig_orientation);
}

bool modifyPose(hrc_01::BaxterCmd::Request &req, hrc_01::BaxterCmd::Response &res){

	// retrieve current end effector pose
	geometry_msgs::PoseStamped current_pose;
	current_pose = move_group_interface->getCurrentPose();
	
	ROS_INFO(	"\nCurrent Position\n"
						"X: %lf\n"
						"Y: %lf\n"
						"Z: %lf\n",
						current_pose.pose.position.x, current_pose.pose.position.y,
						current_pose.pose.position.z
						);
	std::vector<double> current_orientation = move_group_interface->getCurrentRPY();
	ROS_INFO(	"\nCurrent Orientation\n"
						"R: %lf\n"
						"P: %lf\n"
						"Y: %lf\n",
						current_orientation[0]*360/tau, current_orientation[1]*360/tau,
						current_orientation[2]*360/tau
						);

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  
  if (flag_relative_goal) target_pose1  = current_pose.pose; 
  else	target_pose1.orientation.w = 1.0;
  // if flag is false target post would be 0, so the movement would become absolute
  
  target_pose1.position.x += req.position.x;
  target_pose1.position.y += req.position.y;
  target_pose1.position.z += req.position.z;
  RPY2Quat(&(target_pose1.orientation), req.rpy.x, req.rpy.y, req.rpy.z);
  
  ROS_INFO(	"\nGoal Position\n"
						"X: %lf\n"
						"Y: %lf\n"
						"Z: %lf\n",
						target_pose1.position.x, target_pose1.position.y,
						target_pose1.position.z
						);
	
  ROS_INFO(	"\nGoal Orientation\n"
						"X: %lf\n"
						"Y: %lf\n"
						"Z: %lf\n"
						"W: %lf\n",
						target_pose1.orientation.x, target_pose1.orientation.y,
						target_pose1.orientation.z, target_pose1.orientation.w
						);
  
  move_group_interface->setPoseTarget(target_pose1);
  move_group_interface->setGoalTolerance(0.01);
  // move_group_interface->setGoalOrientationTolerance(tau);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  
  human_baxter_collaboration::BaxterTrajectory bxtr_traj;
  bxtr_traj.arm = ARM;
  bxtr_traj.trajectory.push_back(my_plan.trajectory_);
  traj_pub.publish(bxtr_traj);	// publish the plan trajectory
  
  res.success = success;
  
  return success;
}
/*
bool goalPose(hrc_01::BaxterCmd::Request &req, hrc_01::BaxterCmd::Response &res){

	// retrieve current end effector pose
	geometry_msgs::PoseStamped current_pose;
	current_pose = move_group_interface->getCurrentPose();
	
	ROS_INFO(	"\nCurrent Position\n"
						"X: %lf\n"
						"Y: %lf\n"
						"Z: %lf\n",
						current_pose.pose.position.x, current_pose.pose.position.y,
						current_pose.pose.position.z
						);
						
	ROS_INFO(	"\nCurrent Orientation\n"
						"X: %lf\n"
						"Y: %lf\n"
						"Z: %lf\n"
						"W: %lf\n",
						current_pose.pose.orientation.x, current_pose.pose.orientation.y,
						current_pose.pose.orientation.z, current_pose.pose.orientation.w
						);

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1 = current_pose.pose;
  target_pose1.position = req.position;
  //target_pose1.orientation.x = 0; target_pose1.orientation.y = 0; target_pose1.orientation.z = 0; target_pose1.orientation.w = 1;
  RPY2Quat(&(target_pose1.orientation), req.rpy.x, req.rpy.y, req.rpy.z);
  
  move_group_interface->setPoseTarget(target_pose1);
  move_group_interface->setGoalTolerance(0.01);
  //move_group_interface->setGoalOrientationTolerance(tau);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  
  human_baxter_collaboration::BaxterTrajectory bxtr_traj;
  bxtr_traj.arm = "right";
  bxtr_traj.trajectory.push_back(my_plan.trajectory_);
  traj_pub.publish(bxtr_traj);	// publish the plan trajectory
  
  res.success = success;
  
  return success;
}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_test_interface");
  ros::NodeHandle node_handle;
  

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
		
	// The :planning_interface:`MoveGroupInterface` class can be easily
	// setup using just the name of the planning group you would like to control and plan for
	move_group_interface = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	move_group_interface->setPlannerId("RRTConnectkConfigMechanical");
	move_group_interface->setPlanningTime(2);
	move_group_interface->setNumPlanningAttempts(5);
	
	// mgi_2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_2);
	
	/*
	// Add table to the environment
	// ^^^^^^^^^
	// Now let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "table";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.6;
  primitive.dimensions[primitive.BOX_Y] = 2.0;
  primitive.dimensions[primitive.BOX_Z] = 0.8;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.7673182;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO("Table added into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
	*/
	
  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  /*const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface->getCurrentState()->getJointModelGroup(PLANNING_GROUP);*/
	// publisher which pubs the trajectories w/ the info about the arm already in
  traj_pub = node_handle.advertise<human_baxter_collaboration::BaxterTrajectory>("/baxter_moveit_trajectory", 1000);
  // service that gets the user input and updates the current EE pose with that info	
	/* ros::ServiceServer ui_service = node_handle.advertiseService("/user_interface/pose", modifyPose); */
	ros::ServiceServer ui_service = node_handle.advertiseService("/user_interface/pose", modifyPose);
	/*
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface->getJointModelGroupNames().begin(),
            move_group_interface->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  printf("\n");
  // We can get a list of all the groups in the robot:
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "%s End effector link: %s", PLANNING_GROUP_2.c_str(), mgi_2->getEndEffector().c_str());
  ROS_INFO_NAMED("tutorial", "Available Links %s:", PLANNING_GROUP_2.c_str());
  std::copy(mgi_2->getLinkNames().begin(),
            mgi_2->getLinkNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  printf("\n");
  */
  // END_TUTORIAL

  ros::waitForShutdown();
  return 0;
}

