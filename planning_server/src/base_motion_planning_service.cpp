/*
 * base_motion_planning_service.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: kuhnerd
 */

#include<ros/ros.h>
#include<planner_msgs/base_planning.h>
#include<eigen_conversions/eigen_msg.h>
#include<birrt_star_algorithm/birrt_star.h>
#include<motion_trajectory_execution/trajectory_execution_robot.h>

birrt_star_motion_planning::BiRRTstarPlanner *birrt_planner;
trajectory_execution::MotionCommanderRobot *trajectory_controller;

bool plan_base_motion(planner_msgs::base_planning::Request  &req,
		planner_msgs::base_planning::Response &res)
{
  Eigen::Affine3d target;
  tf::poseMsgToEigen (req.target_pose, target);

  bool success = birrt_planner->plan(target);

  if(success){
	  trajectory_controller->execute();
      res.success = true;
  }
  else{
      res.success = false;
  }

  return success;
}

int main(int argc, char** argv){

	ros::init(argc,argv,"birrt_planning_service");
	ros::NodeHandle nh;

	birrt_planner = new birrt_star_motion_planning::BiRRTstarPlanner("omnirob_base");
	trajectory_controller = new trajectory_execution::MotionCommanderRobot("omnirob_base");

	ros::ServiceServer planning_service = nh.advertiseService("plan_base_motion",plan_base_motion);

	ros::spin();

	delete birrt_planner;
	delete trajectory_controller;
}

