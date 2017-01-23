/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: planning_service_node.cpp
 */

#include <ros/ros.h>
#include <planning_server/planning_server.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "planning_service_node");
	ros::NodeHandle n;

	planning_server::PlanningServer planning_server;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}
