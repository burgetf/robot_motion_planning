// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.3 on November 24th 2015

#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// API services:
#include "vrep_skeleton_msg_and_srv/displayText.h"

class ROS_server
{
	public:
		static bool initialize();
		static void shutDown();

		static void instancePass();
		static void mainScriptAboutToBeCalled();

		static void simulationAboutToStart();
		static void simulationEnded();

	private:
		ROS_server() {}; 
		
		static ros::NodeHandle* node;

		static void spinOnce();

		// Services:
		static bool displayText_service(vrep_skeleton_msg_and_srv::displayText::Request &req,vrep_skeleton_msg_and_srv::displayText::Response &res);
		static ros::ServiceServer displayText_server;

		// Publishers:
		static void streamAllData();
		static ros::Publisher objectCount_publisher;

		// Subscribers:
		static void addStatusbarMessage_callback(const std_msgs::String::ConstPtr& msg);
		static ros::Subscriber addStatusBarMessage_subscriber;
};

#endif
