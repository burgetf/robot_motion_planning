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

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <vrep_ros_interface/v_repConst.h>

// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosSetObjectIntParameter.h"
#include "vrep_common/simRosSetRobotPose.h"


// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
//bool sensorTrigger=false;
float simulationTime=0.0f;


// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}


//void sensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens)
//{
//	// We don't care about the detected distance here, we just trigger!
//	sensorTrigger=true;
//}


// Main code:
int main(int argc,char* argv[])
{

    // The joint handles are given in the argument list
	// (when V-REP launches this executable, V-REP will also provide the argument list)
    //Motor handles for omnirob wheels and lbr joints
    std::vector<int> robot_motor_handles;

    //Get Joint Handles
    if (argc>=7)
	{

        for(int i = 1; i < argc ; i++)
            robot_motor_handles.push_back(atoi(argv[i]));

	}
	else
	{
        printf("Indicate following arguments: 'RLWheelMotorHandle RRWheelMotorHandle FLWheelMotorHandle FRWheelMotorHandle LBR_joint1Handle LBR_joint2Handle LBR_joint3Handle LBR_joint4Handle LBR_joint5Handle LBR_joint6Handle LBR_joint7Handle sdh2_finger_joint_12 sdh2_finger_joint_22 sdh2_thumb_joint_2 sdh2_finger_joint_13 sdh2_finger_joint_23 sdh2_thumb_joint_3'!\n");
		sleep(5000);
		return 0;
	}

	// Create a ROS node. The name has a random component: 
	int _argc = 0;
	char** _argv = NULL;
    //struct timeval tv;
    //unsigned int timeVal=0;
    //if (gettimeofday(&tv,NULL)==0)
    //	timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
    std::string nodeName("vrep_ros_communication");
    //std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
    //nodeName+=randId;

    //Init ROS
	ros::init(_argc,_argv,nodeName.c_str());

	if(!ros::master::check())
		return(0);
	
    //Create ROS Node
	ros::NodeHandle node("~");	
    printf("VREP_ROS_COMMUNICATION just started with node name %s\n",nodeName.c_str());

    //Publish the motor handles
    ros::Publisher motor_handles_pub = node.advertise<std_msgs::Int32MultiArray>("motor_handles",1);

    //Prepare message
    std_msgs::Int32MultiArray motor_handles_array;
    //Clear array
    motor_handles_array.data.clear();
    //for loop, pushing data in the size of the array
    for (int i = 0; i < robot_motor_handles.size(); i++)
    {
        //assign array a random number between 0 and 255.
        motor_handles_array.data.push_back(robot_motor_handles[i]);
    }


	// 1. Let's subscribe to V-REP's info stream (that stream is the only one enabled by default,
	// and the only one that can run while no simulation is running):
	ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);

    // 2. Let's tell V-REP to subscribe to the motor speed topic (publisher to that topic will be created further down):
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");


    // 3. Enable Subscribers in V-Rep (for Omnirob wheel,Kuka LBR and Schunk Hand joint speeds)
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_omnirob_wheels;
    srv_enableSubscriber_omnirob_wheels.request.topicName="/"+nodeName+"/omnirob_wheels"; // the topic name
    srv_enableSubscriber_omnirob_wheels.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_omnirob_wheels.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    vrep_common::simRosEnableSubscriber srv_enableSubscriber_robotino_wheels;
    srv_enableSubscriber_robotino_wheels.request.topicName="/"+nodeName+"/robotino_wheels"; // the topic name
    srv_enableSubscriber_robotino_wheels.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_robotino_wheels.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type



    vrep_common::simRosEnableSubscriber srv_enableSubscriber_lbr_joints;
    srv_enableSubscriber_lbr_joints.request.topicName="/"+nodeName+"/lbr_joints"; // the topic name
    srv_enableSubscriber_lbr_joints.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_lbr_joints.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    vrep_common::simRosEnableSubscriber srv_enableSubscriber_schunk_sdh2_joints;
    srv_enableSubscriber_schunk_sdh2_joints.request.topicName="/"+nodeName+"/schunk_sdh2_joints"; // the topic name
    srv_enableSubscriber_schunk_sdh2_joints.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_schunk_sdh2_joints.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

//    vrep_common::simRosEnableSubscriber srv_enableSubscriber_joint_control_modes;
//    srv_enableSubscriber_joint_control_modes.request.topicName="/"+nodeName+"/joints_control_modes"; // the topic name
//    srv_enableSubscriber_joint_control_modes.request.queueSize=1; // the subscriber queue size (on V-REP side)
//    srv_enableSubscriber_joint_control_modes.request.streamCmd=sim_jointintparam_ctrl_enabled; // the subscriber type


    //Check whether subscribers have been enabled successfully
    if ( client_enableSubscriber.call(srv_enableSubscriber_omnirob_wheels)&&(srv_enableSubscriber_omnirob_wheels.response.subscriberID!=-1) \
         && client_enableSubscriber.call(srv_enableSubscriber_lbr_joints)&&(srv_enableSubscriber_lbr_joints.response.subscriberID!=-1) \
         && client_enableSubscriber.call(srv_enableSubscriber_schunk_sdh2_joints)&&(srv_enableSubscriber_schunk_sdh2_joints.response.subscriberID!=-1) \
         && client_enableSubscriber.call(srv_enableSubscriber_robotino_wheels)&&(srv_enableSubscriber_robotino_wheels.response.subscriberID!=-1))
    {	// ok, the service call was ok, and the subscriber was succesfully started on V-REP side
        // V-REP is now listening to the desired motor joint states

        // 3. Let's prepare a publisher of the wheels and joints motor speeds:
        //ros::Publisher omnirob_motor_speedPub=node.advertise<vrep_common::JointSetStateData>("omnirob_wheels",1);
        //ros::Publisher lbr_motor_speedPub=node.advertise<vrep_common::JointSetStateData>("lbr_joints",1);
        //ros::Publisher schunk_sdh2_motor_speedPub=node.advertise<vrep_common::JointSetStateData>("schunk_sdh2_joints",1);


        //Motor Speed Messages to be published
        //vrep_common::JointSetStateData omnirob_wheels_motor_speeds;
        //vrep_common::JointSetStateData lbr_motor_speeds;
        //vrep_common::JointSetStateData schunk_sdh2_motor_speeds;
        //vrep_common::simRosSetObjectIntParameter


        // 6. Finally we have the control loop
        while (ros::ok()&&simulationRunning)
        { // this is the control loop (very simple, just as an example)

            //Continuously publish motor handles array
            motor_handles_pub.publish(motor_handles_array);

            // handle ROS messages:
            ros::spinOnce();

            // sleep a bit:
            usleep(5000);
        }
    }

	ros::shutdown();
    printf("VREP_ROS_COMMUNICATION just ended!\n");
	return(0);
}
