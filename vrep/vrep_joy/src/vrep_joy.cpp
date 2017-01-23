/*
 * =====================================================================================
 *
 *       Filename:  vrep_joy.cpp
 *
 *    Description:  V-REP joystick node 
 *
 *        Version:  1.0
 *        Created:  20/09/2013 12:47:23
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Roberto Marino (rm), formica@member.fsf.org
 *   Organization:  University of Genova
 *
 * =====================================================================================
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class VrepJoy
{
	public:
		VrepJoy();

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;
		ros::Subscriber joy_sub_;
		ros::Publisher  vrep_joy_pub_;
};

VrepJoy::VrepJoy()
{
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",10,&VrepJoy::joyCallback, this);
	vrep_joy_pub_ = nh_.advertise<sensor_msgs::Joy>("vrep/joy",1);
}

void VrepJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	vrep_joy_pub_.publish(joy);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv, "vrep_joy");
	VrepJoy vrep_joy;
	ros::spin();
}




