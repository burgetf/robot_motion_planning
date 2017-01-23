/*
 * feasibility_checker.h
 *
 *  Created on: Oct 06, 2015
 *      Author: Felix Burget
 */

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <planner_data_structures/data_structs.h>
//#include <birrt_star_algorithm/data_structs.h>
#include <kuka_motion_control/control_laws.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/GetPlanningScene.h>

#ifndef FEASIBILITY_CHECKER_H
#define FEASIBILITY_CHECKER_H

// --Namespaces --
using namespace std;

namespace state_feasibility_checker{

class FeasibilityChecker
{	
	public:
    FeasibilityChecker(string robot_desciption_param, string planning_group, string ns_prefix = "");
	~FeasibilityChecker();
	//~FeasibilityCheckerBase() {}
	
    //Set Planning Scene Service Name
    void setPlanningSceneServiceName(string service_name);

	//Set step width for collision checking along an tree edge (only used in "isEdgeValid(Edge tree_edge)") 
	void setCollisionCheckingStepWidth(double step_width);
	
	//Checking an Edge of for collisions with obstacles
    bool isEdgeValid(Edge tree_edge, bool print_contacts = false);
    bool isEdgeValid(Edge tree_edge, int &last_valid_node_idx, bool print_contacts = false);
    //Checking a Config for collisions with obstacles
    bool isConfigValid(vector<double> config, bool print_contacts = false);
    bool isConfigValid(KDL::JntArray config, bool print_contacts = false);

    //Definition of virtual methods (used for solution path smoothing)
    //bool ConfigFeasible(const ParabolicRamp::Vector& x);
    //bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b);
	
	private:

    //Node handle
    ros::NodeHandle m_nh;
    
	//Planning group
	string m_planning_group;

    //Planning frame (from virtual joint in srdf)
    string m_planning_frame;

    //Namespace prefix for robot
    string m_ns_prefix_robot;

    //Name of Planning Scene Service;
    string m_planning_scene_service;
	
	//Kinematic Chain of robot
    KDL::Chain m_manipulator_chain;
    
	//Joint names
    vector<string> m_joint_names;

    //Number of joints
    int m_num_joints;
    
	//Generate KDL Tree from robot urdf
    boost::shared_ptr<kuka_motion_controller::KDLRobotModel> m_KDLRobotModel;
    
    //Planning Scene Monitor (used for Collision Checking)
    planning_scene_monitor::PlanningSceneMonitorPtr m_planning_scene_monitor;
    
    //Step width along an tree edge for collision checking
    double m_collision_check_extend_step_factor;
    
    //Perform a step from a node towards another node
    bool stepAlongEdge(Node start_node, Node &end_node, double extend_step_factor);
		

};

} //end of namespace

#endif // BiRRT_STAR_H

