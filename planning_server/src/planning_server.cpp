/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: planning_server.cpp
 */

#include <planning_server/planning_server.h>

namespace planning_server
{

//Constructor
PlanningServer::PlanningServer()
{
	ros::NodeHandle nh("~");

    //Bind General Services
	m_generate_start_goal_config = nh.advertiseService("generate_start_goal_config", &PlanningControlCenter::generate_start_goal_config, &m_planning_control_center);

    //Bind Planner Setup Services
    m_set_planning_scene = nh.advertiseService("set_planning_scene_info", &PlanningControlCenter::set_planning_scene_info, &m_planning_control_center);
    m_set_edge_costs = nh.advertiseService("set_planner_edge_costs", &PlanningControlCenter::set_edge_cost_weights, &m_planning_control_center);

    //Bind Planner Run Services
    m_run_planner_configs = nh.advertiseService("run_planner_start_goal_config", &PlanningControlCenter::run_planner_start_goal_config, &m_planning_control_center);
    m_run_planner_configs_file = nh.advertiseService("run_planner_configs_file", &PlanningControlCenter::run_planner_configs_file, &m_planning_control_center);
    m_run_planner_poses = nh.advertiseService("run_planner_poses", &PlanningControlCenter::run_planner_poses, &m_planning_control_center);
    m_run_planner_start_config_goal_pose = nh.advertiseService("run_planner_start_config_goal_pose", &PlanningControlCenter::run_planner_start_config_goal_pose, &m_planning_control_center);


    //Bind Trajectory Execution Services
    m_execute_trajectory = nh.advertiseService("execute_motion_plan", &PlanningControlCenter::execute_motion_plan, &m_planning_control_center);
}

//Destructor
PlanningServer::~PlanningServer()
{
}

} /* namespace planning_server */


