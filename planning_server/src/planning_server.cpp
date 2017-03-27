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

    //+++++++++ Bind General Services +++++++++
	m_generate_start_goal_config = nh.advertiseService("generate_start_goal_config", &PlanningControlCenter::generate_start_goal_config, &m_planning_control_center);

    m_set_planning_scene = nh.advertiseService("set_planning_scene_info", &PlanningControlCenter::set_planning_scene_info, &m_planning_control_center);

    m_reset_planner_to_default = nh.advertiseService("reset_planner_to_default", &PlanningControlCenter::reset_planner_to_default, &m_planning_control_center);


    //+++++++++ Bind Planner Configuration Services +++++++++

    m_set_edge_costs = nh.advertiseService("set_planner_edge_costs", &PlanningControlCenter::set_edge_cost_weights, &m_planning_control_center);

    //TODO:
    // -> Services for setting constraints

    //+++++++++ Bind Services for Planning (Configurations/Poses given w.r.t base_link frame) +++++++++

    //Run planner with terminal configs as input
    m_run_planner_configs = nh.advertiseService("run_planner_start_goal_config", &PlanningControlCenter::run_planner_start_goal_config, &m_planning_control_center);
    m_run_planner_configs_file = nh.advertiseService("run_planner_configs_file", &PlanningControlCenter::run_planner_configs_file, &m_planning_control_center);

    //Run planner with start config and ee goal pose as input
    m_run_planner_start_config_goal_pose = nh.advertiseService("run_planner_start_config_goal_pose", &PlanningControlCenter::run_planner_start_config_goal_pose, &m_planning_control_center);

    //Run planner with start and goal ee pose as input
    m_run_planner_poses = nh.advertiseService("run_planner_poses", &PlanningControlCenter::run_planner_poses, &m_planning_control_center);

    //+++++++++ Bind Services for Planning Execution (Configurations/Poses given w.r.t map frame) +++++++++

    //Run planner with current configuration as start config and a desired end-effector goal pose (-> Planner will generate config for end-effector goal pose)
    m_run_planner_map_goal_pose = nh.advertiseService("run_planner_map_goal_pose", &PlanningControlCenter::run_planner_map_goal_pose, &m_planning_control_center);

    //Run planner with current configuration as start config and a desired goal config
    m_run_planner_map_goal_config = nh.advertiseService("run_planner_map_goal_config", &PlanningControlCenter::run_planner_map_goal_config, &m_planning_control_center);


    //TODO:
    // -> Definiton for service "m_run_planner_map_start_goal_pose"
    // -> Definiton for service "m_run_planner_map_start_goal_config"


}

//Destructor
PlanningServer::~PlanningServer()
{
}

} /* namespace planning_server */


