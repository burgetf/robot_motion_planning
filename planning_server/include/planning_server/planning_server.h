/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: planning_server.h
 */

#ifndef PLANNING_SERVER_H_
#define PLANNING_SERVER_H_

#include <ros/ros.h>
#include <planning_server/planning_control_center.h>

namespace planning_server
{
//* NeurobotsDatabase
/**
* Class providing services to query the state of the world
*/
class PlanningServer
{
public:
    /*!
    * \brief PlanningServer Constructor
    */
	PlanningServer();
    /*!
    * \brief PlanningServer Destructor
    */
	~PlanningServer();

protected:
	//World Database Object
    PlanningControlCenter m_planning_control_center;    /**< Planning control center providing service callbacks */

    //General Services
	ros::ServiceServer m_generate_start_goal_config; /**< Service generating a start and goal configuration for a desired start and end end-effector pose */

    //Service for planner setup
    ros::ServiceServer m_set_planning_scene; /**< Service to set planning scene information */
    ros::ServiceServer m_set_edge_costs; /**< Service to set edge costs used by the planner */

    //Services for Planning Execution
    ros::ServiceServer m_run_planner_configs;
    ros::ServiceServer m_run_planner_configs_file;
    ros::ServiceServer m_run_planner_poses;
    ros::ServiceServer m_run_planner_start_config_goal_pose;

    //Services for Trajectory Execution
    ros::ServiceServer m_execute_trajectory;

private:


	
};

}
#endif /* PLANNING_SERVER_H_ */

