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
//* PlanningServer
/**
* Class providing services for motion planning
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

    //+++++++++ General Services +++++++++

    //Generate a start and goal configuration for a given start and goal end-effector pose
	ros::ServiceServer m_generate_start_goal_config; /**< Service generating a start and goal configuration for a desired start and end end-effector pose */

    //Service to set the planning scene information (environment size and obstacles)
    ros::ServiceServer m_set_planning_scene; /**< Service to set planning scene information */

    //Reset Planner to default configuration
    ros::ServiceServer m_reset_planner_to_default; /**< Service to reset plannerto default configuration (i.e. default edge cost weights, no constraints active etc.) */


    //+++++++++ Services for Planner Configuration +++++++++

    ros::ServiceServer m_set_edge_costs; /**< Service to set edge costs used by the planner */


    //+++++++++ Services for Planning (Configurations/Poses given w.r.t base_link frame) +++++++++

    //Run planner with a start and goal configuration (either manually specified or loaded from a file)
    ros::ServiceServer m_run_planner_configs; /**< Service to run planner with a manually specified start and goal configuration */
    ros::ServiceServer m_run_planner_configs_file; /**< Service to run planner with a start and goal configuration loaded from a file*/

    //Run planner with a start config and a goal end-effector pose (-> Planner will generate config for end-effector goal pose)
    ros::ServiceServer m_run_planner_start_config_goal_pose; /**< Service to run planner with a start configuration and desired end-effector goal pose */

    //Run planner with a start and goal end-effector pose (-> Planner will generate config for end-effector start and goal pose)
    ros::ServiceServer m_run_planner_poses; /**< Service to run planner with a desired start and end-effector goal pose */


    //+++++++++ Services for Planning (Configurations/Poses given w.r.t map frame) +++++++++

    //Run planner with current configuration as start config and a desired end-effector goal pose (-> Planner will generate config for end-effector goal pose)
    ros::ServiceServer m_run_planner_map_goal_pose; /**< Service to run planner with current configuration as start config and desired end-effector goal pose expressed w.r.t map frame */

    //Run planner with current configuration as start config and a desired goal config
    ros::ServiceServer m_run_planner_map_goal_config; /**< Service to run planner with current configuration as start config and desired goal config expressed w.r.t map frame*/

    //TODO:
    // -> Add service "m_run_planner_map_start_goal_pose"
    // -> Add service "m_run_planner_map_start_goal_config"

    //TODO:
    // -> Include in the existing srv files parameters for setting geometric end-effector constraints


private:


	
};

}
#endif /* PLANNING_SERVER_H_ */

