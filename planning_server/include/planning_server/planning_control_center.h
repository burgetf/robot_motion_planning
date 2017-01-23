/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: planning_control_center.h
 */
 

#ifndef PLANNING_CONTROL_CENTER_H_
#define PLANNING_CONTROL_CENTER_H_


#include <ros/ros.h>

//Inclide Messages and Services
#include <planner_msgs/generate_start_goal_config.h>
#include <planner_msgs/planning_scene_info.h>
#include <planner_msgs/set_edge_costs.h>
#include <planner_msgs/run_planner_configs.h>
#include <planner_msgs/run_planner_config_file.h>
#include <planner_msgs/run_planner_poses.h>
#include <planner_msgs/run_planner_start_config_goal_pose.h>
#include <planner_msgs/execute_motion_plan.h>


//Include Planners
#include <birrt_star_algorithm/birrt_star.h>
#include <rrt_star_algorithm/rrt_star.h>

#include <motion_trajectory_execution/trajectory_execution_rviz.h>

namespace planning_server
{
//* PlanningServer
/**
 * Class holding the state of the world
 */
class PlanningControlCenter
{
public:
	PlanningControlCenter();
	virtual ~PlanningControlCenter();

    //General Service Callbacks
	bool generate_start_goal_config(planner_msgs::generate_start_goal_config::Request& req,
			planner_msgs::generate_start_goal_config::Response& res); /**< Service callback to generate start and goal configuration given a start and goal end-effector pose */
	
    //Planner Setup Service Callbacks
    bool set_planning_scene_info(planner_msgs::planning_scene_info::Request& req,
            planner_msgs::planning_scene_info::Response& res); /**< Service callback to set planning scene information */


    bool set_edge_cost_weights(planner_msgs::set_edge_costs::Request& req,
                               planner_msgs::set_edge_costs::Response& res); /**< Service callback to set egde costs for planner */

    //Planner Run Service Callbacks
    bool run_planner_start_goal_config(planner_msgs::run_planner_configs::Request& req,
                               planner_msgs::run_planner_configs::Response& res); /**< Service callback to run planner with given start and goal config */

    bool run_planner_configs_file(planner_msgs::run_planner_config_file::Request& req,
                               planner_msgs::run_planner_config_file::Response& res); /**< Service callback to run planner with configs from file */

    bool run_planner_poses(planner_msgs::run_planner_poses::Request& req,
                               planner_msgs::run_planner_poses::Response& res); /**< Service callback to run planner with start and goal EE pose */

    bool run_planner_start_config_goal_pose(planner_msgs::run_planner_start_config_goal_pose::Request& req,
                                            planner_msgs::run_planner_start_config_goal_pose::Response& res); /**< Service callback to run planner with start config and goal EE pose */

    //Trajectory Execution Service Callbacks
    bool execute_motion_plan(planner_msgs::execute_motion_plan::Request& req,
                              planner_msgs::execute_motion_plan::Response& res); /**< Service callback to execute planned joint trajectory */




private:

    //Node Handle
    ros::NodeHandle nh_;

    //Planning Group
    std::string m_planning_group;

    //Bidirectional and Unidirectional RRT
    boost::shared_ptr<birrt_star_motion_planning::BiRRTstarPlanner> m_bi_planner;
    boost::shared_ptr<rrt_star_motion_planning::RRTstarPlanner> m_uni_planner;

    //Trajectory Execution
    boost::shared_ptr<trajectory_execution::MotionCommanderRVIZ> m_trajectory_execution;


	

protected:
	
    //Run planner
    bool execute_planner_run(string planner_type, bool flag_iter_or_time, double max_iter_time, bool tree_optimization, bool informed_sampling, int planner_run_number, bool show_tree_vis);

	//ros::Publisher m_databaseChange; /**< Publisher: sends a message with database information and can be used as a trigger */

};

} /* namespace planning_server */

#endif /* PLANNING_CONTROL_CENTER_H_ */
