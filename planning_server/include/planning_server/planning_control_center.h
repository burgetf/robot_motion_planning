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
//#include <planner_msgs/execute_motion_plan.h>
#include <planner_msgs/reset_planner_to_default.h>
#include <planner_msgs/run_planner_map_goal_pose.h>
#include <planner_msgs/run_planner_map_goal_config.h>

//Include Planners
#include <birrt_star_algorithm/birrt_star.h>
#include <rrt_star_algorithm/rrt_star.h>

//#include <motion_trajectory_execution/trajectory_execution_rviz.h>

#include<eigen_conversions/eigen_msg.h>

namespace planning_server
{
//* PlanningControlCenter
/**
 * Class providing planning callbacks
 */
class PlanningControlCenter
{
public:
	PlanningControlCenter();
	virtual ~PlanningControlCenter();

    //+++++++++ General Service Callbacks +++++++++
	bool generate_start_goal_config(planner_msgs::generate_start_goal_config::Request& req,
			planner_msgs::generate_start_goal_config::Response& res); /**< Service callback to generate start and goal configuration given a start and goal end-effector pose */
	

    bool set_planning_scene_info(planner_msgs::planning_scene_info::Request& req,
            planner_msgs::planning_scene_info::Response& res); /**< Service callback to set planning scene information */


    bool reset_planner_to_default(planner_msgs::reset_planner_to_default::Request& req,
                                  planner_msgs::reset_planner_to_default::Response& res); /**< Service callback to reset planner to default configuration */

    //+++++++++ Planner Configuration Service Callbacks +++++++++
    bool set_edge_cost_weights(planner_msgs::set_edge_costs::Request& req,
                               planner_msgs::set_edge_costs::Response& res); /**< Service callback to set egde costs for planner */



    //+++++++++ Planning Service Callbacks (Configurations/Poses given w.r.t base_link frame) +++++++++

    bool run_planner_start_goal_config(planner_msgs::run_planner_configs::Request& req,
                               planner_msgs::run_planner_configs::Response& res); /**< Service callback to run planner with given start and goal config */

    bool run_planner_configs_file(planner_msgs::run_planner_config_file::Request& req,
                               planner_msgs::run_planner_config_file::Response& res); /**< Service callback to run planner with configs from file */

    bool run_planner_start_config_goal_pose(planner_msgs::run_planner_start_config_goal_pose::Request& req,
                                            planner_msgs::run_planner_start_config_goal_pose::Response& res); /**< Service callback to run planner with start config and goal EE pose */

    bool run_planner_poses(planner_msgs::run_planner_poses::Request& req,
                               planner_msgs::run_planner_poses::Response& res); /**< Service callback to run planner with start and goal EE pose */


    //+++++++++ Planning Service Callbacks (Configurations/Poses given w.r.t map frame) +++++++++

    bool run_planner_map_goal_pose(planner_msgs::run_planner_map_goal_pose::Request& req,
                               planner_msgs::run_planner_map_goal_pose::Response& res); /**< Service callback to run planner with current configuration as start config and desired EE goal pose */

    bool run_planner_map_goal_config(planner_msgs::run_planner_map_goal_config::Request& req,
                               planner_msgs::run_planner_map_goal_config::Response& res); /**< Service callback to run planner with current configuration as start config and desired goal config */


protected:

    //Run planner
    bool solve_planning_problem(string planner_type, bool flag_iter_or_time, double max_iter_time, bool tree_optimization, bool informed_sampling, int planner_run_number, bool show_tree_vis);

    //ros::Publisher m_databaseChange; /**< Publisher: sends a message with database information and can be used as a trigger */

private:

    //Node Handle
    ros::NodeHandle nh_;

    //Planning Group
    std::string m_planning_group;

    //Bidirectional and Unidirectional RRT
    boost::shared_ptr<birrt_star_motion_planning::BiRRTstarPlanner> m_bi_planner;
    boost::shared_ptr<rrt_star_motion_planning::RRTstarPlanner> m_uni_planner;


};

} /* namespace planning_server */

#endif /* PLANNING_CONTROL_CENTER_H_ */
