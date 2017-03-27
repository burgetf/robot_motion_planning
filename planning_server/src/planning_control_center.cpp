/*
 * Copyright (c) 2016 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 2, 2016
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: planning_control_center.cpp
 */

#include <ros/ros.h>

#include <planning_server/planning_control_center.h>
//#include <database_msgs/database_change.h>


namespace planning_server
{

//Constructor
PlanningControlCenter::PlanningControlCenter(): nh_("~")
{
    //Read planning group name from parameter server
    nh_.param("planning_group", m_planning_group, std::string("omnirob_lbr_sdh"));

    ROS_INFO_STREAM("Planning group for planning set to: " << m_planning_group);

    //Generate planners
    m_bi_planner = boost::shared_ptr<birrt_star_motion_planning::BiRRTstarPlanner>(new birrt_star_motion_planning::BiRRTstarPlanner(m_planning_group));
    m_uni_planner = boost::shared_ptr<rrt_star_motion_planning::RRTstarPlanner>(new rrt_star_motion_planning::RRTstarPlanner(m_planning_group));

    //Trajectory Execution
    //m_trajectory_execution = boost::shared_ptr<trajectory_execution::MotionCommanderRVIZ>(new trajectory_execution::MotionCommanderRVIZ(m_planning_group));


	//messages
	//m_databaseChange = nh.advertise<database_msgs::database_change>("database_change", 5);
}

//Destructor
PlanningControlCenter::~PlanningControlCenter()
{
}

//+++++++++ General Service Callbacks +++++++++

//Generate start and goal config given a start and end end-effector pose
bool PlanningControlCenter::generate_start_goal_config(planner_msgs::generate_start_goal_config::Request &req,
		planner_msgs::generate_start_goal_config::Response &res)
{

    //Initialize constraint ee coordinates and permitted target deviation
    vector<pair<double,double> > target_coordinate_dev_local(6);
    vector<int> constraint_vec_start_pose(6);
    vector<int> constraint_vec_goal_pose(6);
    for(int c = 0; c < target_coordinate_dev_local.size(); c++)
    {
        //Set all ee target pose coordinates to be constraint
        constraint_vec_start_pose[c] = 1;
        constraint_vec_goal_pose[c] = 1;
        //Deviation for ee position
        if(c < 3)
        {
            target_coordinate_dev_local[c].first = -0.0005;
            target_coordinate_dev_local[c].second = 0.0005;
        }
        //Deviation for ee orientation
        else{
            target_coordinate_dev_local[c].first =  -0.0005;
            target_coordinate_dev_local[c].second =  0.0005;
        }
    }


    //Set constraint ee target coordinates
    for(int c = 0; c < req.constraint_vec_goal_pose.size(); c++)
    {
        constraint_vec_start_pose[c] = req.constraint_vec_start_pose[c];
        constraint_vec_goal_pose[c] = req.constraint_vec_goal_pose[c];
    }

    //Permitted displacement for ee target coordinates w.r.t desired target frame (Convert message request into vector of pairs)
    for(int c = 0; c < req.target_coordinate_dev.array2d_double.size(); c++)
    {
        target_coordinate_dev_local[c].first = req.target_coordinate_dev.array2d_double[c].array1d_double[0];    //negative deviation [m or rad]
        target_coordinate_dev_local[c].second = req.target_coordinate_dev.array2d_double[c].array1d_double[1];   //positive deviation [m or rad]
    }


//    if(req.target_coordinate_dev.array2d_double.size() != 6 || req.constraint_vec_start_pose.size() != 6 || req.constraint_vec_goal_pose.size() != 6)
//    {
//        ROS_ERROR("Constraint vectors need to be 6-dimensional (pos+rot dev)");
//        res.success = false;
//        return false;
//    }

//    //Permitted displacement for ee coordinates w.r.t desired target frame
//    //Convert message request into vector of pairs
//    vector<pair<double,double> > target_coordinate_dev_local(6);
//    for(int c = 0; c < target_coordinate_dev_local.size(); c++)
//    {
//        target_coordinate_dev_local[c].first = req.target_coordinate_dev.array2d_double[c].array1d_double[0];    //negative deviation [m or rad]
//        target_coordinate_dev_local[c].second = req.target_coordinate_dev.array2d_double[c].array1d_double[1];   //positive deviation [m or rad]
//    }


    //Generate start and goal config
    vector<vector<double> > start_goal_config = m_bi_planner->generate_start_goal_config(req.start_ee_pose, constraint_vec_start_pose, req.goal_ee_pose, constraint_vec_goal_pose, target_coordinate_dev_local, req.show_motion);

    //Store start and goal config in service response
    res.start_conf = start_goal_config[0];
    res.goal_conf = start_goal_config[1];
    res.success = true;

    return true;
}



//Set the planning scene information
bool PlanningControlCenter::set_planning_scene_info(planner_msgs::planning_scene_info::Request& req,
        planner_msgs::planning_scene_info::Response& res)
{

    if(req.env_size_x.size() != 2 || req.env_size_y.size() != 2)
    {
        ROS_ERROR("Environment dimension in x and y direction must be specified by 2 dimensional vector");
        return false;
    }

    //Set information on planning scene
    m_bi_planner->setPlanningSceneInfo(req.env_size_x, req.env_size_y, "motion_plan", req.show_environment_borders);
    m_uni_planner->setPlanningSceneInfo(req.env_size_x, req.env_size_y, "motion_plan", req.show_environment_borders);

    //Planning Info set successfully
    res.ok = true;

    return true;
}

bool PlanningControlCenter::reset_planner_to_default(planner_msgs::reset_planner_to_default::Request& req,
                              planner_msgs::reset_planner_to_default::Response& res)
{
    //Reset planner to default configuration (as specified in the Constructor)
    m_bi_planner->reset_planner_and_config();
    m_uni_planner->reset_planner_and_config();

    //Return success
    res.reinitialized = true;
    return true;
}

 //+++++++++ Planner Configuration Service Callbacks +++++++++

bool PlanningControlCenter::set_edge_cost_weights(planner_msgs::set_edge_costs::Request& req,
                           planner_msgs::set_edge_costs::Response& res)
{
    //Get the number of joints the planning group is composed of
    int num_joints = m_bi_planner->getNumJointsPlanningGroup();

    if(req.edge_costs.size() != num_joints)
    {
        ROS_ERROR("Dimension of edge cost weight vector does not match number of joints of the planning group!!!");
        ROS_ERROR_STREAM("Number of joints in planning group is: " << num_joints);
        res.ok = false;
    }
    else
    {
        //Set edge cost weights used for planning
        m_bi_planner->setEdgeCostWeights(req.edge_costs);
        m_uni_planner->setEdgeCostWeights(req.edge_costs);

        //Planning Info set successfully
        res.ok = true;
    }

    return res.ok;
}


//+++++++++ Planning Execution Service Callbacks +++++++++

//Run planner with a start and goal configuration, specified manually by the user
bool PlanningControlCenter::run_planner_start_goal_config(planner_msgs::run_planner_configs::Request& req,
                           planner_msgs::run_planner_configs::Response& res)
{

    //Initialization flag
    bool initialization_ok = true;

    if(req.planner_type == "bi")
    {
        //Reset planner data
        m_bi_planner->reset_planner_only();
        //Init planner with start and goal config
        initialization_ok = m_bi_planner->init_planner(req.start_config, req.goal_config,1); //1 = C-Space search space
    }
    else if(req.planner_type == "uni")
    {
        //Reset planner data
        m_uni_planner->reset_planner_only();
        //Init planner with start and goal config
        initialization_ok = m_uni_planner->init_planner(req.start_config, req.goal_config,1); //1 = C-Space search space
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi' or 'uni')");
        //Return failure
        res.success = false;
        return false;
    }

    if(initialization_ok)
    {
        //Execute Planner run
        res.success  = solve_planning_problem(req.planner_type, req.flag_iter_or_time, req.iter_or_time, req.tree_optimization, req.informed_sampling, req.run_id, req.show_tree);
    }
    else
    {
        ROS_ERROR("Planner initialization failed");
        //Return failure
        res.success = false;
        return false;
    }

    //Check planning result
    if(res.success == false)
        return false;
    else
    {

        //Joint and End-effector Trajectory
        vector<vector<double> > jt;
        vector<vector<double> > eet;

        //If planning succeeded -> Store robot trajectories in service response
        if(req.planner_type == "bi")
        {
            //Get Trajectories
            jt =  m_bi_planner->getJointTrajectory();
            eet =  m_bi_planner->getEndeffectorTrajectory();
        }
        if(req.planner_type == "uni")
        {
            //Get Trajectories
            jt =  m_uni_planner->getJointTrajectory();
            eet =  m_uni_planner->getEndeffectorTrajectory();
        }

        //Set size for response container
        res.joint_trajectory.array2d_double.resize(jt.size());
        res.ee_trajectory.array2d_double.resize(eet.size());

        //Convert trajectories to response type
        for(int i = 0 ; i < jt.size() ; i++)
        {
            res.joint_trajectory.array2d_double[i].array1d_double = jt[i];
            res.ee_trajectory.array2d_double[i].array1d_double = eet[i];
        }

        return true;
    }

}


//Run planner with a start and goal configuration loaded from a file
bool PlanningControlCenter::run_planner_configs_file(planner_msgs::run_planner_config_file::Request& req,
                           planner_msgs::run_planner_config_file::Response& res)
{
    //Path to the file containing the start and goal config for the scenario
    char* file_path_start_goal_config;
    file_path_start_goal_config = new char[req.path_configs_file.size() + 1];
    copy(req.path_configs_file.begin(), req.path_configs_file.end(), file_path_start_goal_config);
    file_path_start_goal_config[req.path_configs_file.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_start_goal_config<<endl;


    //Initialization flag
    bool initialization_ok = true;

    if(req.planner_type == "bi")
    {
        //Reset planner data
        m_bi_planner->reset_planner_only();
        //Init planner with start and goal config from file
        initialization_ok = m_bi_planner->init_planner(file_path_start_goal_config, 1); //1 = C-Space search space
    }
    else if(req.planner_type == "uni")
    {
        //Reset planner data
        m_uni_planner->reset_planner_only();
        //Init planner with start and goal config from file
        initialization_ok = m_uni_planner->init_planner(file_path_start_goal_config, 1); //1 = C-Space search space
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi'' or 'uni')");
        //Return failure
        res.success = false;
        return false;
    }

    if(initialization_ok)
    {
        //Execute Planner run
        res.success = solve_planning_problem(req.planner_type, req.flag_iter_or_time, req.iter_or_time, req.tree_optimization, req.informed_sampling, req.run_id, req.show_tree);
    }
    else
    {
        ROS_ERROR("Planner initialization failed");
        //Return failure
        res.success = false;
        return false;
    }


    //Check planning result
    if(res.success == false)
        return false;
    else
    {
        //Joint and End-effector Trajectory
        vector<vector<double> > jt;
        vector<vector<double> > eet;

        //If planning succeeded -> Store robot trajectories in service response
        if(req.planner_type == "bi")
        {
            //Get Trajectories
            jt =  m_bi_planner->getJointTrajectory();
            eet =  m_bi_planner->getEndeffectorTrajectory();
        }
        if(req.planner_type == "uni")
        {
            //Get Trajectories
            jt =  m_uni_planner->getJointTrajectory();
            eet =  m_uni_planner->getEndeffectorTrajectory();
        }

        //Set size for response container
        res.joint_trajectory.array2d_double.resize(jt.size());
        res.ee_trajectory.array2d_double.resize(eet.size());

        //Convert trajectories to response type
        for(int i = 0 ; i < jt.size() ; i++)
        {
            res.joint_trajectory.array2d_double[i].array1d_double = jt[i];
            res.ee_trajectory.array2d_double[i].array1d_double = eet[i];
        }

        return true;
    }

}




//Run planner with a start config and a goal end-effector pose (-> Planner will generate config for end-effector goal pose)
bool PlanningControlCenter::run_planner_start_config_goal_pose(planner_msgs::run_planner_start_config_goal_pose::Request& req,
                                        planner_msgs::run_planner_start_config_goal_pose::Response& res)
{
    //Initialize constraint ee coordinates and permitted target deviation
    vector<pair<double,double> > target_coordinate_dev_local(6);
    vector<int> constraint_vec_goal_pose(6);
    for(int c = 0; c < target_coordinate_dev_local.size(); c++)
    {
        //Set all ee target pose coordinates to be constraint
        constraint_vec_goal_pose[c] = 1;
        //Deviation for ee position
        if(c < 3)
        {
            target_coordinate_dev_local[c].first = -0.005;
            target_coordinate_dev_local[c].second = 0.005;
        }
        //Deviation for ee orientation
        else{
            target_coordinate_dev_local[c].first =  -0.05;
            target_coordinate_dev_local[c].second =  0.05;
        }
    }


    //Set constraint ee target coordinates
    for(int c = 0; c < req.constraint_vec_goal_pose.size(); c++)
    {
        constraint_vec_goal_pose[c] = req.constraint_vec_goal_pose[c];
    }

    //Permitted displacement for ee target coordinates w.r.t desired target frame (Convert message request into vector of pairs)
    for(int c = 0; c < req.target_coordinate_dev.array2d_double.size(); c++)
    {
        target_coordinate_dev_local[c].first = req.target_coordinate_dev.array2d_double[c].array1d_double[0];    //negative deviation [m or rad]
        target_coordinate_dev_local[c].second = req.target_coordinate_dev.array2d_double[c].array1d_double[1];   //positive deviation [m or rad]
    }


    //Initialization flag
    bool initialization_ok = true;

    if(req.planner_type == "bi")
    {
        //Reset planner data
        m_bi_planner->reset_planner_only();
        //Generate goal config
        vector<double> goal_config = m_bi_planner->generate_config_from_ee_pose_with_reference_config(req.goal_ee_pose, constraint_vec_goal_pose, target_coordinate_dev_local, req.start_config, true);
        //Init planner with start and goal config
        initialization_ok = m_bi_planner->init_planner(req.start_config,goal_config,1); //1 = C-Space search space
    }
    else if(req.planner_type == "uni")
    {
        //Reset planner data
        m_uni_planner->reset_planner_only();
        //Generate goal config
        vector<double> goal_config = m_uni_planner->generate_config_from_ee_pose_with_reference_config(req.goal_ee_pose, constraint_vec_goal_pose, target_coordinate_dev_local, req.start_config, true);
        //Init planner with start and goal config
        initialization_ok = m_uni_planner->init_planner(req.start_config,goal_config,1); //1 = C-Space search space
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi' or 'uni')");
        //Return failure
        res.success = false;
        return false;
    }


    if(initialization_ok)
    {
        //Execute Planner run
        res.success  = solve_planning_problem(req.planner_type, req.flag_iter_or_time, req.iter_or_time, req.tree_optimization, req.informed_sampling,req.run_id, req.show_tree);
    }
    else
    {
        ROS_ERROR("Planner initialization failed");
        //Return failure
        res.success = false;
        return false;
    }

    //Check planning result
    if(res.success == false)
        return false;
    else
    {

        //Joint and End-effector Trajectory
        vector<vector<double> > jt;
        vector<vector<double> > eet;

        //If planning succeeded -> Store robot trajectories in service response
        if(req.planner_type == "bi")
        {
            //Get Trajectories
            jt =  m_bi_planner->getJointTrajectory();
            eet =  m_bi_planner->getEndeffectorTrajectory();
        }
        if(req.planner_type == "uni")
        {
            //Get Trajectories
            jt =  m_uni_planner->getJointTrajectory();
            eet =  m_uni_planner->getEndeffectorTrajectory();
        }

        //Set size for response container
        res.joint_trajectory.array2d_double.resize(jt.size());
        res.ee_trajectory.array2d_double.resize(eet.size());

        //Convert trajectories to response type
        for(int i = 0 ; i < jt.size() ; i++)
        {
            res.joint_trajectory.array2d_double[i].array1d_double = jt[i];
            res.ee_trajectory.array2d_double[i].array1d_double = eet[i];
        }

        return true;
    }
}


//Run planner with a start and goal end-effector pose (-> Planner will generate config for end-effector start and goal pose)
bool PlanningControlCenter::run_planner_poses(planner_msgs::run_planner_poses::Request& req,
                           planner_msgs::run_planner_poses::Response& res)
{
    //Initialize constraint ee coordinates and permitted target deviation
    vector<pair<double,double> > target_coordinate_dev_local(6);
    vector<int> constraint_vec_start_pose(6);
    vector<int> constraint_vec_goal_pose(6);
    for(int c = 0; c < target_coordinate_dev_local.size(); c++)
    {
        //Set all ee target pose coordinates to be constraint
        constraint_vec_start_pose[c] = 1;
        constraint_vec_goal_pose[c] = 1;
        //Deviation for ee position
        if(c < 3)
        {
            target_coordinate_dev_local[c].first = -0.01;
            target_coordinate_dev_local[c].second = 0.01;
        }
        //Deviation for ee orientation
        else{
            target_coordinate_dev_local[c].first =  -0.05;
            target_coordinate_dev_local[c].second =  0.05;
        }
    }


    //Set constraint ee target coordinates
    for(int c = 0; c < req.constraint_vec_goal_pose.size(); c++)
    {
        constraint_vec_start_pose[c] = req.constraint_vec_start_pose[c];
        constraint_vec_goal_pose[c] = req.constraint_vec_goal_pose[c];
    }

    //Permitted displacement for ee target coordinates w.r.t desired target frame (Convert message request into vector of pairs)
    for(int c = 0; c < req.target_coordinate_dev.array2d_double.size(); c++)
    {
        target_coordinate_dev_local[c].first = req.target_coordinate_dev.array2d_double[c].array1d_double[0];    //negative deviation [m or rad]
        target_coordinate_dev_local[c].second = req.target_coordinate_dev.array2d_double[c].array1d_double[1];   //positive deviation [m or rad]
    }

    //Initialization flag
    bool initialization_ok = true;

    if(req.planner_type == "bi")
    {
        //Reset planner data
        m_bi_planner->reset_planner_only();
        //Init planner with start and goal ee pose
        initialization_ok = m_bi_planner->init_planner(req.start_ee_pose, constraint_vec_start_pose, req.goal_ee_pose, constraint_vec_goal_pose, target_coordinate_dev_local, 1); //1 = C-Space search space
    }
    else if(req.planner_type == "uni")
    {
        //Reset planner data
        m_uni_planner->reset_planner_only();
        //Init planner with start and goal ee pose
        initialization_ok = m_uni_planner->init_planner(req.start_ee_pose, constraint_vec_start_pose, req.goal_ee_pose, constraint_vec_goal_pose, target_coordinate_dev_local, 1);
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi'' or 'uni')");
        //Return failure
        res.success = false;
        return false;
    }


    if(initialization_ok)
    {
        //Execute Planner run
        res.success = solve_planning_problem(req.planner_type, req.flag_iter_or_time, req.iter_or_time, req.tree_optimization, req.informed_sampling,req.run_id, req.show_tree);
    }
    else
    {
        ROS_ERROR("Planner initialization failed");
        //Return failure
        res.success = false;
        return false;
    }

    //Check planning result
    if(res.success == false)
        return false;
    else
    {
        //Joint and End-effector Trajectory
        vector<vector<double> > jt;
        vector<vector<double> > eet;

        //If planning succeeded -> Store robot trajectories in service response
        if(req.planner_type == "bi")
        {
            //Get Trajectories
            jt =  m_bi_planner->getJointTrajectory();
            eet =  m_bi_planner->getEndeffectorTrajectory();
        }
        if(req.planner_type == "uni")
        {
            //Get Trajectories
            jt =  m_uni_planner->getJointTrajectory();
            eet =  m_uni_planner->getEndeffectorTrajectory();
        }

        //Set size for response container
        res.joint_trajectory.array2d_double.resize(jt.size());
        res.ee_trajectory.array2d_double.resize(eet.size());

        //Convert trajectories to response type
        for(int i = 0 ; i < jt.size() ; i++)
        {
            res.joint_trajectory.array2d_double[i].array1d_double = jt[i];
            res.ee_trajectory.array2d_double[i].array1d_double = eet[i];
        }

        return true;
    }
}


bool PlanningControlCenter::run_planner_map_goal_pose(planner_msgs::run_planner_map_goal_pose::Request& req,
                           planner_msgs::run_planner_map_goal_pose::Response& res)
{

    //Convert goal pose to Eigen
    Eigen::Affine3d goal_pose;
    tf::poseMsgToEigen (req.goal_pose, goal_pose);

    //Get Number of revolute joints in the planning group (independent of planner 'bi' or 'uni')
    int num_rev_joints = m_bi_planner->getNumRevoluteJointsPlanningGroup();

    //Initialize constraint ee coordinates and permitted target deviation
    vector<pair<double,double> > target_coordinate_dev_local(6);
    vector<int> constraint_vec_goal_pose(6);
    for(int c = 0; c < target_coordinate_dev_local.size(); c++)
    {
        //Set all ee target pose coordinates to be constraint
        constraint_vec_goal_pose[c] = 1;
        //Deviation for ee position
        if(c < 3)
        {
            target_coordinate_dev_local[c].first = -0.01;
            target_coordinate_dev_local[c].second = 0.01;
        }
        //Deviation for ee orientation
        else{
            target_coordinate_dev_local[c].first =  -0.05;
            target_coordinate_dev_local[c].second =  0.05;
        }
    }


    //Set constraint ee target coordinates
    for(int c = 0; c < req.constraint_vec_goal_pose.size(); c++)
    {
        constraint_vec_goal_pose[c] = req.constraint_vec_goal_pose[c];
    }

    //Permitted displacement for ee target coordinates w.r.t desired target frame (Convert message request into vector of pairs)
    for(int c = 0; c < req.target_coordinate_dev.array2d_double.size(); c++)
    {
        target_coordinate_dev_local[c].first = req.target_coordinate_dev.array2d_double[c].array1d_double[0];    //negative deviation [m or rad]
        target_coordinate_dev_local[c].second = req.target_coordinate_dev.array2d_double[c].array1d_double[1];   //positive deviation [m or rad]
    }


    //Initialization flag
    bool initialization_ok = true;

    if(req.planner_type == "bi")
    {
        //Reset planner data
        m_bi_planner->reset_planner_only();
        //Init planner for real robot
        initialization_ok = m_bi_planner->init_planner_map_goal_pose(goal_pose,constraint_vec_goal_pose, target_coordinate_dev_local, req.planner_type);
    }
    else if(req.planner_type == "uni")
    {
        //Reset planner data
        m_uni_planner->reset_planner_only();
        //Init planner for real robot
        initialization_ok = m_uni_planner->init_planner_map_goal_pose(goal_pose,constraint_vec_goal_pose, target_coordinate_dev_local, req.planner_type);
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi'' or 'uni')");
        //Return failure
        res.success = false;
        return false;
    }

    if(initialization_ok)
    {
        //Execute Planner run
        res.success = solve_planning_problem(req.planner_type, req.flag_iter_or_time, req.iter_or_time, req.tree_optimization, req.informed_sampling,req.run_id, req.show_tree);
    }
    else
    {
        ROS_ERROR("Planner initialization failed");
        //Return failure
        res.success = false;
        return false;
    }


    //Check planning result
    if(res.success == false)
        return false;
    else
    {
        //Joint and End-effector Trajectory
        vector<vector<double> > jt;
        vector<vector<double> > eet;

        //If planning succeeded -> Store robot trajectories in service response
        if(req.planner_type == "bi")
        {
            //Get Trajectories
            jt =  m_bi_planner->getJointTrajectory();
            eet =  m_bi_planner->getEndeffectorTrajectory();
        }
        if(req.planner_type == "uni")
        {
            //Get Trajectories
            jt =  m_uni_planner->getJointTrajectory();
            eet =  m_uni_planner->getEndeffectorTrajectory();
        }

        //Set size for response container
        res.joint_trajectory.array2d_double.resize(jt.size());
        res.ee_trajectory.array2d_double.resize(eet.size());

        //Convert trajectories to response type
        for(int i = 0 ; i < jt.size() ; i++)
        {
            res.joint_trajectory.array2d_double[i].array1d_double = jt[i];
            res.ee_trajectory.array2d_double[i].array1d_double = eet[i];
        }

        return true;
    }
}


bool PlanningControlCenter::run_planner_map_goal_config(planner_msgs::run_planner_map_goal_config::Request& req,
                           planner_msgs::run_planner_map_goal_config::Response& res)
{

    //Initialization flag
    bool initialization_ok = true;

    if(req.planner_type == "bi")
    {
        //Reset planner data
        m_bi_planner->reset_planner_only();
        //Init planner for real robot
        initialization_ok = m_bi_planner->init_planner_map_goal_config(req.goal_config, req.planner_type);
    }
    else if(req.planner_type == "uni")
    {
        //Reset planner data
        m_uni_planner->reset_planner_only();
        //Init planner for real robot
        initialization_ok = m_uni_planner->init_planner_map_goal_config(req.goal_config, req.planner_type);
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi'' or 'uni')");
        //Return failure
        res.success = false;
        return false;
    }

    if(initialization_ok)
    {
        //Execute Planner run
        res.success = solve_planning_problem(req.planner_type, req.flag_iter_or_time, req.iter_or_time, req.tree_optimization, req.informed_sampling,req.run_id, req.show_tree);
    }
    else
    {
        ROS_ERROR("Planner initialization failed");
        //Return failure
        res.success = false;
        return false;
    }


    //Check planning result
    if(res.success == false)
        return false;
    else
    {
        //Joint and End-effector Trajectory
        vector<vector<double> > jt;
        vector<vector<double> > eet;

        //If planning succeeded -> Store robot trajectories in service response
        if(req.planner_type == "bi")
        {
            //Get Trajectories
            jt =  m_bi_planner->getJointTrajectory();
            eet =  m_bi_planner->getEndeffectorTrajectory();
        }
        if(req.planner_type == "uni")
        {
            //Get Trajectories
            jt =  m_uni_planner->getJointTrajectory();
            eet =  m_uni_planner->getEndeffectorTrajectory();
        }

        //Set size for response container
        res.joint_trajectory.array2d_double.resize(jt.size());
        res.ee_trajectory.array2d_double.resize(eet.size());

        //Convert trajectories to response type
        for(int i = 0 ; i < jt.size() ; i++)
        {
            res.joint_trajectory.array2d_double[i].array1d_double = jt[i];
            res.ee_trajectory.array2d_double[i].array1d_double = eet[i];
        }

        return true;
    }

}



//--------------------------------------- Helper Functions for Callbacks ------------------------------------------------------

//Run planning routine
bool PlanningControlCenter::solve_planning_problem(string planner_type, bool flag_iter_or_time, double max_iter_time, bool tree_optimization, bool informed_sampling, int planner_run_number, bool show_tree_vis)
{
    //Init planning result
    bool success = false;

    if(planner_type == "bi")
    {
        //Set tree optimization and informed sampling flags
        if(tree_optimization == false)
            m_bi_planner->deactivateTreeOptimization();
        else
            m_bi_planner->activateTreeOptimization();

        if(informed_sampling == false)
            m_bi_planner->deactivateInformedSampling();
        else
            m_bi_planner->activateInformedSampling();

        //Run planner
        success = m_bi_planner->run_planner(1,flag_iter_or_time,max_iter_time,show_tree_vis,0.0,planner_run_number);
    }
    else if(planner_type == "uni")
    {
        //Set tree optimization and informed sampling flags
        if(tree_optimization == false)
            m_uni_planner->deactivateTreeOptimization();
        else
            m_uni_planner->activateTreeOptimization();

        if(informed_sampling == false)
            m_uni_planner->deactivateInformedSampling();
        else
            m_uni_planner->activateInformedSampling();

        //Run planner
        success = m_uni_planner->run_planner(1,flag_iter_or_time,max_iter_time,show_tree_vis,0.0,planner_run_number);
    }
    else
    {
        ROS_ERROR("Planner type not known! (Available options: 'bi'' or 'uni')");
        success = false;
    }


    return success;

}


//Trajectory Execution Service Callbacks
//bool PlanningControlCenter::execute_motion_plan(planner_msgs::execute_motion_plan::Request& req,
//                          planner_msgs::execute_motion_plan::Response& res)
//{

//    //Joint and End-effector Trajectory
//    vector<vector<double> > jt(req.joint_trajectory.array2d_double.size());
//    vector<vector<double> > eet(req.ee_trajectory.array2d_double.size());

//    for(int c = 0; c < req.joint_trajectory.array2d_double.size(); c++)
//    {
//        jt[c] = req.joint_trajectory.array2d_double[c].array1d_double;    //c-th configuration of trajectory
//        eet[c] = req.ee_trajectory.array2d_double[c].array1d_double;    //c-th ee pose of trajectory
//    }

//    //Execute the Trajectory in RViz
//    m_trajectory_execution->executeJointTrajectory(jt,eet);

//    res.execution_success = true;

//    return res.execution_success;
//}



} /* namespace planning_server */


