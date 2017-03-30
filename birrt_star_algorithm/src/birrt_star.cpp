#include <birrt_star_algorithm/birrt_star.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

namespace birrt_star_motion_planning
{


//Constructor
BiRRTstarPlanner::BiRRTstarPlanner(string planning_group)
{
    //Get namespaces of robot
    m_nh.param("ns_prefix_robot", m_ns_prefix_robot, std::string(""));

    //Get name of robot_description parameter
    string robot_description_robot;
    m_nh.param("robot_description_robot", robot_description_robot, std::string("robot_description"));

    //Get package path of "planning_scenarios"
    m_terminal_configs_path = ros::package::getPath("planning_scenarios");

    //Get package path of "planner_statistics"
    m_planner_package_path = ros::package::getPath("planner_statistics");

    //Set planning group (needs to correspond to one of the groups defined in the robot srdf)
    m_planning_group = planning_group;


    //Create Robot model
    //m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel("robot_description", "planning_scene", "endeffector_trajectory", m_planning_group));
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_description_robot, m_ns_prefix_robot + "planning_scene", m_ns_prefix_robot + "endeffector_trajectory", m_planning_group));

    ROS_INFO("KDL Robot Model initialized .....");

    //Get Planning Environment Size
    //m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder("robot_description", planning_group));
    //m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(robot_description_robot, planning_group,m_ns_prefix_robot));
    m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(m_KDLRobotModel, m_ns_prefix_robot));

    ROS_INFO("Planning World initialized .....");

    //Create FeasibilityChecker
    //m_FeasibilityChecker = boost::shared_ptr<state_feasibility_checker::FeasibilityChecker>(new state_feasibility_checker::FeasibilityChecker("robot_description", m_planning_group));
    //m_FeasibilityChecker = boost::shared_ptr<state_feasibility_checker::FeasibilityChecker>(new state_feasibility_checker::FeasibilityChecker(robot_description_robot, m_planning_group, m_ns_prefix_robot));
    m_FeasibilityChecker = boost::shared_ptr<state_feasibility_checker::FeasibilityChecker>(new state_feasibility_checker::FeasibilityChecker(m_KDLRobotModel, robot_description_robot, m_planning_group, m_ns_prefix_robot));

    ROS_INFO("Feasibility Checker initialized .....");

    //Planning Scene Monitor (required for collision checks)
    //m_planning_scene_monitor =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    //m_planning_scene_monitor =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description_robot);

    //Create Robot Controller
    //m_RobotMotionController = boost::shared_ptr<kuka_motion_controller::RobotController>(new kuka_motion_controller::RobotController("robot_description", m_planning_group));
    //m_RobotMotionController = boost::shared_ptr<kuka_motion_controller::RobotController>(new kuka_motion_controller::RobotController(robot_description_robot, m_planning_group, m_ns_prefix_robot));
    m_RobotMotionController = boost::shared_ptr<kuka_motion_controller::RobotController>(new kuka_motion_controller::RobotController(m_KDLRobotModel, robot_description_robot, m_planning_group, m_ns_prefix_robot));

    ROS_INFO("Robot Controller initialized .....");


    // ----- Get Planning Frame from srdf (if "map" planning for real robot / "base_link" planning in simulation) -----------

    //Check the planning frame (from virtual joint in srdf)
    m_planning_frame = getPlanningFrameFromSRDF(robot_description_robot);


    // ----- Init Controller Data  -----------

    //Set Motion Strategy (0 = all joints weighted equal)
    m_RobotMotionController->set_motion_strategy(0);


    // ----- Init Planning World Data  -----------

    //Set default environment size
    m_env_size_x.resize(2);
    m_env_size_x[0] = 0.0; //env dim in negative x dir
    m_env_size_x[1] = 0.0; //env dim in positive x dir
    m_env_size_y.resize(2);
    m_env_size_y[0] = 0.0; //env dim in negative y dir
    m_env_size_y[1] = 0.0; //env dim in positive y dir

    //Set default scene name
    m_planning_world->setSceneName("motion_plan");


    // ----- Init Kinematic Model Data  -----------

    //Get Kinematic Chain of manipulator
    m_manipulator_chain = m_KDLRobotModel->getCompleteArmChain();

    //Get Name of Base Link (required to set reference frame for visualization)
    m_base_link_name = m_KDLRobotModel->getBaseLinkName();

    //Consider robot prefix
    m_base_link_name = m_ns_prefix_robot + m_base_link_name;

    //Get the joint names
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Number of joints
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();


    // ----- Init Transform map to base_link (used for planning with real robot)  -----------

    //Reset flag indicating whether map to base_link transform is available
    m_transform_map_to_base_available = false;


    // ----- Init Planner Data  -----------

    //Weights for individual edge cost components (to punish motions of some variables stronger/less than the one of others)
    m_edge_cost_weights.resize(m_num_joints);

    //Initialize edge cost weights
    for(int j = 0 ; j < m_num_joints ; j++)
        m_edge_cost_weights[j] = 1.0;


    //Set start tree name
    m_start_tree.name = "START";

    //Init total number if nodes and edges contained in the start tree
    m_start_tree.num_nodes = 0;
    m_start_tree.num_edges = 0;
    m_start_tree.num_rewire_operations = 0;

    //Set goal tree name
    m_goal_tree.name = "GOAL";

    //Init total number if nodes and edges contained in the start tree
    m_goal_tree.num_nodes = 0;
    m_goal_tree.num_edges = 0;
    m_goal_tree.num_rewire_operations = 0;

    //Init Cost of current solution path
    m_cost_best_solution_path = 10000.0;
    m_cost_best_solution_path_revolute = 10000.0;
    m_cost_best_solution_path_prismatic = 10000.0;


    //Theoretical Minimum Cost for Solution Path (linear interpolation betweeen start and goal pose/config)
    m_cost_theoretical_solution_path.resize(3);
    m_cost_theoretical_solution_path[0] = 0.0;
    m_cost_theoretical_solution_path[1] = 0.0;
    m_cost_theoretical_solution_path[2] = 0.0;

    //Init Maximal Planner iterations and actually executed planner iterations
    m_max_planner_iter = 0;
    m_executed_planner_iter = 0;

    //Init Maximal Planner run time and actually executed planner run time
    m_max_planner_time = 0.0;
    m_executed_planner_time = 0.0;

    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Time when planning started and ended
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;

    //Set solution path available flag
    m_solution_path_available = false;

    //Tree Optimization Flag
    // -> If "activated" near nodes are considered in the nearest neighbour search
    // -> Nodes are rewired when a new node is added
    // -> If "deactivated" the planner reduces to the standard RRT-CONNECT
    m_tree_optimization_active = true;

    //Informed Sampling Heuristic Flag
    // -> If "activated" , samples are drawn from an informed subset once a solution is found
    // -> If "deactivated" , samples are drawn uniformly from the C-Space
    m_informed_sampling_active = true;

    //Init Bidirectional planner type
    m_planner_type = "bi_informed_rrt_star";

    // ----- General Planning Settings -----------

    //Define nearness of nodes (max. distance between nodes)
    m_nh.param("near_threshold_control", m_near_threshold_control, 0.5); //0.25m = 25cm

    //Distance threshold for C-Space samples(Defines "nearness" of ndoes in "find_near_vertices"-function)
    m_nh.param("near_threshold_interpolation", m_near_threshold_interpolation, 4.0); //3.0 rad (norm of c-space distance vector)

    //Number of points for C-Space interpolation
    m_nh.param("num_traj_segments_interp", m_num_traj_segments_interp, 20);

    //Step width from nearest neighbour in the direction of random sample
    m_nh.param("unconstraint_extend_step_factor", m_unconstraint_extend_step_factor, 0.5);

    //Maximum number of near nodes considered in "choose_node_parent_iterpolation" and "rewireTreeInterpolation"
    // -> "choose_node_parent_iterpolation" : Only first "m_max_near_nodes" nodes are considered
    // -> "rewireTreeInterpolation" : Only last "m_max_near_nodes" nodes are considered
    // Note: Near Nodes are stored in ascending order of cost-to-reach
    m_nh.param("max_near_nodes", m_max_near_nodes, 20);

    //Maximum available planning time
    m_nh.param("max_planning_time", m_max_planning_time, 30.0);

    //Treshold on path optimality, i.e. distance to theoretical solution path cost
    m_nh.param("path_optimality_treshold", m_path_optimality_treshold, 1.0);

    //Decides whether single or multiple steps are performed towards a random config
    m_nh.param("single_extend_step", m_single_extend_step, true);



    //------ Joint config sampling from Ellipse Initialization ------

    //Center of the unit ball
    m_ball_center_revolute.resize(m_num_joints_revolute);
    m_ball_center_prismatic.resize(m_num_joints_prismatic);

    //Rotation Matrix "C"
    m_rotation_C_revolute.resize(m_num_joints_revolute,m_num_joints_revolute);
    m_rotation_C_prismatic.resize(m_num_joints_prismatic,m_num_joints_prismatic);



    // ----- Constraint Planning Settings -----------

    //Flag indicating whether constraint motion planning is active
    m_constraint_active = false;

    //Set task frame reference
    m_task_pos_global = true;
    m_task_orient_global = true;

    //Constraint Selection Vector + Permitted Deviation Initialization
    m_constraint_vector.resize(6);
    m_coordinate_dev.resize(6);
    for(int i = 0 ; i < m_constraint_vector.size() ; i++)
    {
        m_constraint_vector[i] = 0;
        m_coordinate_dev[i].first = 0.0;
        m_coordinate_dev[i].second = 0.0;
    }



    //Step width factor in the direction of the unit vector pointing from nearest neighbour towards random sample
    m_nh.param("constraint_extend_step_factor", m_constraint_extend_step_factor, 0.4);

    //Minimum distance between near sample and projected sample
    // -> to avoid random samples being projected back onto near sample
    m_nh.param("min_projection_distance", m_min_projection_distance, 0.02);

    //Permitted sample projection error
    //m_nh.param("projection_error_threshold", m_projection_error_threshold, 0.1);

    //Maximum permitted task error for intermediate ee poses between two given configurations
    //m_nh.param("max_task_error_interpolation", m_max_task_error_interpolation, 0.1);

    //Maximum iterations for projection operation
    m_nh.param("max_projection_iter", m_max_projection_iter, 700);



    // ----- Attached Object Settings -----------

    //Init ID of attached object to default
    m_attached_object.object.id = "none";


    // ----- Tree Visualization Settings -----------

    m_nh.param("node_marker_scale", m_node_marker_scale, 0.03);
    m_nh.param("edge_marker_scale", m_edge_marker_scale, 0.005);
    m_nh.param("terminal_nodes_marker_scale", m_terminal_nodes_marker_scale, 0.05);
    m_nh.param("solution_path_ee_marker_scale", m_solution_path_ee_marker_scale, 0.01);
    m_nh.param("solution_path_base_marker_scale", m_solution_path_base_marker_scale, 0.01);


    //Tree Edges Visualization Topic for START Tree
    m_start_tree_edge_pub = m_nh.advertise<visualization_msgs::MarkerArray>("tree_edges_marker", 100);

    //Set up properties for add nodes marker
    m_start_tree_add_nodes_marker.header.frame_id = m_base_link_name;
    m_start_tree_add_nodes_marker.ns = "sphere";
    m_start_tree_add_nodes_marker.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    m_start_tree_add_nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    //Nodes markers
    m_start_tree_add_nodes_marker.scale.x = m_node_marker_scale;
    m_start_tree_add_nodes_marker.scale.y = m_node_marker_scale;
    m_start_tree_add_nodes_marker.scale.z = m_node_marker_scale;
    m_start_tree_add_nodes_marker.pose.orientation.w = 1.0;
    //Set colour for start tree nodes
    m_start_tree_add_nodes_marker.color.r = 0.0;
    m_start_tree_add_nodes_marker.color.g = 0.0;
    m_start_tree_add_nodes_marker.color.b = 0.0;
    m_start_tree_add_nodes_marker.color.a = 1.0;

    //Tree Nodes Visualization Topic for START Tree
    m_start_tree_node_pub = m_nh.advertise<visualization_msgs::Marker>("tree_nodes_marker", 100);


    //Tree Edges Visualization Topic for GOAL Tree
    m_goal_tree_edge_pub = m_nh.advertise<visualization_msgs::MarkerArray>("tree_edges_marker2", 100);

    //Set up properties for add nodes marker
    m_goal_tree_add_nodes_marker.header.frame_id = m_base_link_name;
    m_goal_tree_add_nodes_marker.ns = "sphere";
    m_goal_tree_add_nodes_marker.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    m_goal_tree_add_nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    //Nodes markers
    m_goal_tree_add_nodes_marker.scale.x = m_node_marker_scale;
    m_goal_tree_add_nodes_marker.scale.y = m_node_marker_scale;
    m_goal_tree_add_nodes_marker.scale.z = m_node_marker_scale;
    m_goal_tree_add_nodes_marker.pose.orientation.w = 1.0;
    //Set colour for goal tree nodes
    m_goal_tree_add_nodes_marker.color.r = 0.0;
    m_goal_tree_add_nodes_marker.color.g = 0.0;
    m_goal_tree_add_nodes_marker.color.b = 0.0;
    m_goal_tree_add_nodes_marker.color.a = 1.0;

    //Tree Nodes Visualization Topic for GOAL Tree
    m_goal_tree_node_pub = m_nh.advertise<visualization_msgs::Marker>("tree_nodes_marker2", 100);


    //Tree Terminal Nodes Visualization Topic
    m_tree_terminal_nodes_pub = m_nh.advertise<visualization_msgs::MarkerArray>("start_goal_node_marker", 100);

    //Solution path publisher (for ee trajectory)
    m_ee_solution_path_pub = m_nh.advertise<visualization_msgs::Marker>("ee_tree_solution_path", 100);

    //Solution path publisher (for base trajectory)
    m_base_solution_path_pub = m_nh.advertise<visualization_msgs::Marker>("base_tree_solution_path", 100);

    //Subscriber to get current joint state from real lbr arm
    m_lbr_joint_state_sub = m_nh.subscribe(m_ns_prefix_robot + "joint_states", 1000, &BiRRTstarPlanner::callback_lbr_joint_states, this);
    //Flag indicating that lbr joint state is available
    m_lbr_joint_state_received = false;
    //Current Lbr joint state
    m_lbr_joint_state.resize(7);

    //Set Result of Motion Planning to failure (= 0)
    m_planner_success = 0;


    //Set Publisher topic for planning progress (0 to 100%)
    m_pub_planning_progress = m_nh.advertise<std_msgs::Float32>("rrt_planner_progress", 10);

    ROS_INFO("Bi RRT* Planner initialized .....");

}



//Destructor
BiRRTstarPlanner::~BiRRTstarPlanner()
{
    //Nothing to do yet
}


//Initialize RRT* Planner (reading start and goal config from file)
bool BiRRTstarPlanner::init_planner(char *start_goal_config_file, int search_space)
{
    //Read Start and Goal Configuration from File
    vector<double> start_conf, goal_conf;
    bool read_ok = readStartGoalConfig(start_goal_config_file,start_conf,goal_conf);
    if(read_ok == false)
    {
        ROS_ERROR("File path to start and goal config is invalid!!!");
        ros::shutdown();
    }else{
        ROS_INFO("Start and goal config successfully read from file!!!");
    }

    //Check dimension of config
    if(start_conf.size() != m_num_joints || goal_conf.size() != m_num_joints)
    {
        ROS_ERROR("Dimension of configuration vector does not match the number of joints in the planning group!");
        return false;
    }

    //Convert start and goal configuration into KDL Joint Array
    KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(start_conf);
    KDL::JntArray goal_configuration = m_RobotMotionController->Vector_to_JntArray(goal_conf);

    //Initialize map to robot transform before performing collision checks
    if(m_planning_frame == "/map")
    {
        if(!m_FeasibilityChecker->update_map_to_robot_transform())
            ROS_INFO_STREAM("Failed to update map to robot transform in feasibility checker");
    }


    //Check start and goal config for validity
    bool start_conf_valid = m_FeasibilityChecker->isConfigValid(start_configuration);
    bool goal_conf_valid = m_FeasibilityChecker->isConfigValid(goal_configuration);
    if(start_conf_valid == false){
        ROS_ERROR("Start configuration is invalid!!!");
        return false;
    }else if (goal_conf_valid == false){
        ROS_ERROR("Goal configuration is invalid!!!");
        return false;
    }else{
        //Nothing to do
    }


    //Find endeffector pose for start and goal configuration
    vector<double> ee_start_pose = computeEEPose(start_configuration);
    vector<double> ee_goal_pose = computeEEPose(goal_configuration);



    //Set pose of task frame to start ee pos
//    if(m_constraint_active)
//    {
//        //Set Task frame to pose of endeffector frame at start config
//        m_task_frame = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);
//    }



    //Compute Heuristic for root node (either distance between start and goal config or between start and goal endeffector pose)
    //m_cost_theoretical_solution_path[0] -> total
    //m_cost_theoretical_solution_path[1] -> revolute distance norm
    //m_cost_theoretical_solution_path[2] -> prismatic distance norm
    if(search_space == 0) //Planning in Control Space
        m_cost_theoretical_solution_path[0] = m_Heuristic.euclidean_pose_distance(ee_start_pose, ee_goal_pose);
    else if (search_space == 1) //Planning in Joint Space
        m_cost_theoretical_solution_path = m_Heuristic.euclidean_joint_space_distance(m_manipulator_chain, start_conf, goal_conf);
    else
        ROS_ERROR("Requested planner search space does not exist!!!");


    //---Create Goal Node for RRT* Tree
    Node goal_node;
    goal_node.node_id = 0; //negative index to avoid using goal node as nearest neighbour (goal node only considered in REWIRE function)
    goal_node.parent_id = 0; //No parent yet
    //Find collision free configuration for goal node
    goal_node.config = goal_conf;
    goal_node.ee_pose = ee_goal_pose;
    goal_node.cost_reach.total = 0.0;
    goal_node.cost_reach.revolute = 0.0;
    goal_node.cost_reach.prismatic = 0.0;
    goal_node.cost_h.total = m_cost_theoretical_solution_path[0] ;// goal node has same heuristic cost as the start node
    goal_node.cost_h.revolute = m_cost_theoretical_solution_path[1];   //distance for revolute joints
    goal_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];  //distance for prismatic joints
    //Add goal Node to the tree
    m_goal_tree.nodes.push_back(goal_node);
    m_goal_tree.num_nodes++;


    //Add sphere to visualize goal node
    visualization_msgs::Marker goal_node_marker_; //terminal nodes
    goal_node_marker_.id = 0;
    goal_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    goal_node_marker_.header.frame_id = m_base_link_name;
    goal_node_marker_.ns = "terminal_nodes";
    goal_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    goal_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    goal_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    goal_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    goal_node_marker_.color.r = 0.0;
    goal_node_marker_.color.g = 1.0;
    goal_node_marker_.color.b = 0.0;
    goal_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    goal_node_marker_.pose.position.x = goal_node.ee_pose[0];
    goal_node_marker_.pose.position.y = goal_node.ee_pose[1];
    goal_node_marker_.pose.position.z = goal_node.ee_pose[2];
    //Add start node to marker
    m_terminal_nodes_marker_array_msg.markers.push_back(goal_node_marker_);



    //--- Create Start Node for RRT* Tree
    Node start_node;
    start_node.node_id = 0;
    start_node.parent_id = 0;
    start_node.config = start_conf;
    start_node.ee_pose = ee_start_pose;
    start_node.cost_reach.total = 0.0;
    start_node.cost_reach.revolute = 0.0;
    start_node.cost_reach.prismatic = 0.0;
    start_node.cost_h.total = m_cost_theoretical_solution_path[0];
    start_node.cost_h.revolute = m_cost_theoretical_solution_path[1];
    start_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];
    //Add start Node to the tree
    m_start_tree.nodes.push_back(start_node);
    m_start_tree.num_nodes++;

    //Add sphere to visualize goal node
    visualization_msgs::Marker start_node_marker_; //terminal nodes
    start_node_marker_.id = 1;
    start_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    start_node_marker_.header.frame_id = m_base_link_name;
    start_node_marker_.ns = "terminal_nodes";
    start_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    start_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    start_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    start_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    start_node_marker_.color.r = 1.0;
    start_node_marker_.color.g = 0.0;
    start_node_marker_.color.b = 0.0;
    start_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    start_node_marker_.pose.position.x = start_node.ee_pose[0];
    start_node_marker_.pose.position.y = start_node.ee_pose[1];
    start_node_marker_.pose.position.z = start_node.ee_pose[2];
    //Add start node to marke
    m_terminal_nodes_marker_array_msg.markers.push_back(start_node_marker_);


    //Publish start and goal node
    m_tree_terminal_nodes_pub.publish(m_terminal_nodes_marker_array_msg);


    //Save start and goal ee_pose
    m_ee_start_pose = ee_start_pose;
    m_ee_goal_pose = ee_goal_pose;

    //Save start and goal config
    m_config_start_pose = start_conf;
    m_config_goal_pose = goal_node.config;


    //Empty array of nodes
    m_start_tree_add_nodes_marker.points.empty();
    m_start_tree_node_pub.publish(m_start_tree_add_nodes_marker);
    m_goal_tree_add_nodes_marker.points.empty();
    m_goal_tree_node_pub.publish(m_goal_tree_add_nodes_marker);

    //Empty array of edges
    //m_start_tree_add_edge_marker_array_msg.markers.empty();
    m_start_tree_add_edge_marker_array_msg.markers.clear();
    m_start_tree_edge_pub.publish(m_start_tree_add_edge_marker_array_msg);
    m_goal_tree_add_edge_marker_array_msg.markers.clear();
    m_goal_tree_edge_pub.publish(m_goal_tree_add_edge_marker_array_msg);


    //Delete solution path
    visualization_msgs::Marker empty_solution_path_marker;
    empty_solution_path_marker.id = 1;
    empty_solution_path_marker.header.stamp = ros::Time::now();
    empty_solution_path_marker.header.frame_id = m_base_link_name;
    empty_solution_path_marker.ns = "solution";
    empty_solution_path_marker.action = visualization_msgs::Marker::DELETE;
    empty_solution_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    empty_solution_path_marker.scale.x = m_solution_path_ee_marker_scale;
    empty_solution_path_marker.pose.orientation.w = 1.0;
    empty_solution_path_marker.color.r = 1.0;
    empty_solution_path_marker.color.g = 1.0;
    empty_solution_path_marker.color.b = 1.0;
    empty_solution_path_marker.color.a = 1.0;
    m_ee_solution_path_pub.publish(empty_solution_path_marker);


    //Initialization of Variables required for Ellipse Sampling
    if(search_space == 0) //Planning in Control Space
    {
        //TODO Ellipse Sampling Initialization
    }
    else if (search_space == 1) //Planning in Joint Space
        jointConfigEllipseInitialization();
    else
        ROS_ERROR("Requested planner search space does not exist!!!");


    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Time when planning started and ended
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;

    //If planning is performed in the map frame
    if(m_planning_frame == "/map")
    {
        //Get current pose of robot in the map frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
            m_transform_map_to_base_available = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Transform /map to /base_link not available!");
            m_transform_map_to_base_available = false;
            return false;
        }
    }

    //Everything went fine
    return true;

}

//Initialize RRT* Planner (given start and config)
bool BiRRTstarPlanner::init_planner(vector<double> start_conf, vector<double> goal_conf, int search_space)
{
    //Check dimension of config
    if(start_conf.size() != m_num_joints || goal_conf.size() != m_num_joints)
    {
        ROS_ERROR("Dimension of configuration vector does not match the number of joints in the planning group!");
        return false;
    }

    //Convert start and goal configuration into KDL Joint Array
    KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(start_conf);
    KDL::JntArray goal_configuration = m_RobotMotionController->Vector_to_JntArray(goal_conf);

    //Initialize map to robot transform before performing collision checks
    if(m_planning_frame == "/map")
    {
        if(!m_FeasibilityChecker->update_map_to_robot_transform())
            ROS_INFO_STREAM("Failed to update map to robot transform in feasibility checker");
    }

    //Check start and goal config for validity
    bool start_conf_valid = m_FeasibilityChecker->isConfigValid(start_configuration);
    bool goal_conf_valid = m_FeasibilityChecker->isConfigValid(goal_configuration);
    if(start_conf_valid == false){
        ROS_ERROR("Start configuration is invalid!!!");
        return false;
    }else if (goal_conf_valid == false){
        ROS_ERROR("Goal configuration is invalid!!!");
        return false;
    }else{
        //Nothing to do
    }

    //Find endeffector pose for start and goal configuration
    vector<double> ee_start_pose = computeEEPose(start_configuration);
    vector<double> ee_goal_pose = computeEEPose(goal_configuration);


    //Set pose of task frame to start ee pos
//    if(m_constraint_active)
//    {
//        //Set Task frame to pose of endeffector frame at start config
//        m_task_frame = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);
//    }



    //Compute Heuristic for root node (either distance between start and goal config or between start and goal endeffector pose)
    //m_cost_theoretical_solution_path[0] -> total
    //m_cost_theoretical_solution_path[1] -> revolute distance norm
    //m_cost_theoretical_solution_path[2] -> prismatic distance norm
    if(search_space == 0) //Planning in Control Space
        m_cost_theoretical_solution_path[0] = m_Heuristic.euclidean_pose_distance(ee_start_pose, ee_goal_pose);
    else if (search_space == 1) //Planning in Joint Space
        m_cost_theoretical_solution_path = m_Heuristic.euclidean_joint_space_distance(m_manipulator_chain, start_conf, goal_conf);
    else
        ROS_ERROR("Requested planner search space does not exist!!!");




    //---Create Goal Node for RRT* Tree
    Node goal_node;
    goal_node.node_id = 0; //negative index to avoid using goal node as nearest neighbour (goal node only considered in REWIRE function)
    goal_node.parent_id = 0; //No parent yet
    //Find collision free configuration for goal node
    goal_node.config = goal_conf;
    goal_node.ee_pose = ee_goal_pose;
    goal_node.cost_reach.total = 0.0;
    goal_node.cost_reach.revolute = 0.0;
    goal_node.cost_reach.prismatic = 0.0;
    goal_node.cost_h.total = m_cost_theoretical_solution_path[0] ;// goal node has same heuristic cost as the start node
    goal_node.cost_h.revolute = m_cost_theoretical_solution_path[1];   //distance for revolute joints
    goal_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];  //distance for prismatic joints
    //Add goal Node to the tree
    m_goal_tree.nodes.push_back(goal_node);
    m_goal_tree.num_nodes++;


    //Add sphere to visualize goal node
    visualization_msgs::Marker goal_node_marker_; //terminal nodes
    goal_node_marker_.id = 0;
    goal_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    goal_node_marker_.header.frame_id = m_base_link_name;
    goal_node_marker_.ns = "terminal_nodes";
    goal_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    goal_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    goal_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    goal_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    goal_node_marker_.color.r = 0.0;
    goal_node_marker_.color.g = 1.0;
    goal_node_marker_.color.b = 0.0;
    goal_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    goal_node_marker_.pose.position.x = goal_node.ee_pose[0];
    goal_node_marker_.pose.position.y = goal_node.ee_pose[1];
    goal_node_marker_.pose.position.z = goal_node.ee_pose[2];
    //Add start node to marker
    m_terminal_nodes_marker_array_msg.markers.push_back(goal_node_marker_);



    //--- Create Start Node for RRT* Tree
    Node start_node;
    start_node.node_id = 0;
    start_node.parent_id = 0;
    start_node.config = start_conf;
    start_node.ee_pose = ee_start_pose;
    start_node.cost_reach.total = 0.0;
    start_node.cost_reach.revolute = 0.0;
    start_node.cost_reach.prismatic = 0.0;
    start_node.cost_h.total = m_cost_theoretical_solution_path[0];
    start_node.cost_h.revolute = m_cost_theoretical_solution_path[1];
    start_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];
    //Add start Node to the tree
    m_start_tree.nodes.push_back(start_node);
    m_start_tree.num_nodes++;

    //Add sphere to visualize goal node
    visualization_msgs::Marker start_node_marker_; //terminal nodes
    start_node_marker_.id = 1;
    start_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    start_node_marker_.header.frame_id = m_base_link_name;
    start_node_marker_.ns = "terminal_nodes";
    start_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    start_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    start_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    start_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    start_node_marker_.color.r = 1.0;
    start_node_marker_.color.g = 0.0;
    start_node_marker_.color.b = 0.0;
    start_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    start_node_marker_.pose.position.x = start_node.ee_pose[0];
    start_node_marker_.pose.position.y = start_node.ee_pose[1];
    start_node_marker_.pose.position.z = start_node.ee_pose[2];
    //Add start node to marke
    m_terminal_nodes_marker_array_msg.markers.push_back(start_node_marker_);


    //Publish start and goal node
    m_tree_terminal_nodes_pub.publish(m_terminal_nodes_marker_array_msg);


    //Save start and goal ee_pose
    m_ee_start_pose = ee_start_pose;
    m_ee_goal_pose = ee_goal_pose;

    //Save start and goal config
    m_config_start_pose = start_conf;
    m_config_goal_pose = goal_node.config;


    //Empty array of nodes
    m_start_tree_add_nodes_marker.points.empty();
    m_start_tree_node_pub.publish(m_start_tree_add_nodes_marker);
    m_goal_tree_add_nodes_marker.points.empty();
    m_goal_tree_node_pub.publish(m_goal_tree_add_nodes_marker);

    //Empty array of edges
    //m_start_tree_add_edge_marker_array_msg.markers.empty();
    m_start_tree_add_edge_marker_array_msg.markers.clear();
    m_start_tree_edge_pub.publish(m_start_tree_add_edge_marker_array_msg);
    m_goal_tree_add_edge_marker_array_msg.markers.clear();
    m_goal_tree_edge_pub.publish(m_goal_tree_add_edge_marker_array_msg);


    //Delete solution path
    visualization_msgs::Marker empty_solution_path_marker;
    empty_solution_path_marker.id = 1;
    empty_solution_path_marker.header.stamp = ros::Time::now();
    empty_solution_path_marker.header.frame_id = m_base_link_name;
    empty_solution_path_marker.ns = "solution";
    empty_solution_path_marker.action = visualization_msgs::Marker::DELETE;
    empty_solution_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    empty_solution_path_marker.scale.x = m_solution_path_ee_marker_scale;
    empty_solution_path_marker.pose.orientation.w = 1.0;
    empty_solution_path_marker.color.r = 1.0;
    empty_solution_path_marker.color.g = 1.0;
    empty_solution_path_marker.color.b = 1.0;
    empty_solution_path_marker.color.a = 1.0;
    m_ee_solution_path_pub.publish(empty_solution_path_marker);


    //Initialization of Variables required for Ellipse Sampling
    if(search_space == 0) //Planning in Control Space
    {
        //TODO Ellipse Sampling Initialization
    }
    else if (search_space == 1) //Planning in Joint Space
        jointConfigEllipseInitialization();
    else
        ROS_ERROR("Requested planner search space does not exist!!!");



    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Time when planning started and ended
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;

    //If planning is performed in the map frame
    if(m_planning_frame == "/map")
    {
        //Get current pose of robot in the map frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
            m_transform_map_to_base_available = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Transform /map to /base_link not available!");
            m_transform_map_to_base_available = false;
            return false;
        }

    }

    //Everything went fine
    return true;

}


//Initialize RRT* Planner (given start config and final endeffector pose)
bool BiRRTstarPlanner::init_planner(vector<double> start_conf, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space)
{

    //Check dimension of config
    if(start_conf.size() != m_num_joints)
    {
        ROS_ERROR("Dimension of configuration vector does not match the number of joints in the planning group!");
        return false;
    }

    //Note: ee_goal_pose represents the end-effector goal pose with respect to the "base_link"

    //Convert XYZ euler orientation of goal pose to quaternion
    vector<double> quat_goal_pose = m_RobotMotionController->convertEulertoQuat(ee_goal_pose[3],ee_goal_pose[4],ee_goal_pose[5]);

    //Set up goal pose with orientation expressed by quaternion
    vector<double> goal_ee_pose_quat_orient (7);
    goal_ee_pose_quat_orient[0] = ee_goal_pose[0]; //x
    goal_ee_pose_quat_orient[1] = ee_goal_pose[1]; //y
    goal_ee_pose_quat_orient[2] = ee_goal_pose[2]; //z
    goal_ee_pose_quat_orient[3] = quat_goal_pose[0];  //quat_x
    goal_ee_pose_quat_orient[4] = quat_goal_pose[1];  //quat_y
    goal_ee_pose_quat_orient[5] = quat_goal_pose[2];  //quat_z
    goal_ee_pose_quat_orient[6] = quat_goal_pose[3];  //quat_w


    //Find configuration for endeffector goal pose (using start_config as mean config for init_config sampling)
    vector<double> ik_sol_ee_goal_pose = findIKSolution(goal_ee_pose_quat_orient, constraint_vec_goal_pose, coordinate_dev, start_conf,false);


    //Convert start configuration into KDL Joint Array
    KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(start_conf);

    //Find endeffector pose for start configuration
    vector<double> ee_start_pose = computeEEPose(start_configuration);

    //Initialize map to robot transform before performing collision checks
    if(m_planning_frame == "/map")
    {
        if(!m_FeasibilityChecker->update_map_to_robot_transform())
            ROS_INFO_STREAM("Failed to update map to robot transform in feasibility checker");
    }

    //Check start and goal config for validity
    bool start_conf_valid = m_FeasibilityChecker->isConfigValid(start_configuration);
    bool goal_conf_valid = m_FeasibilityChecker->isConfigValid(ik_sol_ee_goal_pose);
    if(start_conf_valid == false){
        ROS_ERROR("Start configuration is invalid!!!");
        return false;
    }else if (goal_conf_valid == false){
        ROS_ERROR("Goal configuration is invalid!!!");
        return false;
    }else{
        //Nothing to do
    }


    //Store orientation of start ee pose as orientation of task frame (i.e. task frame orientation set globally)
//    if(m_constraint_active)
//    {
//        //Set Task frame to pose of endeffector frame at start config
//        m_task_frame = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);
//    }



    //Compute Heuristic for root node (either distance between start and goal config or between start and goal endeffector pose)
    //m_cost_theoretical_solution_path[0] -> total
    //m_cost_theoretical_solution_path[1] -> revolute distance norm
    //m_cost_theoretical_solution_path[2] -> prismatic distance norm
    if(search_space == 0) //Planning in Control Space
        m_cost_theoretical_solution_path[0] = m_Heuristic.euclidean_pose_distance(ee_start_pose, ee_goal_pose);
    else if (search_space == 1) //Planning in Joint Space
        m_cost_theoretical_solution_path = m_Heuristic.euclidean_joint_space_distance(m_manipulator_chain, start_conf, ik_sol_ee_goal_pose);
    else
        ROS_ERROR("Requested planner search space does not exist!!!");




    //---Create Goal Node for RRT* Tree
    Node goal_node;
    goal_node.node_id = 0; //negative index to avoid using goal node as nearest neighbour (goal node only considered in REWIRE function)
    goal_node.parent_id = 0; //No parent yet
    //Find collision free configuration for goal node
    goal_node.config = ik_sol_ee_goal_pose;
    goal_node.ee_pose = ee_goal_pose;
    goal_node.cost_reach.total = 0.0;
    goal_node.cost_reach.revolute = 0.0;
    goal_node.cost_reach.prismatic = 0.0;
    goal_node.cost_h.total = m_cost_theoretical_solution_path[0] ;// goal node has same heuristic cost as the start node
    goal_node.cost_h.revolute = m_cost_theoretical_solution_path[1];   //distance for revolute joints
    goal_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];  //distance for prismatic joints
    //Add goal Node to the tree
    m_goal_tree.nodes.push_back(goal_node);
    m_goal_tree.num_nodes++;


    //Add sphere to visualize goal node
    visualization_msgs::Marker goal_node_marker_; //terminal nodes
    goal_node_marker_.id = 0;
    goal_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    goal_node_marker_.header.frame_id = m_base_link_name;
    goal_node_marker_.ns = "terminal_nodes";
    goal_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    goal_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    goal_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    goal_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    goal_node_marker_.color.r = 0.0;
    goal_node_marker_.color.g = 1.0;
    goal_node_marker_.color.b = 0.0;
    goal_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    goal_node_marker_.pose.position.x = goal_node.ee_pose[0];
    goal_node_marker_.pose.position.y = goal_node.ee_pose[1];
    goal_node_marker_.pose.position.z = goal_node.ee_pose[2];
    //Add start node to marker
    m_terminal_nodes_marker_array_msg.markers.push_back(goal_node_marker_);



    //--- Create Start Node for RRT* Tree
    Node start_node;
    start_node.node_id = 0;
    start_node.parent_id = 0;
    start_node.config = start_conf;
    start_node.ee_pose = ee_start_pose;
    start_node.cost_reach.total = 0.0;
    start_node.cost_reach.revolute = 0.0;
    start_node.cost_reach.prismatic = 0.0;
    start_node.cost_h.total = m_cost_theoretical_solution_path[0];
    start_node.cost_h.revolute = m_cost_theoretical_solution_path[1];
    start_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];
    //Add start Node to the tree
    m_start_tree.nodes.push_back(start_node);
    m_start_tree.num_nodes++;

    //Add sphere to visualize goal node
    visualization_msgs::Marker start_node_marker_; //terminal nodes
    start_node_marker_.id = 1;
    start_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    start_node_marker_.header.frame_id = m_base_link_name;
    start_node_marker_.ns = "terminal_nodes";
    start_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    start_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    start_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    start_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    start_node_marker_.color.r = 1.0;
    start_node_marker_.color.g = 0.0;
    start_node_marker_.color.b = 0.0;
    start_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    start_node_marker_.pose.position.x = start_node.ee_pose[0];
    start_node_marker_.pose.position.y = start_node.ee_pose[1];
    start_node_marker_.pose.position.z = start_node.ee_pose[2];
    //Add start node to marke
    m_terminal_nodes_marker_array_msg.markers.push_back(start_node_marker_);


    //Publish start and goal node
    m_tree_terminal_nodes_pub.publish(m_terminal_nodes_marker_array_msg);


    //Save start and goal ee_pose
    m_ee_start_pose = ee_start_pose;
    m_ee_goal_pose = ee_goal_pose;

    //Save start and goal config
    m_config_start_pose = start_conf;
    m_config_goal_pose = goal_node.config;


    //Empty array of nodes
    m_start_tree_add_nodes_marker.points.empty();
    m_start_tree_node_pub.publish(m_start_tree_add_nodes_marker);
    m_goal_tree_add_nodes_marker.points.empty();
    m_goal_tree_node_pub.publish(m_goal_tree_add_nodes_marker);

    //Empty array of edges
    //m_start_tree_add_edge_marker_array_msg.markers.empty();
    m_start_tree_add_edge_marker_array_msg.markers.clear();
    m_start_tree_edge_pub.publish(m_start_tree_add_edge_marker_array_msg);
    m_goal_tree_add_edge_marker_array_msg.markers.clear();
    m_goal_tree_edge_pub.publish(m_goal_tree_add_edge_marker_array_msg);


    //Delete solution path
    visualization_msgs::Marker empty_solution_path_marker;
    empty_solution_path_marker.id = 1;
    empty_solution_path_marker.header.stamp = ros::Time::now();
    empty_solution_path_marker.header.frame_id = m_base_link_name;
    empty_solution_path_marker.ns = "solution";
    empty_solution_path_marker.action = visualization_msgs::Marker::DELETE;
    empty_solution_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    empty_solution_path_marker.scale.x = m_solution_path_ee_marker_scale;
    empty_solution_path_marker.pose.orientation.w = 1.0;
    empty_solution_path_marker.color.r = 1.0;
    empty_solution_path_marker.color.g = 1.0;
    empty_solution_path_marker.color.b = 1.0;
    empty_solution_path_marker.color.a = 1.0;
    m_ee_solution_path_pub.publish(empty_solution_path_marker);


    //Initialization of Variables required for Ellipse Sampling
    if(search_space == 0) //Planning in Control Space
    {
        //TODO Ellipse Sampling Initialization
    }
    else if (search_space == 1) //Planning in Joint Space
        jointConfigEllipseInitialization();
    else
        ROS_ERROR("Requested planner search space does not exist!!!");


    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Time when planning started and ended
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;

    //If planning is performed in the map frame
    if(m_planning_frame == "/map")
    {
        //Get current pose of robot in the map frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
            m_transform_map_to_base_available = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Transform /map to /base_link not available!");
            m_transform_map_to_base_available = false;
            return false;
        }
    }

    //Everything went fine
    return true;

}


//Initialize RRT* Planner (given start and final endeffector pose)
bool BiRRTstarPlanner::init_planner(vector<double> ee_start_pose, vector<int> constraint_vec_start_pose, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space)
{

    //Note: ee_start_pose and ee_goal_pose represents the end-effector pose with respect to the "base_link"

    //Generate start and goal config
    //vector<vector<double> > start_goal_config = generate_start_goal_config(ee_start_pose,constraint_vec_start_pose, ee_goal_pose, constraint_vec_goal_pose, coordinate_dev, true);

    //Store start and goal config
    //vector<double> ik_sol_ee_start_pose = start_goal_config[0];
    //vector<double> ik_sol_ee_goal_pose = start_goal_config[1];

    //Convert XYZ euler orientation of start pose to quaternion
    vector<double> quat_start_pose = m_RobotMotionController->convertEulertoQuat(ee_start_pose[3],ee_start_pose[4],ee_start_pose[5]);

    //Convert XYZ euler orientation of goal pose to quaternion
    vector<double> quat_goal_pose = m_RobotMotionController->convertEulertoQuat(ee_goal_pose[3],ee_goal_pose[4],ee_goal_pose[5]);

    //Set up poses with orientation expressed by quaternion
    vector<double> start_ee_pose_quat_orient (7);
    start_ee_pose_quat_orient[0] = ee_start_pose[0]; //x
    start_ee_pose_quat_orient[1] = ee_start_pose[1]; //y
    start_ee_pose_quat_orient[2] = ee_start_pose[2]; //z
    start_ee_pose_quat_orient[3] = quat_start_pose[0];  //quat_x
    start_ee_pose_quat_orient[4] = quat_start_pose[1];  //quat_y
    start_ee_pose_quat_orient[5] = quat_start_pose[2];  //quat_z
    start_ee_pose_quat_orient[6] = quat_start_pose[3];  //quat_w
    vector<double> goal_ee_pose_quat_orient (7);
    goal_ee_pose_quat_orient[0] = ee_goal_pose[0]; //x
    goal_ee_pose_quat_orient[1] = ee_goal_pose[1]; //y
    goal_ee_pose_quat_orient[2] = ee_goal_pose[2]; //z
    goal_ee_pose_quat_orient[3] = quat_goal_pose[0];  //quat_x
    goal_ee_pose_quat_orient[4] = quat_goal_pose[1];  //quat_y
    goal_ee_pose_quat_orient[5] = quat_goal_pose[2];  //quat_z
    goal_ee_pose_quat_orient[6] = quat_goal_pose[3];  //quat_w


    //Find configuration for endeffector goal pose (using ik_sol_ee_start_pose as mean config for init_config sampling)
    //Note: -> Gaussian sampling for initial config is only performed for revolute joints
    //      -> Initial values for prismatic joint are sampled uniformly from their respective joint range)
    vector<double> ik_sol_ee_goal_pose = findIKSolution(goal_ee_pose_quat_orient, constraint_vec_goal_pose, coordinate_dev, true);

    //Find configuration for endeffector start pose
    vector<double> ik_sol_ee_start_pose = findIKSolution(start_ee_pose_quat_orient, constraint_vec_start_pose, coordinate_dev, ik_sol_ee_goal_pose, true);

    //Initialize map to robot transform before performing collision checks
    if(m_planning_frame == "/map")
    {
        if(!m_FeasibilityChecker->update_map_to_robot_transform())
            ROS_INFO_STREAM("Failed to update map to robot transform in feasibility checker");
    }

    //Check start and goal config for validity
    bool start_conf_valid = m_FeasibilityChecker->isConfigValid(ik_sol_ee_start_pose);
    bool goal_conf_valid = m_FeasibilityChecker->isConfigValid(ik_sol_ee_goal_pose);
    if(start_conf_valid == false){
        ROS_ERROR("Start configuration is invalid!!!");
        return false;
    }else if (goal_conf_valid == false){
        ROS_ERROR("Goal configuration is invalid!!!");
        return false;
    }else{
        //Nothing to do
    }

    //Compute Heuristic for root node (either distance between start and goal config or between start and goal endeffector pose)
    //m_cost_theoretical_solution_path[0] -> total
    //m_cost_theoretical_solution_path[1] -> revolute distance norm
    //m_cost_theoretical_solution_path[2] -> prismatic distance norm
    if(search_space == 0) //Planning in Control Space
        m_cost_theoretical_solution_path[0] = m_Heuristic.euclidean_pose_distance(start_ee_pose_quat_orient, goal_ee_pose_quat_orient);
    else if (search_space == 1) //Planning in Joint Space
        m_cost_theoretical_solution_path = m_Heuristic.euclidean_joint_space_distance(m_manipulator_chain, ik_sol_ee_start_pose, ik_sol_ee_goal_pose);
    else
        ROS_ERROR("Requested planner search space does not exist!!!");


    //---Create Goal Node for RRT* Tree
    Node goal_node;
    goal_node.node_id = 0; //negative index to avoid using goal node as nearest neighbour (goal node only considered in REWIRE function)
    goal_node.parent_id = 0; //No parent yet
    //Find collision free configuration for goal node
    goal_node.config = ik_sol_ee_goal_pose;
    goal_node.ee_pose = goal_ee_pose_quat_orient;
    goal_node.cost_reach.total = 0.0;
    goal_node.cost_reach.revolute = 0.0;
    goal_node.cost_reach.prismatic = 0.0;
    goal_node.cost_h.total = m_cost_theoretical_solution_path[0] ;// goal node has same heuristic cost as the start node
    goal_node.cost_h.revolute = m_cost_theoretical_solution_path[1];   //distance for revolute joints
    goal_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];  //distance for prismatic joints
    //Add goal Node to the tree
    m_goal_tree.nodes.push_back(goal_node);
    m_goal_tree.num_nodes++;


    //Add sphere to visualize goal node
    visualization_msgs::Marker goal_node_marker_; //terminal nodes
    goal_node_marker_.id = 0;
    goal_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    goal_node_marker_.header.frame_id = m_base_link_name;
    goal_node_marker_.ns = "terminal_nodes";
    goal_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    goal_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    goal_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    goal_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    goal_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    goal_node_marker_.color.r = 0.0;
    goal_node_marker_.color.g = 1.0;
    goal_node_marker_.color.b = 0.0;
    goal_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    goal_node_marker_.pose.position.x = goal_node.ee_pose[0];
    goal_node_marker_.pose.position.y = goal_node.ee_pose[1];
    goal_node_marker_.pose.position.z = goal_node.ee_pose[2];
    //Add start node to marker
    m_terminal_nodes_marker_array_msg.markers.push_back(goal_node_marker_);



    //--- Create Start Node for RRT* Tree
    Node start_node;
    start_node.node_id = 0;
    start_node.parent_id = 0;
    start_node.config = ik_sol_ee_start_pose;
    start_node.ee_pose = start_ee_pose_quat_orient;
    start_node.cost_reach.total = 0.0;
    start_node.cost_reach.revolute = 0.0;
    start_node.cost_reach.prismatic = 0.0;
    start_node.cost_h.total = m_cost_theoretical_solution_path[0];
    start_node.cost_h.revolute = m_cost_theoretical_solution_path[1];
    start_node.cost_h.prismatic = m_cost_theoretical_solution_path[2];
    //Add start Node to the tree
    m_start_tree.nodes.push_back(start_node);
    m_start_tree.num_nodes++;

    //Add sphere to visualize goal node
    visualization_msgs::Marker start_node_marker_; //terminal nodes
    start_node_marker_.id = 1;
    start_node_marker_.header.stamp = ros::Time::now();
    //Set up properties for terminal nodes marker
    start_node_marker_.header.frame_id = m_base_link_name;
    start_node_marker_.ns = "terminal_nodes";
    start_node_marker_.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    start_node_marker_.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    start_node_marker_.scale.x = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.y = m_terminal_nodes_marker_scale;
    start_node_marker_.scale.z = m_terminal_nodes_marker_scale;
    start_node_marker_.pose.orientation.w = 1.0;
    //Set colour for terminal tree nodes
    start_node_marker_.color.r = 1.0;
    start_node_marker_.color.g = 0.0;
    start_node_marker_.color.b = 0.0;
    start_node_marker_.color.a = 1.0;
    //Set Marker Position for goal node marker
    start_node_marker_.pose.position.x = start_node.ee_pose[0];
    start_node_marker_.pose.position.y = start_node.ee_pose[1];
    start_node_marker_.pose.position.z = start_node.ee_pose[2];
    //Add start node to marke
    m_terminal_nodes_marker_array_msg.markers.push_back(start_node_marker_);


    //Publish start and goal node
    m_tree_terminal_nodes_pub.publish(m_terminal_nodes_marker_array_msg);


    //Save start and goal ee_pose
    m_ee_start_pose = start_ee_pose_quat_orient;
    m_ee_goal_pose = goal_ee_pose_quat_orient;

    //Save start and goal config
    m_config_start_pose = start_node.config;
    m_config_goal_pose = goal_node.config;


    //Empty array of nodes
    m_start_tree_add_nodes_marker.points.empty();
    m_start_tree_node_pub.publish(m_start_tree_add_nodes_marker);
    m_goal_tree_add_nodes_marker.points.empty();
    m_goal_tree_node_pub.publish(m_goal_tree_add_nodes_marker);

    //Empty array of edges
    //m_start_tree_add_edge_marker_array_msg.markers.empty();
    m_start_tree_add_edge_marker_array_msg.markers.clear();
    m_start_tree_edge_pub.publish(m_start_tree_add_edge_marker_array_msg);
    m_goal_tree_add_edge_marker_array_msg.markers.clear();
    m_goal_tree_edge_pub.publish(m_goal_tree_add_edge_marker_array_msg);


    //Delete solution path
    visualization_msgs::Marker empty_solution_path_marker;
    empty_solution_path_marker.id = 1;
    empty_solution_path_marker.header.stamp = ros::Time::now();
    empty_solution_path_marker.header.frame_id = m_base_link_name;
    empty_solution_path_marker.ns = "solution";
    empty_solution_path_marker.action = visualization_msgs::Marker::DELETE;
    empty_solution_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    empty_solution_path_marker.scale.x = m_solution_path_ee_marker_scale;
    empty_solution_path_marker.pose.orientation.w = 1.0;
    empty_solution_path_marker.color.r = 1.0;
    empty_solution_path_marker.color.g = 1.0;
    empty_solution_path_marker.color.b = 1.0;
    empty_solution_path_marker.color.a = 1.0;
    m_ee_solution_path_pub.publish(empty_solution_path_marker);


    //Initialization of Variables required for Ellipse Sampling
    if(search_space == 0) //Planning in Control Space
    {
        //TODO Ellipse Sampling Initialization
    }
    else if (search_space == 1) //Planning in Joint Space
        jointConfigEllipseInitialization();
    else
        ROS_ERROR("Requested planner search space does not exist!!!");


    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Time when planning started and ended
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;


    //If planning is performed in the map frame
    if(m_planning_frame == "/map")
    {
        //Get current pose of robot in the map frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
            m_transform_map_to_base_available = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Transform /map to /base_link not available!");
            m_transform_map_to_base_available = false;
            return false;
        }
    }

    //Everything went fine
    return true;
}


//Run RRT* Planner given a target pose for the endeffector (last link along planning chain) w.r.t the /map frame
bool BiRRTstarPlanner::init_planner_map_goal_pose(const Eigen::Affine3d& goal, const vector<int> constraint_vec_goal_pose, const vector<pair<double,double> > target_coordinate_dev, const string planner_type)
{
    //Set init_ok flag to default value
    bool init_ok = false;

    //------------ START CONFIG (from LOCALIZATION Node) --------------
    vector<double> start_conf(m_num_joints);

    //Set zero config
    for(int j = 0; j < m_num_joints ; j++)
        start_conf[j] = 0.0;

    //Determine whether one of the revolute joints belong to the base (i.e. theta joint)
    // -> If yes, set the index where the manipulator joints start in the start_conf vector
    int offset_manipulator_joint_indices = 0;
    if((m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) || m_num_joints_revolute > 7)
        offset_manipulator_joint_indices = 1;

    //There are revolute joints that belong to the manipulator
    if(m_num_joints_revolute > 1)
    {
        //WAIT FOR START CONFIG (get from /joint_states topic)
        while(!m_lbr_joint_state_received)
        {
            ROS_INFO("Current Manipulator Joint State not available yet");

            ros::spinOnce();
        }

        int start_manipulator_joint_indices = m_num_joints_prismatic+offset_manipulator_joint_indices;
        for(int j = start_manipulator_joint_indices; j < m_num_joints ; j++)
            start_conf[j] = m_lbr_joint_state[j - start_manipulator_joint_indices];
    }


    //------------ GOAL CONFIG (from LOCALIZATION Node) --------------

    //Planning is performed for mobile base
    // Note: For base planning the goal pose is equal to the goal configuration, i.e. (x,y,theta) -> therefore no control-based config generation is run
    if(m_num_joints_prismatic == 2 && m_num_joints_revolute == 1)
    {
            //Get goal config of base from input (in /map frame)
            double goal_x_map, goal_y_map, goal_theta_map;

            //Get rot_x, rot_y, rot_z
            Eigen::Matrix3d rot = goal.linear();
            Eigen::Vector3d angles = rot.eulerAngles(0,1,2);

            //Set desired x,y,theta for base (in /map frame)
            goal_x_map = goal.translation().x();
            goal_y_map = goal.translation().y();
            goal_theta_map = angles(2);


            //Transform map to goal frame
            tf::StampedTransform transform_map_to_goal;
            transform_map_to_goal.setOrigin(tf::Vector3(goal_x_map,goal_y_map, 0.0));
            transform_map_to_goal.setRotation(tf::createQuaternionFromYaw(goal_theta_map));
            //transform_map_to_goal.setRotation(tf::createQuaternionFromYaw(goal_theta_map * (M_PI/180.0)));


            //Get current pose of robot in the map frame
            tf::TransformListener listener;
            try {
                listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
                listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
                m_transform_map_to_base_available = true;
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                ROS_ERROR("Transform /map to /base_link not available!");
                m_transform_map_to_base_available = false;
                return false;
            }


            //Transform goal from map to base frame
            tf::StampedTransform transform_base_to_goal;
            transform_base_to_goal.mult(m_transform_map_to_base.inverse(),transform_map_to_goal);


            //Set Goal (in /base_link frame)
            vector<double> goal_conf_base(3);
            tf::Vector3 goal_trans_base = transform_base_to_goal.getOrigin();
            tf::Quaternion goal_rot_base = transform_base_to_goal.getRotation();
            goal_conf_base[0] = goal_trans_base.x();
            goal_conf_base[1] = goal_trans_base.y();
            double z_dir = transform_base_to_goal.getRotation().getAxis().z();
            goal_conf_base[2] = z_dir > 0.0 ? goal_rot_base.getAngle() : -goal_rot_base.getAngle();

            cout <<"Goal pose in base frame" << endl;
            //cout << goal_pose_base.matrix() << endl << endl;
            cout <<goal_conf_base[0] << endl;
            cout <<goal_conf_base[1] << endl;
            cout <<goal_conf_base[2] << endl<<endl;

            //cout <<goal_rot_base.getAngle() << endl;
            //cout <<goal_rot_base.getAxis().z() << endl;

            //Compute distance between start and goal config
            double x_start_goal_dist = goal_conf_base[0] - start_conf[0];
            double y_start_goal_dist = goal_conf_base[1] - start_conf[1];
            double theta_start_goal_dist = goal_conf_base[2] - start_conf[2];
            //Minimal distances required between start and goal config to trigger planning
            double x_min_dist, y_min_dist, theta_min_dist;
            x_min_dist = 0.1; //10 cm
            y_min_dist = 0.1; //10 cm
            theta_min_dist = 0.5236; // 30 degree

            //Check whether planning is required
            if (fabs(x_start_goal_dist) < x_min_dist && fabs(y_start_goal_dist) < y_min_dist && fabs(theta_start_goal_dist) < theta_min_dist)
            {
                //Trajectory must have at least two points
                m_result_joint_trajectory.push_back(start_conf);
                m_result_joint_trajectory.push_back(start_conf);

                //Set planner type
                m_planner_type = planner_type;
                //Get scenario name
                string scenario_name = m_planning_world->getSceneName();

                //Set path to the file that will store the planned joint trajectory
                string folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_joint_trajectory_run_0.txt";
                m_file_path_joint_trajectory = new char[folder_path.size() + 1];
                copy(folder_path.begin(), folder_path.end(), m_file_path_joint_trajectory);
                m_file_path_joint_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0
                //cout<<m_file_path_joint_trajectory<<endl;

                m_KDLRobotModel->writeTrajectoryToFile(m_result_joint_trajectory,m_file_path_joint_trajectory);

                //Robot already at target pose (success = true)
                return true;
            }

            //------------ PLANNER INITIALIZATION --------------

            //Initialize Planner
            init_ok = init_planner(start_conf,goal_conf_base,1);

            //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid

    }

    //Planning is performed for manipulator
    if(m_num_joints_prismatic == 0 && m_num_joints_revolute != 0)
    {
        //------------ GOAL ENDEFFECTOR POSE (from function input) --------------

        //Get endeffector goal pose from input (in /map frame)
        double goal_x_map, goal_y_map, goal_z_map, goal_rot_x_map, goal_rot_y_map, goal_rot_z_map;

        //Get rot_x, rot_y, rot_z
        Eigen::Matrix3d rot = goal.linear();
        Eigen::Vector3d angles = rot.eulerAngles(0,1,2);

        //Set desired x,y,z,rotX,rotY,rotZ for endeffector (in /map frame)
        goal_x_map = goal.translation().x();
        goal_y_map = goal.translation().y();
        goal_z_map = goal.translation().z();
        goal_rot_x_map = angles(0); //roll
        goal_rot_y_map = angles(1); //pitch
        goal_rot_z_map = angles(2); //yaw


        //Transform from map to endeffector goal frame
        tf::StampedTransform transform_map_to_ee_goal;
        transform_map_to_ee_goal.setOrigin(tf::Vector3(goal_x_map, goal_y_map, goal_z_map));
        transform_map_to_ee_goal.setRotation(tf::createQuaternionFromRPY(goal_rot_x_map, goal_rot_y_map, goal_rot_z_map));


        //Get current pose of manipulator base link in the map frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/map", m_ns_prefix_robot + "lbr_0_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", m_ns_prefix_robot + "lbr_0_link", ros::Time(0), m_transform_map_to_base);
            m_transform_map_to_base_available = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Transform /map to /base_link not available!");
            m_transform_map_to_base_available = false;
            return false;
        }



        //Transform ee goal from map to robot arm base frame
        tf::StampedTransform transform_arm_base_to_goal;
        transform_arm_base_to_goal.mult(m_transform_map_to_base.inverse(),transform_map_to_ee_goal);


        //Set Goal (in /lbr_0_link frame)
        vector<double> goal_pose_endeffector(6);
        //Position
        tf::Vector3 goal_trans_ee = transform_arm_base_to_goal.getOrigin();
        goal_pose_endeffector[0] = goal_trans_ee.x();
        goal_pose_endeffector[1] = goal_trans_ee.y();
        goal_pose_endeffector[2] = goal_trans_ee.z();
        //Orientation
        tf::Quaternion goal_rot_ee = transform_arm_base_to_goal.getRotation();
        tf::Matrix3x3 m(goal_rot_ee);
        m.getRPY(goal_pose_endeffector[3], goal_pose_endeffector[4], goal_pose_endeffector[5]); //Roll, Pitch, Yaw


        cout <<"Goal endeffector pose in robot arm base frame" << endl;
        cout << "Position"<< endl;
        cout <<goal_pose_endeffector[0] << endl;
        cout <<goal_pose_endeffector[1] << endl;
        cout <<goal_pose_endeffector[2] << endl;
        cout << "Orientation"<< endl;
        cout <<goal_pose_endeffector[3] << endl;
        cout <<goal_pose_endeffector[4] << endl;
        cout <<goal_pose_endeffector[5] << endl<<endl;

        //------------ PLANNER INITIALIZATION --------------

        //Initialize Planner
        //Syntax: init_planner(vector<double> start_conf, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space)
        init_ok = init_planner(start_conf, goal_pose_endeffector, constraint_vec_goal_pose, target_coordinate_dev, 1);

        //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid

    }


    //Planning is performed for mobile manipulator
    // -> Note: In this case the goal pose corresponds to the goal config -> thus no need to run controller to generate goal config
    if(m_num_joints_prismatic == 2 && m_num_joints_revolute > 1)
    {
        //------------ GOAL ENDEFFECTOR POSE (from function input) --------------

        //Get endeffector goal pose from input (in /map frame)
        double goal_x_map, goal_y_map, goal_z_map, goal_rot_x_map, goal_rot_y_map, goal_rot_z_map;

        //Get rot_x, rot_y, rot_z
        Eigen::Matrix3d rot = goal.linear();
        Eigen::Vector3d angles = rot.eulerAngles(0,1,2);

        //Set desired x,y,z,rotX,rotY,rotZ for endeffector (in /map frame)
        goal_x_map = goal.translation().x();
        goal_y_map = goal.translation().y();
        goal_z_map = goal.translation().z();
        goal_rot_x_map = angles(0); //roll
        goal_rot_y_map = angles(1); //pitch
        goal_rot_z_map = angles(2); //yaw


        //Transform from map to endeffector goal frame
        tf::StampedTransform transform_map_to_ee_goal;
        transform_map_to_ee_goal.setOrigin(tf::Vector3(goal_x_map, goal_y_map, goal_z_map));
        transform_map_to_ee_goal.setRotation(tf::createQuaternionFromRPY(goal_rot_x_map, goal_rot_y_map, goal_rot_z_map));


        //Get current pose of robot in the map frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
            m_transform_map_to_base_available = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Transform /map to /base_link not available!");
            m_transform_map_to_base_available = false;
            return false;
        }


        //Transform from /base_link frame to goal ee frame
        tf::StampedTransform transform_base_to_ee_goal;
        transform_base_to_ee_goal.mult(m_transform_map_to_base.inverse(),transform_map_to_ee_goal);


        //Set Goal (in /base_link frame)
        vector<double> goal_pose_endeffector(6);
        //Position
        tf::Vector3 goal_trans_ee = transform_base_to_ee_goal.getOrigin();
        goal_pose_endeffector[0] = goal_trans_ee.x();
        goal_pose_endeffector[1] = goal_trans_ee.y();
        goal_pose_endeffector[2] = goal_trans_ee.z();
        //Orientation
        tf::Quaternion goal_rot_ee = transform_base_to_ee_goal.getRotation();
        tf::Matrix3x3 m(goal_rot_ee);
        m.getRPY(goal_pose_endeffector[3], goal_pose_endeffector[4], goal_pose_endeffector[5]); //Roll, Pitch, Yaw


        cout <<"Goal endeffector pose in robot base link frame" << endl;
        cout << "Position"<< endl;
        cout <<goal_pose_endeffector[0] << endl;
        cout <<goal_pose_endeffector[1] << endl;
        cout <<goal_pose_endeffector[2] << endl;
        cout << "Orientation"<< endl;
        cout <<goal_pose_endeffector[3] << endl;
        cout <<goal_pose_endeffector[4] << endl;
        cout <<goal_pose_endeffector[5] << endl<<endl;

        //------------ PLANNER INITIALIZATION --------------

        //Initialize Planner
        //Syntax: init_planner(vector<double> start_conf, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space)
        init_ok = init_planner(start_conf, goal_pose_endeffector, constraint_vec_goal_pose, target_coordinate_dev, 1);

        //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid
    }


    //------------ FEASIBILITY CHECKER INITIALIZATION --------------
    //m_FeasibilityChecker->update_map_to_robot_transform();

    return init_ok;

}


bool BiRRTstarPlanner::init_planner_map_goal_config(const vector<double> goal, const string planner_type)
{
    //Check whether dimension of goal config vector is equal the number of joints in the planning group
    if(goal.size() != m_num_joints)
    {
        ROS_ERROR("Dimension of configuration vector does not match number of joints in planning group!!");
        return false;
    }

    //------------ START CONFIG (from LOCALIZATION Node) --------------
    vector<double> start_conf(m_num_joints);

    //Set zero config
    for(int j = 0; j < m_num_joints ; j++)
        start_conf[j] = 0.0;

    //Determine whether one of the revolute joints belong to the base (i.e. theta joint)
    // -> If yes, set the index where the manipulator joints start in the start_conf vector
    int offset_manipulator_joint_indices = 0;
    if((m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) || m_num_joints_revolute > 7)
        offset_manipulator_joint_indices = 1;

    //There are revolute joints that belong to the manipulator
    if(m_num_joints_revolute > 1)
    {
        //WAIT FOR START CONFIG (get from /joint_states topic)
        while(!m_lbr_joint_state_received)
        {
            ROS_INFO("Current Manipulator Joint State not available yet");

            ros::spinOnce();
        }

        int start_manipulator_joint_indices = m_num_joints_prismatic+offset_manipulator_joint_indices;
        for(int j = start_manipulator_joint_indices; j < m_num_joints ; j++)
            start_conf[j] = m_lbr_joint_state[j - start_manipulator_joint_indices];
    }



    //------------ GOAL CONFIG (from LOCALIZATION Node) --------------

    //Goal configuration in base_link frame
    vector<double> goal_conf(m_num_joints);

    //Planning includes mobile base -> thus base goal pose needs to be transformed from map frame into base_link frame
    if(m_num_joints_prismatic == 2 && m_num_joints_revolute != 0)
    {

            //Get goal config of base from input (in /map frame)
            double goal_x_map, goal_y_map, goal_theta_map;

            //Set desired x,y,theta for base (in /map frame)
            goal_x_map = goal[0];
            goal_y_map = goal[1];
            goal_theta_map = goal[2];

            //Transform map to goal frame
            tf::StampedTransform transform_map_to_goal;
            transform_map_to_goal.setOrigin(tf::Vector3(goal_x_map,goal_y_map, 0.0));
            transform_map_to_goal.setRotation(tf::createQuaternionFromYaw(goal_theta_map));
            //transform_map_to_goal.setRotation(tf::createQuaternionFromYaw(goal_theta_map * (M_PI/180.0)));


            //Get current pose of robot in the map frame
            tf::TransformListener listener;
            try {
                listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
                listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
                m_transform_map_to_base_available = true;
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                ROS_ERROR("Transform /map to /base_link not available!");
                m_transform_map_to_base_available = false;
                return false;
            }


            //Transform goal from map to base frame
            tf::StampedTransform transform_base_to_goal;
            transform_base_to_goal.mult(m_transform_map_to_base.inverse(),transform_map_to_goal);


            //Set Goal (in /base_link frame)
            tf::Vector3 goal_trans_base = transform_base_to_goal.getOrigin();
            tf::Quaternion goal_rot_base = transform_base_to_goal.getRotation();
            goal_conf[0] = goal_trans_base.x();
            goal_conf[1] = goal_trans_base.y();
            double z_dir = transform_base_to_goal.getRotation().getAxis().z();
            goal_conf[2] = z_dir > 0.0 ? goal_rot_base.getAngle() : -goal_rot_base.getAngle();

            ROS_INFO_STREAM("Goal pose in base frame");
            //cout << goal_pose_base.matrix() << endl << endl;
            ROS_INFO_STREAM(goal_conf[0]<<"  "<<goal_conf[1]<<"  "<<goal_conf[2]);
            

            //Check whether planning group includes only the revolute joint of the mobile base
            // -> If yes, check whether the base is already at the correct pose or planning is required
            if(m_num_joints_revolute == 1)
            {
                //Compute distance between start and goal config
                double x_start_goal_dist = goal_conf[0] - start_conf[0];
                double y_start_goal_dist = goal_conf[1] - start_conf[1];
                double theta_start_goal_dist = goal_conf[2] - start_conf[2];
                //Minimal distances required between start and goal config to trigger planning
                double x_min_dist, y_min_dist, theta_min_dist;
                x_min_dist = 0.1; //10 cm
                y_min_dist = 0.1; //10 cm
                theta_min_dist = 0.5236; // 30 degree

                //Check whether planning is required
                if (fabs(x_start_goal_dist) < x_min_dist && fabs(y_start_goal_dist) < y_min_dist && fabs(theta_start_goal_dist) < theta_min_dist)
                {
                    //Trajectory must have at least two points
                    m_result_joint_trajectory.push_back(start_conf);
                    m_result_joint_trajectory.push_back(start_conf);

                    //Set planner type
                    m_planner_type = planner_type;
                    //Get scenario name
                    string scenario_name = m_planning_world->getSceneName();

                    //Set path to the file that will store the planned joint trajectory
                    string folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_joint_trajectory_run_0.txt";
                    m_file_path_joint_trajectory = new char[folder_path.size() + 1];
                    copy(folder_path.begin(), folder_path.end(), m_file_path_joint_trajectory);
                    m_file_path_joint_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0
                    //cout<<m_file_path_joint_trajectory<<endl;

                    m_KDLRobotModel->writeTrajectoryToFile(m_result_joint_trajectory,m_file_path_joint_trajectory);

                    //Robot already at target pose (success = true)
                    return true;
                }
            }
    }

    //Planning is performed using also joints of the manipulator
    if(m_num_joints_revolute > 1)
    {
        int start_manipulator_joint_indices = m_num_joints_prismatic+offset_manipulator_joint_indices;
        for(int j = start_manipulator_joint_indices; j < m_num_joints ; j++)
            goal_conf[j] = goal[j];

    }

    //------------ PLANNER INITIALIZATION --------------

    //Initialize Planner
    bool init_ok = init_planner(start_conf,goal_conf,1);
    //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid


    //------------ FEASIBILITY CHECKER INITIALIZATION --------------
    //m_FeasibilityChecker->update_map_to_robot_transform();
    

    return init_ok;
}


//Get the planning frame from the SRDF description
string BiRRTstarPlanner::getPlanningFrameFromSRDF(string robot_desciption_param)
{
    //Planning frame
    string planning_frame;

    //Check the planning frame (from virtual joint in srdf)
    boost::shared_ptr<srdf::Model> srdf_robot;
    boost::shared_ptr<urdf::ModelInterface> urdf_robot;

    //Get param content
    std::string content;
    if (!m_nh.getParam(robot_desciption_param, content))
    {
         ROS_ERROR("Robot model parameter empty '%s'?", robot_desciption_param.c_str());
         return "none";
    }

    urdf::Model* umodel = new urdf::Model();
    if (!umodel->initString(content))
    {
      ROS_ERROR("Unable to parse URDF from parameter '%s'", robot_desciption_param.c_str());
      return "none";
    }
    urdf_robot.reset(umodel);

    const std::string srdf_description(robot_desciption_param + "_semantic");
    std::string scontent;
    if (!m_nh.getParam(srdf_description, scontent))
    {
      ROS_ERROR("Robot semantic description not found. Did you forget to define or remap '%s'?", srdf_description.c_str());
     return "none";
   }

    srdf_robot.reset(new srdf::Model());
    if (!srdf_robot->initString(*urdf_robot, scontent))
    {
      ROS_ERROR("Unable to parse SRDF from parameter '%s'", srdf_description.c_str());
      srdf_robot.reset();
      return "none";
    }

    //Set planning frame class variable
    const std::vector< srdf::Model::VirtualJoint > &virtual_joint = srdf_robot->getVirtualJoints();
    planning_frame =  "/" + virtual_joint[0].parent_frame_;

    return planning_frame;
}


bool BiRRTstarPlanner::run_planner(int search_space, bool flag_iter_or_time, double max_iter_time, bool show_tree_vis, double iter_sleep, int planner_run_number)
{

    // ----- Path to files storing the result joint and endeffector Trajectory of the RRT* planner-----------

    stringstream convert_planner_run_number; // stringstream used for the conversion
    string string_planner_run_number; //string which will contain the result
    convert_planner_run_number << planner_run_number; //add the value of Number to the characters in the stream
    string_planner_run_number = convert_planner_run_number.str(); //set Result to the content of the stream

    //Get Planning Scenario Name
    string scenario_name = m_planning_world->getSceneName();

    //Get planner type depending on planner settings
    if(m_informed_sampling_active == false && m_tree_optimization_active == false)
        m_planner_type = "bi_rrt_connect";
    else if (m_informed_sampling_active == true && m_tree_optimization_active == false)
        m_planner_type = "bi_informed_rrt";
    else if (m_informed_sampling_active == false && m_tree_optimization_active == true)
        m_planner_type = "bi_rrt_star";
    else
        m_planner_type = "bi_informed_rrt_star";


    //Set path to the file that will store the planned joint trajectory
    string folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_joint_trajectory_run_" + string_planner_run_number +".txt";
    m_file_path_joint_trajectory = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), m_file_path_joint_trajectory);
    m_file_path_joint_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<m_file_path_joint_trajectory<<endl;

    //Set path to the file that will store the planned ee trajectory
    folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_ee_trajectory_run_" + string_planner_run_number +".txt";
    m_file_path_ee_trajectory = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), m_file_path_ee_trajectory);
    m_file_path_ee_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<m_file_path_ee_trajectory<<endl;


    // ----- Path to files storing RRT* planner statistics-----------

    //Set path to the file that will store the planner statistics
    folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_planner_statistics_run_" + string_planner_run_number +".txt";
    m_file_path_planner_statistics = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), m_file_path_planner_statistics);
    m_file_path_planner_statistics[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<m_file_path_planner_statistics<<endl;

    //Set path to file that will store the Solution Path Cost Evolution
    folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_cost_evolution_run_" + string_planner_run_number +".txt";
    m_file_path_cost_evolution = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), m_file_path_cost_evolution);
    m_file_path_cost_evolution[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<m_file_path_cost_evolution<<endl;


    // ---------- RRT* planner Initialization -----------

    if(flag_iter_or_time == 0)
    {
        //Init maximum permitted planner iterations
        m_max_planner_iter = max_iter_time;
    }
    else if (flag_iter_or_time == 1)
    {   //Init maximum permitted planner time
        m_max_planner_time = max_iter_time;
    }
    else
        ROS_ERROR("You need to set either maximum planner iterations or time!");


    //Set number of executed planner iterations to zero
    m_executed_planner_iter = 0;

    //Random ee_pose
    vector<double> ee_pose_rand;

    //Random config
    KDL::JntArray config_rand;

    //Random new node
    Node x_rand;

    //Initialize the two trees (start with expansion using the start tree)
    Rrt_star_tree *tree_A;
    tree_A = &m_start_tree;
    Rrt_star_tree *tree_B;
    tree_B = &m_goal_tree;

    //Planner progress message
    std_msgs::Float32 msg_planner_progress;


    // ------------ Check whether Planning is needed by trying to directly connect the root nodes -----------

    //Try to connect the two RRT* Trees given the root nodes of "tree_A" and "tree_B"
    connectGraphsInterpolation(tree_A, tree_A->nodes[0], tree_B->nodes[0], show_tree_vis);

    //Check whether solution path is available after connectGraphsInterpolation
    bool no_planning = m_solution_path_available;
    if(no_planning)
    {
        //Set planner progress to 100% indicating that planning is done (a straight line solution is found)
        msg_planner_progress.data = 100.0;
        m_pub_planning_progress.publish(msg_planner_progress);

        ROS_INFO_STREAM("No Planning required. Straight line connecting start and goal config in configuration space is valid.");
    }

    // ------------ RRT* Planning -----------

    //Variable for timer (measuring time required to find a first solution path and total planning time)
    gettimeofday(&m_timer, NULL);
    m_time_planning_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);


    //Start planning
    while(no_planning == false && ((flag_iter_or_time == 0 && m_executed_planner_iter < m_max_planner_iter) || (flag_iter_or_time == 1 && m_executed_planner_time < m_max_planner_time)))
    {

        //cout<<"Current iter: "<<m_executed_planner_iter<<endl;

        // -- CARTESIAN SPACE SEARCH
        if(search_space == 0)
        {

            //Sample from the informed subject once a solution is available
            if(m_solution_path_available == true)
            {
                ee_pose_rand = sampleEEposefromEllipse();
            }
            else
            {
                //Sample EE pose uniformly
                ee_pose_rand = sampleEEpose();
            }

            //Set Data for random new Node
            x_rand.node_id = tree_A->nodes.size();
            x_rand.ee_pose = ee_pose_rand;

            //Find the nearest neighbour node (closest node using a particular heuristic)
            Node nn_node = find_nearest_neighbour_control(tree_A, x_rand.ee_pose);


            //Connect Nodes (running Variable DLS Controller with start config from nearest neighbour and goal pose equal to x_rand)
            Node x_new; //New node and edge generated in connectNodesControl - function
            Edge e_new;
            kuka_motion_controller::Status tree_extend_state = connectNodesControl(tree_A, nn_node, x_rand, x_new, e_new);
            //Check validity of edge connecting nn_node to x_rand
            bool tree_extend_NN = m_FeasibilityChecker->isEdgeValid(e_new);


            //Set cost to reach new node to large value in order to allow near vertices to connect to the new node
            if(tree_extend_NN == false)
            {
                //New Node has not been reached from nearest neighbour (set cost for new node to be considered in parent node search)
                x_new.cost_reach.total = 10000.0;
            }

            vector<int> near_nodes;
            bool tree_extend_BP = false;
            if(m_solution_path_available == true)
            {
                //Find the set of vertices in the vicinity of x_new
                near_nodes = find_near_vertices_control(tree_A, x_new);

                //Choose Parent for x_new minimizing the cost of reaching x_new (given the set of vertices surrounding x_new as potential parents)
                tree_extend_BP = choose_node_parent_control(tree_A, near_nodes, nn_node, e_new, x_new);
            }


            //If a parent for the new node has been found (either the nearest neighbour or another vertex in the set near_nodes)
            if(tree_extend_NN == true || tree_extend_BP == true)
            {
                //Insert Node and Edge into the RRT* Tree
                insertNode(tree_A, e_new, x_new, show_tree_vis);


                if(m_solution_path_available == true)
                {
                    //Rewire Nodes of the Tree, i.e. checking if some if the nodes can be reached by a lower cost path (due to the new node added to the tree previously)
                    rewireTreeControl(tree_A, near_nodes, x_new, show_tree_vis);
                }

                //Search for nearest neighbour to x_new in tree "tree_B"
                Node x_connect = find_nearest_neighbour_control(tree_B, x_new.ee_pose);

                //Try to connect the two RRT* Trees given the new node added to "tree_A" and its nearest neighbour in "tree_B"
                connectGraphsControl(tree_B, x_connect, x_new, show_tree_vis);
            }
            else
            {
                //cout<<"No connection to node found!!!"<<endl;
            }


        }


        // -- C-SPACE SEARCH
        if(search_space == 1)
        {
            //ROS_INFO_STREAM("Current iter: "<<m_executed_planner_iter);

            //Sample from the informed subject once a solution is available
            if(m_informed_sampling_active == true && m_solution_path_available == true)
            {
                config_rand = sampleJointConfigfromEllipse_JntArray();
            }
            else
            {
                //Sample Joint configuration uniformly
                config_rand = sampleJointConfig_JntArray();
            }


            //Set Data for random new Node
            x_rand.node_id = tree_A->nodes.size();
            x_rand.config = m_RobotMotionController->JntArray_to_Vector(config_rand);
            x_rand.ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,config_rand);

            //Variable for timer
            //gettimeofday(&m_timer, NULL);
            //double time_nearest_neighbour_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

            //Find the nearest neighbour node (closest node using a particular heuristic)
            Node nn_node = find_nearest_neighbour_interpolation(tree_A, x_rand.config);

            //Get time elapsed
            //gettimeofday(&m_timer, NULL);
            //double time_nearest_neighbour_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
            //cout<<"Time Nearest Neighbour: "<<(time_nearest_neighbour_end - time_nearest_neighbour_start)<<endl;

            //New node and edge that will be generated in connectNodesInterpolation - function
            Node x_new;
            Edge e_new;


            //Variable for timer
            //gettimeofday(&m_timer, NULL);
            //double time_expand_tree_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

            //Flag indicating whether a new edge from the nearest node towards x_rand has been found
            bool tree_extend_NN = expandTree(tree_A, nn_node, x_rand, x_new, e_new, show_tree_vis);

            //Get time elapsed
            //gettimeofday(&m_timer, NULL);
            //double time_expand_tree_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
            //cout<<"Time Expand Tree: "<<(time_expand_tree_end - time_expand_tree_start)<<endl;


            //Set cost to reach new node to large value in order to allow near vertices to connect to the new node
            if(tree_extend_NN == false)
            {
                //New Node has not been reached from nearest neighbour (set cost for new node to be considered in parent node search)
                x_new.cost_reach.total = 10000.0;
            }


            //Near vertices set (in the vicinity of x_new)
            vector<int> near_nodes;
            bool tree_extend_BP = false;
            //Consider nearest neighbours for parent search when "expandTree" failed or a solution path has been found
            if((m_tree_optimization_active == true && m_solution_path_available == true))// || tree_extend_NN == false)
            {
                //Find the set of vertices in the vicinity of x_new
                near_nodes = find_near_vertices_interpolation(tree_A, x_new);

                //Choose Parent for x_new minimizing the cost of reaching x_new (given the set of vertices surrounding x_new as potential parents)
                tree_extend_BP = choose_node_parent_interpolation(tree_A, near_nodes, nn_node, e_new, x_new, show_tree_vis);

            }


            //If a parent for the new node has been found (either the nearest neighbour or another vertex in the set near_nodes)
            if(tree_extend_NN == true || tree_extend_BP == true)
            {
                //Insert Node and Edge into the RRT* Tree
                insertNode(tree_A, e_new, x_new, show_tree_vis);

                //Rewire tree only if tree optimization is activated and a solution already has been found
                if(m_tree_optimization_active == true && m_solution_path_available == true)
                {
                    //Variable for timer
                    //gettimeofday(&m_timer, NULL);
                    //double time_rewire_tree_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                    //Rewire Nodes of the Tree, i.e. checking if some if the nodes can be reached by a lower cost path (due to the new node added to the tree previously)
                    rewireTreeInterpolation(tree_A, near_nodes, x_new, show_tree_vis);

                    //Get time elapsed
                    //gettimeofday(&m_timer, NULL);
                    //double time_rewire_tree_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                    //cout<<"Time Rewire Tree: "<<(time_rewire_tree_end - time_rewire_tree_start)<<endl;
                }

                //Search for nearest neighbour to x_new in tree "tree_B"
                Node x_connect = find_nearest_neighbour_interpolation(tree_B, x_new.config);

                //Variable for timer
                //gettimeofday(&m_timer, NULL);
                //double time_connect_graphs_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                //Try to connect the two RRT* Trees given the new node added to "tree_A" and its nearest neighbour in "tree_B"
                connectGraphsInterpolation(tree_B, x_connect, x_new, show_tree_vis);


                //Get time elapsed
                //gettimeofday(&m_timer, NULL);
                //double time_connect_graphs_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                //cout<<"Time Connect Graphs Tree: "<<(time_connect_graphs_end - time_connect_graphs_start)<<endl;

            }
            else
            {
                //cout<<"No connection to node found!!!"<<endl;
            }
        }


       //int test;
       //cin>>test;


        //SWAP TREES
        bool tree_idx = tree_A->name == "START" ? true : false;
        if(tree_idx == true)
        {
            tree_A = &m_goal_tree;
            tree_B = &m_start_tree;
            //cout<<"Trees swapped 1"<<endl;
        }
        else
        {
            tree_A = &m_start_tree;
            tree_B = &m_goal_tree;
            //cout<<"Trees swapped 2"<<endl;
        }


        //TREE VISUALIZATION
        if(show_tree_vis == true)
        {
            //Publish tree nodes
            m_start_tree_node_pub.publish(m_start_tree_add_nodes_marker);
            m_goal_tree_node_pub.publish(m_goal_tree_add_nodes_marker);

            //Publish tree edges to be added
            m_start_tree_edge_pub.publish(m_start_tree_add_edge_marker_array_msg);
            m_goal_tree_edge_pub.publish(m_goal_tree_add_edge_marker_array_msg);

            //Publish tree edges to be added to be removed
            //m_start_tree_edge_pub.publish(remove_edge_marker_array_msg_);

            //Sleep to better see tree construction
            ros::Duration(iter_sleep).sleep();
        }


        //Increment iteration counter
        m_executed_planner_iter++;


        //Get time elapsed since planner started
        gettimeofday(&m_timer, NULL);
        double current_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
        double planning_time_elapsed = current_time - m_time_planning_start;

        //Set executed planning time to "planning_time_elapsed"
        m_executed_planner_time = planning_time_elapsed;

        double elapsed_time_percent = (planning_time_elapsed/m_max_planner_time)*100.0;
        double elapsed_time_percent_output = elapsed_time_percent < 100.0 ? elapsed_time_percent : 100.0;
        //ROS_INFO_STREAM("Time elapsed for planning (% of max. planning time): "<<elapsed_time_percent_output<<"%");
        msg_planner_progress.data = elapsed_time_percent_output;
        m_pub_planning_progress.publish(msg_planner_progress);

        //Store Evolution of the best solution cost
        vector<double> current_optimal_cost(5);
        current_optimal_cost [0] = m_executed_planner_iter;
        current_optimal_cost [1] = planning_time_elapsed;
        current_optimal_cost [2] = m_cost_best_solution_path;
        current_optimal_cost [3] = m_cost_best_solution_path_revolute;
        current_optimal_cost [4] = m_cost_best_solution_path_prismatic;
        m_solution_cost_trajectory.push_back(current_optimal_cost);

        //Stop when current solution path is close enough to theoretical optimal solution path (value stored in start and goal node)
        if((m_cost_best_solution_path - m_start_tree.nodes[0].cost_h.total) < m_path_optimality_treshold)
        {
            ROS_INFO_STREAM("Solution Path within optimality treshold found, leaving planning loop ....");

            //Publish planning progress 100%
            msg_planner_progress.data = 100.0;
            m_pub_planning_progress.publish(msg_planner_progress);

            //Leave planning loop
            break;
        }


    }// end of main planner loop


    //Get time elapsed
    gettimeofday(&m_timer, NULL);
    double time_planning_final = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
    m_time_planning_end = time_planning_final - m_time_planning_start;


    cout<<endl;
    ROS_INFO_STREAM("****************** Planner Statistics **********************");

    ROS_INFO_STREAM("Planning performed for group: "<<m_planning_group);

    ROS_INFO_STREAM("Planner result: "<<(m_planner_success == 1 ? "success" : "failure"));

    //Generate output depending on whether max. iterations or plannig time has been set
    if(flag_iter_or_time == 0)
    {
        ROS_INFO_STREAM("Planner performed: "<<m_executed_planner_iter<<" out of a maximum of: "<<m_max_planner_iter<<" iterations");
        ROS_INFO_STREAM("Total Planning Time: "<<m_executed_planner_time<<" sec");
    }
    else
    {
        ROS_INFO_STREAM("Planner run: "<<m_executed_planner_time<<"sec out of a maximum available time of: "<<m_max_planner_time<<"sec");
        ROS_INFO_STREAM("Total Planning Iterations: "<<m_executed_planner_iter<<" iterations");
    }

    ROS_INFO_STREAM("Iterations required to find first Solution Path: "<<m_first_solution_iter);
    ROS_INFO_STREAM("Time required to find first Solution Path: "<<m_time_first_solution<<" sec");
    ROS_INFO_STREAM("Iterations required to find last Solution Path: "<<m_last_solution_iter);
    ROS_INFO_STREAM("Time required to find last Solution Path: "<<m_time_last_solution<<" sec");

    ROS_INFO_STREAM("Cost of Theoretical Optimal Solution Path (no coll.obj. and constraints): "<<m_start_tree.nodes[0].cost_h.total);
    ROS_INFO_STREAM("Cost of Final Solution Path (Total): "<<m_cost_best_solution_path);
    ROS_INFO_STREAM("Cost of Final Solution Path (Revolute): "<<m_cost_best_solution_path_revolute);
    ROS_INFO_STREAM("Cost of Final Solution Path (Prismatic): "<<m_cost_best_solution_path_prismatic);

    ROS_INFO_STREAM("Number of nodes generated (start tree): "<<m_start_tree.num_nodes);
    ROS_INFO_STREAM("Number of nodes generated (goal tree): "<<m_goal_tree.num_nodes);
    ROS_INFO_STREAM("Total number of nodes generated: "<<m_start_tree.num_nodes + m_goal_tree.num_nodes);

    ROS_INFO_STREAM("Number of edges in the tree (start tree): "<<m_start_tree.num_edges);
    ROS_INFO_STREAM("Number of edges in the tree (goal tree): "<<m_goal_tree.num_edges);
    ROS_INFO_STREAM("Total number of edges in the trees: "<<m_start_tree.num_edges + m_goal_tree.num_edges);

    ROS_INFO_STREAM("Number of rewire operations (start tree): "<<m_start_tree.num_rewire_operations);
    ROS_INFO_STREAM("Number of rewire operations (goal tree): "<<m_goal_tree.num_rewire_operations);
    ROS_INFO_STREAM("Total number of rewire operations : "<<m_start_tree.num_rewire_operations + m_goal_tree.num_rewire_operations);

    ROS_INFO_STREAM("************************************************************");

     //Consistency check
    no_two_parents_check(tree_A);
    no_two_parents_check(tree_B);

    if(m_solution_path_available == true)
    {
        //Compute Final joint and endeffector trajectory (and writes them to a file)
        computeFinalSolutionPathTrajectories();
    }

    //Write Planner Statistics to File
    writePlannerStatistics(m_file_path_planner_statistics, m_file_path_cost_evolution);

    //Return planner result (true = success, false = failure)
    return m_solution_path_available;

}



//Reset RRT* Planner Data and Configuration (i.e. edge costs constraints etc.)
//-> Planner is now in its default configuration
void BiRRTstarPlanner::reset_planner_and_config()
{

    //--------- Reset Variables, Flags and Data Structures ----------

    //Resets tree data and flags
    reset_planner_only();


    //--------- Reset Planner Configuration ----------

    //Tree Optimization Flag
    m_tree_optimization_active = true;

    //Informed Sampling Heuristic Flag
    m_informed_sampling_active = true;

    //Init Bidirectional planner type
    m_planner_type = "bi_informed_rrt_star";

    //Set constraints active to false
    m_constraint_active = false;

    //Reset task frame reference
    m_task_pos_global = true;
    m_task_orient_global = true;

    for(int i = 0 ; i < m_constraint_vector.size() ; i++)
    {
        m_constraint_vector[i] = 0;
        m_coordinate_dev[i].first = 0.0;
        m_coordinate_dev[i].second = 0.0;
    }

    //Reset environment size
    m_env_size_x[0] = 0.0; //env dim in negative x dir
    m_env_size_x[1] = 0.0; //env dim in positive x dir
    m_env_size_y[0] = 0.0; //env dim in negative y dir
    m_env_size_y[1] = 0.0; //env dim in positive y dir

    //Initialize edge cost weights
    for(int j = 0 ; j < m_num_joints ; j++)
        m_edge_cost_weights[j] = 1.0;

}


//Reset RRT* Planner Data but keep configuration (i.e. planner type, environment size. edge cost weights etc.)
//-> Planner is now in its initial state (planner settings and environment data is kept)
void BiRRTstarPlanner::reset_planner_only()
{

    //--------- Reset Variables and Flags ----------

    //Init total number of nodes and edges contained in the start tree
    m_start_tree.num_nodes = 0;
    m_start_tree.num_edges = 0;
    m_start_tree.num_rewire_operations = 0;

    //Init total number of nodes and edges contained in the goal tree
    m_goal_tree.num_nodes = 0;
    m_goal_tree.num_edges = 0;
    m_goal_tree.num_rewire_operations = 0;

    //Init Cost of current solution path
    m_cost_best_solution_path = 10000.0;
    m_cost_best_solution_path_revolute = 10000.0;
    m_cost_best_solution_path_prismatic = 10000.0;

    //Init Cost of theoretical solution path
    m_cost_theoretical_solution_path[0] = 0.0;
    m_cost_theoretical_solution_path[1] = 0.0;
    m_cost_theoretical_solution_path[2] = 0.0;

    //Init Maximal Planner iterations and actually executed planner iterations
    m_max_planner_iter = 0;
    m_executed_planner_iter = 0;

    //Init Maximal Planner run time and actually executed planner run time
    m_max_planner_time = 0.0;
    m_executed_planner_time = 0.0;

    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Overall Runtime of planner
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;

    //Set solution path available flag
    m_solution_path_available = false;


    //Reset tree connection data
    m_connected_tree_name = "none";
    //m_node_tree_B.; //Node of the tree "m_connected_tree_name" that has connected "to m_node_tee_A"
    //m_node_tree_A; //Node of the other tree that has been reached by tree "m_connected_tree_name"

    //Reset flag indicating whether map to base_link transform is available
    m_transform_map_to_base_available = false;

    //Reset Feasibility checker data
    m_FeasibilityChecker->reset_data();

    //--------- Reset Data Structures ----------

    //Reset tree node arrays
    m_start_tree.nodes.clear();
    m_goal_tree.nodes.clear();


    //Reset planning results
    m_result_joint_trajectory.clear();
    m_result_ee_trajectory.clear();
    m_solution_cost_trajectory.clear();


    //Reset markers used for visualization
    //Clear terminal node for visualization
    //m_terminal_nodes_marker_array_msg.markers.clear();
    //Clear edges for visualization
    m_start_tree_add_edge_marker_array_msg.markers.clear();
    m_goal_tree_add_edge_marker_array_msg.markers.clear();
    //Clear nodes for visualization
    m_start_tree_add_nodes_marker.points.clear();
    m_goal_tree_add_nodes_marker.points.clear();

    //Flag indicating that lbr joint state is available
    m_lbr_joint_state_received = false;

    //Set Result of Motion Planning to failure (= 0)
    m_planner_success = 0;

}

//Reset planner to initial trees (containing only start and goal config)
// -> Planner is now in its initial state (planner settings and environment data is kept)
// -> No init_planner function call required afterwards
void BiRRTstarPlanner::reset_planner_to_initial_state(){

    //--------- Reset Variables ----------

    //Init total number of nodes and edges contained in the start tree
    m_start_tree.num_nodes = 1;
    m_start_tree.num_edges = 0;
    m_start_tree.num_rewire_operations = 0;

    //Init total number of nodes and edges contained in the goal tree
    m_goal_tree.num_nodes = 1;
    m_goal_tree.num_edges = 0;
    m_goal_tree.num_rewire_operations = 0;

    //Init Cost of current solution path
    m_cost_best_solution_path = 10000.0;
    m_cost_best_solution_path_revolute = 10000.0;
    m_cost_best_solution_path_prismatic = 10000.0;

    //Init Maximal Planner iterations and actually executed planner iterations
    m_max_planner_iter = 0;
    m_executed_planner_iter = 0;

    //Init Maximal Planner run time and actually executed planner run time
    m_max_planner_time = 0.0;
    m_executed_planner_time = 0.0;

    //Iteration and time when first and last solution is found
    m_first_solution_iter = 10000;
    m_last_solution_iter = 10000;
    m_time_first_solution = 10000.0;
    m_time_last_solution = 10000.0;

    //Overall Runtime of planner
    m_time_planning_start = 0.0;
    m_time_planning_end = 0.0;

    //Set solution path available flag
    m_solution_path_available = false;


    //Reset tree connection data
    m_connected_tree_name = "none";
    //m_node_tree_B.; //Node of the tree "m_connected_tree_name" that has connected "to m_node_tee_A"
    //m_node_tree_A; //Node of the other tree that has been reached by tree "m_connected_tree_name"

    //Reset flag indicating whether map to base_link transform is available
    m_transform_map_to_base_available = false;


    //--------- Reset Data Structures ----------

    //Reset Start and goal tree

    //Reset outgoing edges of start and goal node
    m_start_tree.nodes[0].outgoing_edges.clear();
    m_goal_tree.nodes[0].outgoing_edges.clear();

    //Extract start and goal node from the tree nodes array
    vector<Node> start_tree_node, goal_tree_node;
    start_tree_node.push_back(m_start_tree.nodes[0]);
    goal_tree_node.push_back(m_goal_tree.nodes[0]);
    //Replace node array of trees with the new array containing only start/goal node
    m_start_tree.nodes.swap(start_tree_node);
    m_goal_tree.nodes.swap(goal_tree_node);


    //Reset planning results
    m_result_joint_trajectory.clear();
    m_result_ee_trajectory.clear();
    m_solution_cost_trajectory.clear();


    //Reset markers used for visualization
    //Clear terminal node for visualization
    //m_terminal_nodes_marker_array_msg.markers.clear();
    //Clear edges for visualization
    m_start_tree_add_edge_marker_array_msg.markers.clear();
    m_goal_tree_add_edge_marker_array_msg.markers.clear();
    //Clear nodes for visualization
    m_start_tree_add_nodes_marker.points.clear();
    m_goal_tree_add_nodes_marker.points.clear();

    //Flag indicating that lbr joint state is available
    m_lbr_joint_state_received = false;

    //Set Result of Motion Planning to failure (= 0)
    m_planner_success = 0;

}



//Set the planning scene
void BiRRTstarPlanner::setPlanningSceneInfo(vector<double> size_x, vector<double> size_y, string scene_name, bool show_env_borders)
{
    //Update environment size and name
    m_env_size_x[0] = size_x[0];
    m_env_size_x[1] = size_x[1];
    m_env_size_y[0] = size_y[0];
    m_env_size_y[1] = size_y[1];
    m_planning_world->setSceneName(scene_name);
    if(show_env_borders)
        m_planning_world->insertEnvironmentBorders(m_env_size_x,m_env_size_y,0.6,scene_name);
}

void BiRRTstarPlanner::setPlanningSceneInfo(planning_world::PlanningWorldBuilder world_builder)
{
    world_builder.getEnvironmentDimensions(m_env_size_x,m_env_size_y);
    m_planning_world->setSceneName(world_builder.getSceneName());
}



//LBR Joint State Subscriber Callback
void BiRRTstarPlanner::callback_lbr_joint_states(const sensor_msgs::JointState::ConstPtr& msg){

    //Get current position
    m_lbr_joint_state[0] = msg->position[0];
    m_lbr_joint_state[1] = msg->position[1];
    m_lbr_joint_state[2] = msg->position[2];
    m_lbr_joint_state[3] = msg->position[3];
    m_lbr_joint_state[4] = msg->position[4];
    m_lbr_joint_state[5] = msg->position[5];
    m_lbr_joint_state[6] = msg->position[6];

    //Flag indicating that lbr joint state is available
    m_lbr_joint_state_received = true;
}


////Implementation of the virtual function "move" defined in Abstract Class robot_interface_definition
//bool BiRRTstarPlanner::plan(const Eigen::Affine3d& goal){

//    //Planning result
//    bool success = false;

//    if(m_planning_group == "omnirob_base" || m_planning_group == "pr2_base")
//    {
//        //Move base to goal pose
//        success = move_base(goal);
//    }
//    else if (m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "pr2_base_arm")
//    {
//        //Move endeffector to goal (using also the base)
//        success = move_base_endeffector(goal);
//    }
//    else if (m_planning_group == "kuka_complete_arm" || m_planning_group == "pr2_arm")
//    {
//        //Move endeffector to goal (fixed base)
//        success = move_endeffector(goal);
//    }
//    else{
//        ROS_ERROR("You defined an unknown planning group!");
//        return false;
//    }

//    //Reset the planner data
//    reset_planner_only();

//    //Return planning result
//    return success;

//}

////Planning for moving the omnirob_base from the current pose in the map to a target destination
//bool BiRRTstarPlanner::move_base(const Eigen::Affine3d& goal)
//{
//    //Notes:
//    // -> Start Config for base is always x,y,theta = 0, 0, 0 (planning is performed in base_link)
//    // -> Goal config must be expressed in base_link (map to base_link + map to goal pose transforms are known)

//    //------------ WORLD SETUP --------------

//    //Note:
//    //  -> size of environment defined w.r.t. /base_link of /map frame depending on "m_planning_frame"
//    //World dimensions
//    vector<double> env_size_x(2);
//    env_size_x[0] = -4.0;
//    env_size_x[1] = 1.0;
//    vector<double> env_size_y(2);
//    env_size_y[0] = -1.0;
//    env_size_y[1] = 4.0;
//    setPlanningSceneInfo(env_size_x,env_size_y,"neurobots_demo");


//    //------------ START CONFIG (from LOCALIZATION Node) --------------
//    vector<double> start_conf_base(3);

//    //Set config to zero (i.e. robot is at position of base_link frame given by localization node)
//    start_conf_base[0] = 0.0; //x
//    start_conf_base[1] = 0.0; //y
//    start_conf_base[2] = 0.0; //theta


//    //------------ GOAL CONFIG (from LOCALIZATION Node) --------------

//    //Get goal config of base from input (in /map frame)
//    double goal_x_map, goal_y_map, goal_theta_map;

//    //Get rot_x, rot_y, rot_z
//    Eigen::Matrix3d rot = goal.linear();
//    Eigen::Vector3d angles = rot.eulerAngles(0,1,2);

//    //Set desired x,y,theta for base (in /map frame)
//    goal_x_map = goal.translation().x();
//    goal_y_map = goal.translation().y();
//    goal_theta_map = angles(2);


//    //Transform map to goal frame
//    tf::StampedTransform transform_map_to_goal;
//    transform_map_to_goal.setOrigin(tf::Vector3(goal_x_map,goal_y_map, 0.0));
//    transform_map_to_goal.setRotation(tf::createQuaternionFromYaw(goal_theta_map));
//    //transform_map_to_goal.setRotation(tf::createQuaternionFromYaw(goal_theta_map * (M_PI/180.0)));


//    //Get current pose of robot in the map frame
//    tf::TransformListener listener;
//    tf::StampedTransform transform_map_to_base;
//    try {
//        listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
//        listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), transform_map_to_base);
//    } catch (tf::TransformException ex) {
//        ROS_ERROR("%s",ex.what());
//        ROS_ERROR("Transform /map to /base_link not available!");
//        return false;
//    }


//    //Transform goal from map to base frame
//    tf::StampedTransform transform_base_to_goal;
//    transform_base_to_goal.mult(transform_map_to_base.inverse(),transform_map_to_goal);


//    //Set Goal (in /base_link frame)
//    vector<double> goal_conf_base(3);
//    tf::Vector3 goal_trans_base = transform_base_to_goal.getOrigin();
//    tf::Quaternion goal_rot_base = transform_base_to_goal.getRotation();
//    goal_conf_base[0] = goal_trans_base.x();
//    goal_conf_base[1] = goal_trans_base.y();
//    double z_dir = transform_base_to_goal.getRotation().getAxis().z();
//    goal_conf_base[2] = z_dir > 0.0 ? goal_rot_base.getAngle() : -goal_rot_base.getAngle();

//    //Convert goal_pose for base from vector to Eigen::Affine3d
//    Eigen::Affine3d goal_pose_base;
//    Eigen::Affine3d trans_base(Eigen::Translation3d(Eigen::Vector3d(goal_conf_base[0],goal_conf_base[1],0)));
//    Eigen::Affine3d rot_base = Eigen::Affine3d(Eigen::AngleAxisd(goal_conf_base[2], Eigen::Vector3d(0, 0, 1)));
//    goal_pose_base = trans_base * rot_base;

//    cout <<"Goal pose in base frame" << endl;
//    //cout << goal_pose_base.matrix() << endl << endl;
//    cout <<goal_conf_base[0] << endl;
//    cout <<goal_conf_base[1] << endl;
//    cout <<goal_conf_base[2] << endl<<endl;

//    //cout <<goal_rot_base.getAngle() << endl;
//    //cout <<goal_rot_base.getAxis().z() << endl;

//    //Compute distance between start and goal config
//    double x_start_goal_dist = goal_conf_base[0] - start_conf_base[0];
//    double y_start_goal_dist = goal_conf_base[1] - start_conf_base[1];
//    double theta_start_goal_dist = goal_conf_base[2] - start_conf_base[2];
//    //Minimal distances required between start and goal config to trigger planning
//    double x_min_dist, y_min_dist, theta_min_dist;
//    x_min_dist = 0.1; //10 cm
//    y_min_dist = 0.1; //10 cm
//    theta_min_dist = 0.5236; // 30 degree

//    //Check whether planning is required
//    if (fabs(x_start_goal_dist) < x_min_dist && fabs(y_start_goal_dist) < y_min_dist && fabs(theta_start_goal_dist) < theta_min_dist)
//    {
//        //Trajectory must have at least two points
//        m_result_joint_trajectory.push_back(start_conf_base);
//        m_result_joint_trajectory.push_back(start_conf_base);

//        m_planner_type = "bi_informed_rrt_star";
//        string scenario_name = m_planning_world->getSceneName();

//        //Set path to the file that will store the planned joint trajectory
//        string folder_path = m_planner_package_path + "/data/"+ m_planner_type +"/"+  scenario_name  + "_joint_trajectory_run_0.txt";
//        m_file_path_joint_trajectory = new char[folder_path.size() + 1];
//        copy(folder_path.begin(), folder_path.end(), m_file_path_joint_trajectory);
//        m_file_path_joint_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0
//        //cout<<m_file_path_joint_trajectory<<endl;

//        m_KDLRobotModel->writeTrajectoryToFile(m_result_joint_trajectory,m_file_path_joint_trajectory);

//        //Robot already at target pose (success = true)
//        return true;
//    }

//    //------------ PLANNER INITIALIZATION --------------

//    //Initialize Planner
//    bool init_ok = init_planner(start_conf_base,goal_conf_base,1);

//    //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid


//    //------------ PLANNER PARAMETERIZATION --------------

//    //Set edge cost variable weights (to apply motion preferences)
//    vector<double> edge_cost_weights(3);
//    //Weight for base translation motion
//    edge_cost_weights[0] = 1.0; //base_x motion weight
//    edge_cost_weights[1] = 1.0; //base_y motion weight
//    //Weight for base rotation motion (set higher to get motion with preferably less base rotations)
//    edge_cost_weights[2] = 3.0; //base_theta motion weight
//    setEdgeCostWeights(edge_cost_weights);


//    //------------ RUN PLANNER --------------
//    // Params : PlanningSpace, Iter(0)_or_time(1), max. iterations or time, show_tree, sleep_between_iters, run_number

//    bool success = false;
//    if(init_ok){
//        success = run_planner(1, 1, m_max_planning_time, 1, 0.0, 0);
//    }else{
//        ROS_ERROR("Planner initialization failed!!!");
//    }

//    //------------ RETURN PLANNING RESULT --------------
//    return success;
//}


////Planning for moving the endeffector from the current pose to a target pose
//bool BiRRTstarPlanner::move_endeffector(const Eigen::Affine3d& goal)
//{

//    //------------ WORLD SETUP --------------

//    //TODO:
//    //  -> get size of environment from database or /map topic (see "omnirob_gmapping" package for map file)

//    //World dimensions
//    vector<double> env_size_x(2);
//    env_size_x[0] = -5.0;
//    env_size_x[1] = 5.0;
//    vector<double> env_size_y(2);
//    env_size_y[0] = -5.0;
//    env_size_y[1] = 5.0;
//    setPlanningSceneInfo(env_size_x,env_size_y,"neurobots_demo");


//    //------------ WAIT FOR START CONFIG (get from /joint_states topic) --------------

//    while(!m_lbr_joint_state_received)
//    {
//        ROS_INFO("Current LBR Joint State not available yet");

//        ros::spinOnce();
//    }


//    //------------ GOAL ENDEFFECTOR POSE (from function input) --------------

//    //Get endeffector goal pose from input (in /map frame)
//    double goal_x_map, goal_y_map, goal_z_map, goal_rot_x_map, goal_rot_y_map, goal_rot_z_map;

//    //Get rot_x, rot_y, rot_z
//    Eigen::Matrix3d rot = goal.linear();
//    Eigen::Vector3d angles = rot.eulerAngles(0,1,2);

//    //Set desired x,y,z,rotX,rotY,rotZ for endeffector (in /map frame)
//    goal_x_map = goal.translation().x();
//    goal_y_map = goal.translation().y();
//    goal_z_map = goal.translation().z();
//    goal_rot_x_map = angles(0); //roll
//    goal_rot_y_map = angles(1); //pitch
//    goal_rot_z_map = angles(2); //yaw


//    //Transform from map to endeffector goal frame
//    tf::StampedTransform transform_map_to_ee_goal;
//    transform_map_to_ee_goal.setOrigin(tf::Vector3(goal_x_map, goal_y_map, goal_z_map));
//    transform_map_to_ee_goal.setRotation(tf::createQuaternionFromRPY(goal_rot_x_map, goal_rot_y_map, goal_rot_z_map));


//    //Get current robot arm base pose (in the /map frame)
//    tf::TransformListener listener;
//    tf::StampedTransform transform_map_to_arm_base;
//    try {
//        listener.waitForTransform("/map", "/lbr_0_link", ros::Time(0), ros::Duration(10.0) );
//        listener.lookupTransform("/map", "/lbr_0_link", ros::Time(0), transform_map_to_arm_base);
//    } catch (tf::TransformException ex) {
//        ROS_ERROR("%s",ex.what());
//        ROS_ERROR("Transform /map to /lbr_0_link not available!");
//        return false;
//    }


//    //Transform ee goal from map to robot arm base frame
//    tf::StampedTransform transform_arm_base_to_goal;
//    transform_arm_base_to_goal.mult(transform_map_to_arm_base.inverse(),transform_map_to_ee_goal);


//    //Set Goal (in /lbr_0_link frame)
//    vector<double> goal_pose_endeffector(6);
//    //Position
//    tf::Vector3 goal_trans_ee = transform_arm_base_to_goal.getOrigin();
//    goal_pose_endeffector[0] = goal_trans_ee.x();
//    goal_pose_endeffector[1] = goal_trans_ee.y();
//    goal_pose_endeffector[2] = goal_trans_ee.z();
//    //Orientation
//    tf::Quaternion goal_rot_ee = transform_arm_base_to_goal.getRotation();
//    tf::Matrix3x3 m(goal_rot_ee);
//    m.getRPY(goal_pose_endeffector[3], goal_pose_endeffector[4], goal_pose_endeffector[5]); //Roll, Pitch, Yaw


//    cout <<"Goal endeffector pose in robot arm base frame" << endl;
//    cout << "Position"<< endl;
//    cout <<goal_pose_endeffector[0] << endl;
//    cout <<goal_pose_endeffector[1] << endl;
//    cout <<goal_pose_endeffector[2] << endl;
//    cout << "Orientation"<< endl;
//    cout <<goal_pose_endeffector[3] << endl;
//    cout <<goal_pose_endeffector[4] << endl;
//    cout <<goal_pose_endeffector[5] << endl<<endl;


//    //------------ CONSTRAINT ENDEFFECTOR POSE COORDINATES --------------

//    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
//    constraint_vec_goal_pose[0] = 1; //X
//    constraint_vec_goal_pose[1] = 1; //Y
//    constraint_vec_goal_pose[2] = 1; //Z
//    constraint_vec_goal_pose[3] = 0; //RotX
//    constraint_vec_goal_pose[4] = 0; //RotY
//    constraint_vec_goal_pose[5] = 0; //RotZ


//    //Permitted displacement for ee coordinates w.r.t desired target frame
//    vector<pair<double,double> > target_coordinate_dev(6);
//    target_coordinate_dev[0].first = -0.005;    //negative X deviation [m]
//    target_coordinate_dev[0].second = 0.005;    //positive X deviation
//    target_coordinate_dev[1].first = -0.005;    //negative Y deviation
//    target_coordinate_dev[1].second = 0.005;    //positive Y deviation
//    target_coordinate_dev[2].first = -0.005;    //negative Z deviation
//    target_coordinate_dev[2].second = 0.005;    //positive Z deviation
//    target_coordinate_dev[3].first = -0.05;     //negative Xrot deviation [rad]
//    target_coordinate_dev[3].second = 0.05;     //positive Xrot deviation
//    target_coordinate_dev[4].first = -0.05;     //negative Yrot deviation
//    target_coordinate_dev[4].second = 0.05;     //positive Yrot deviation
//    target_coordinate_dev[5].first = -0.05;     //negative Zrot deviation
//    target_coordinate_dev[5].second = 0.05;     //positive Zrot deviation


//    //------------ PLANNER INITIALIZATION --------------

//    //Initialize Planner
//    //Syntax: init_planner(vector<double> start_conf, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space)
//    bool init_ok = init_planner(m_lbr_joint_state, goal_pose_endeffector, constraint_vec_goal_pose, target_coordinate_dev, 1);

//    //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid


//    //------------ PLANNER PARAMETERIZATION --------------

//    //Set edge cost variable weights (to apply motion preferences)
//    vector<double> edge_cost_weights(7);
//    //Weight for manipulator joints
//    edge_cost_weights[0] = 1.0; //base_x motion weight
//    edge_cost_weights[1] = 1.0; //base_y motion weight
//    edge_cost_weights[2] = 1.0; //base_theta motion weight
//    edge_cost_weights[3] = 1.0; //base_theta motion weight
//    edge_cost_weights[4] = 1.0; //base_theta motion weight
//    edge_cost_weights[5] = 1.0; //base_theta motion weight
//    edge_cost_weights[6] = 1.0; //base_theta motion weight
//    setEdgeCostWeights(edge_cost_weights);


//    //------------ RUN PLANNER --------------
//    // Params : PlanningSpace, Iter(0)_or_time(1), max. iterations or time, show_tree, sleep_between_iters, run_number

//    bool success = false;
//    if(init_ok){
//        success = run_planner(1, 1, m_max_planning_time, 1, 0.0, 0);
//    }else{
//        ROS_ERROR("Planner initialization failed!!!");
//    }

//    //------------ RETURN PLANNING RESULT --------------
//    return success;

//}

//bool BiRRTstarPlanner::move_base_endeffector(const Eigen::Affine3d& goal){

//    while(!m_lbr_joint_state_received)
//    {
//        ROS_INFO("Current LBR Joint State not available yet");

//        ros::spinOnce();
//    }

//    //------------ WORLD SETUP --------------

//    //TODO:
//    //  -> get size of environment from database or /map topic (see "omnirob_gmapping" package for map file)

//    //World dimensions
//    vector<double> env_size_x(2);
//    env_size_x[0] = -5.0;
//    env_size_x[1] = 5.0;
//    vector<double> env_size_y(2);
//    env_size_y[0] = -5.0;
//    env_size_y[1] = 5.0;
//    setPlanningSceneInfo(env_size_x,env_size_y,"neurobots_demo");


//    //------------ START CONFIG (from /joint_states topic) --------------
//    vector<double> start_conf_omnirob_arm(10);

//    //Set base config to zero (i.e. robot is at position of base_link frame given by localization node)
//    start_conf_omnirob_arm[0] = 0.0; //x
//    start_conf_omnirob_arm[1] = 0.0; //y
//    start_conf_omnirob_arm[2] = 0.0; //theta

//    //Get joint config of lbr arm
//    start_conf_omnirob_arm[3] = m_lbr_joint_state[0]; //joint 1
//    start_conf_omnirob_arm[4] = m_lbr_joint_state[1]; //joint 2
//    start_conf_omnirob_arm[5] = m_lbr_joint_state[2]; //joint 3
//    start_conf_omnirob_arm[6] = m_lbr_joint_state[3]; //joint 4
//    start_conf_omnirob_arm[7] = m_lbr_joint_state[4]; //joint 5
//    start_conf_omnirob_arm[8] = m_lbr_joint_state[5]; //joint 6
//    start_conf_omnirob_arm[9] = m_lbr_joint_state[6]; //joint 7


//    //------------ GOAL ENDEFFECTOR POSE (from function input) --------------

//    //Get endeffector goal pose from input (in /map frame)
//    double goal_x_map, goal_y_map, goal_z_map, goal_rot_x_map, goal_rot_y_map, goal_rot_z_map;

//    //Get rot_x, rot_y, rot_z
//    Eigen::Matrix3d rot = goal.linear();
//    Eigen::Vector3d angles = rot.eulerAngles(0,1,2);

//    //Set desired x,y,z,rotX,rotY,rotZ for endeffector (in /map frame)
//    goal_x_map = goal.translation().x();
//    goal_y_map = goal.translation().y();
//    goal_z_map = goal.translation().z();
//    goal_rot_x_map = angles(0); //roll
//    goal_rot_y_map = angles(1); //pitch
//    goal_rot_z_map = angles(2); //yaw


//    //Transform from map to endeffector goal frame
//    tf::StampedTransform transform_map_to_ee_goal;
//    transform_map_to_ee_goal.setOrigin(tf::Vector3(goal_x_map, goal_y_map, goal_z_map));
//    transform_map_to_ee_goal.setRotation(tf::createQuaternionFromRPY(goal_rot_x_map, goal_rot_y_map, goal_rot_z_map));


//    //Get current robot arm base pose (in the /map frame)
//    tf::TransformListener listener;
//    tf::StampedTransform transform_map_to_base;
//    try {
//        listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
//        listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), transform_map_to_base);
//    } catch (tf::TransformException ex) {
//        ROS_ERROR("%s",ex.what());
//        ROS_ERROR("Transform /map to /base_link not available!");
//        return false;
//    }


//    //Transform from /base_link frame to goal ee frame
//    tf::StampedTransform transform_base_to_ee_goal;
//    transform_base_to_ee_goal.mult(transform_map_to_base.inverse(),transform_map_to_ee_goal);


//    //Set Goal (in /base_link frame)
//    vector<double> goal_pose_endeffector(6);
//    //Position
//    tf::Vector3 goal_trans_ee = transform_base_to_ee_goal.getOrigin();
//    goal_pose_endeffector[0] = goal_trans_ee.x();
//    goal_pose_endeffector[1] = goal_trans_ee.y();
//    goal_pose_endeffector[2] = goal_trans_ee.z();
//    //Orientation
//    tf::Quaternion goal_rot_ee = transform_base_to_ee_goal.getRotation();
//    tf::Matrix3x3 m(goal_rot_ee);
//    m.getRPY(goal_pose_endeffector[3], goal_pose_endeffector[4], goal_pose_endeffector[5]); //Roll, Pitch, Yaw


//    cout <<"Goal endeffector pose in robot base link frame" << endl;
//    cout << "Position"<< endl;
//    cout <<goal_pose_endeffector[0] << endl;
//    cout <<goal_pose_endeffector[1] << endl;
//    cout <<goal_pose_endeffector[2] << endl;
//    cout << "Orientation"<< endl;
//    cout <<goal_pose_endeffector[3] << endl;
//    cout <<goal_pose_endeffector[4] << endl;
//    cout <<goal_pose_endeffector[5] << endl<<endl;


//    //------------ CONSTRAINT ENDEFFECTOR POSE COORDINATES --------------

//    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
//    constraint_vec_goal_pose[0] = 1; //X
//    constraint_vec_goal_pose[1] = 1; //Y
//    constraint_vec_goal_pose[2] = 1; //Z
//    constraint_vec_goal_pose[3] = 0; //RotX
//    constraint_vec_goal_pose[4] = 0; //RotY
//    constraint_vec_goal_pose[5] = 0; //RotZ


//    //Permitted displacement for ee coordinates w.r.t desired target frame
//    vector<pair<double,double> > target_coordinate_dev(6);
//    target_coordinate_dev[0].first = -0.005;    //negative X deviation [m]
//    target_coordinate_dev[0].second = 0.005;    //positive X deviation
//    target_coordinate_dev[1].first = -0.005;    //negative Y deviation
//    target_coordinate_dev[1].second = 0.005;    //positive Y deviation
//    target_coordinate_dev[2].first = -0.005;    //negative Z deviation
//    target_coordinate_dev[2].second = 0.005;    //positive Z deviation
//    target_coordinate_dev[3].first = -0.05;     //negative Xrot deviation [rad]
//    target_coordinate_dev[3].second = 0.05;     //positive Xrot deviation
//    target_coordinate_dev[4].first = -0.05;     //negative Yrot deviation
//    target_coordinate_dev[4].second = 0.05;     //positive Yrot deviation
//    target_coordinate_dev[5].first = -0.05;     //negative Zrot deviation
//    target_coordinate_dev[5].second = 0.05;     //positive Zrot deviation


//    //------------ PLANNER INITIALIZATION --------------

//    //Initialize Planner
//    //Syntax: init_planner(vector<double> start_conf, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space)
//    bool init_ok = init_planner(start_conf_omnirob_arm, goal_pose_endeffector, constraint_vec_goal_pose, target_coordinate_dev, 1);

//    //Remark: Fails is start or goal config (start_conf_base,goal_conf_base) is invalid


//    //------------ PLANNER PARAMETERIZATION --------------

//    //Set edge cost variable weights (to apply motion preferences)
//    vector<double> edge_cost_weights(10);
//    //Weight for base joints
//    edge_cost_weights[0] = 1.0; //base_x motion weight
//    edge_cost_weights[1] = 1.0; //base_y motion weight
//    edge_cost_weights[2] = 3.0; //base_theta motion weight
//    //Weight for lbr joints
//    edge_cost_weights[3] = 1.0; //lbr joint 1 weight
//    edge_cost_weights[4] = 1.0; //lbr joint 2 weight
//    edge_cost_weights[5] = 1.0; //lbr joint 3 weight
//    edge_cost_weights[6] = 1.0; //lbr joint 4 weight
//    edge_cost_weights[7] = 1.0; //lbr joint 5 weight
//    edge_cost_weights[8] = 1.0; //lbr joint 6 weight
//    edge_cost_weights[9] = 1.0; //lbr joint 7 weight
//    setEdgeCostWeights(edge_cost_weights);



//    //------------ RUN PLANNER --------------
//    // Params : PlanningSpace, Iter(0)_or_time(1), max. iterations or time, show_tree, sleep_between_iters, run_number

//    bool success = false;
//    if(init_ok){
//        success = run_planner(1, 1, m_max_planning_time, 1, 0.0, 0);
//    }else{
//        ROS_ERROR("Planner initialization failed!!!");
//    }

//    //------------ RETURN PLANNING RESULT --------------
//    return success;

//}



//Compute pose of endeffector in start configuration
vector<double> BiRRTstarPlanner::computeEEPose(vector<double> start_conf)
{
    //Convert start configuration into KDL Joint Array
    KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(start_conf);

//    start_configuration = KDL::JntArray(start_conf.size());
//    for (int i = 0; i < start_conf.size(); i++)
//        start_configuration(i) = start_conf[i];

    //Compute endeffector pose for start node
    vector<double> ee_start_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain, start_configuration);

    return ee_start_pose;
}


//Compute pose of endeffector in start configuration
vector<double> BiRRTstarPlanner::computeEEPose(KDL::JntArray start_conf)
{
    //Compute endeffector pose for start node
    vector<double> ee_start_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain, start_conf);

    return ee_start_pose;
}



//Compute IK for given endeffector goal pose
vector<double> BiRRTstarPlanner::findIKSolution(vector<double> goal_ee_pose, vector<int> constraint_vec, vector<pair<double,double> > coordinate_dev, bool show_motion)
{
    //Init configuration validity
    bool collision_free = false;

    //Set Variable Constraint Vector (constraint variable and don't care variables)
    // -> Considered in error computation of control loop
    m_RobotMotionController->setVariableConstraints(constraint_vec, coordinate_dev);


    //Run numerical IK Solver
    vector<double> ik_solution;
    while (collision_free == false)
    {

        //Output joint and endeffector trajectory from controller
        vector<vector<double> > joint_trajectory;
        vector<vector<double> > ee_trajectory;


        //Get  random start config
        vector<double> conf_init = sampleJointConfig_Vector();

        //Set new random start config
        m_RobotMotionController->setStartConf(conf_init);


        //Set ee goal pose for numerical IK solver
        //Note: "set_EE_goal_pose" needs to be called after "setStartConf" since this function also initializes the initial error
        m_RobotMotionController->set_EE_goal_pose(goal_ee_pose);

        //control loop execution
        kuka_motion_controller::Status connector_state = m_RobotMotionController->run_VDLS_Control_Connector(1000, joint_trajectory,ee_trajectory,show_motion,show_motion);

        //Set last configuration of joint trajectory as goal config / ik solution
        if(0 < joint_trajectory.size())
            ik_solution = joint_trajectory[joint_trajectory.size()-1];
        else
        {
            ROS_ERROR("Something went wrong in generating the IK solution with the VDLS_Control_Connector");
        }

        //Perform collision check when pose has been reached
        if(connector_state == kuka_motion_controller::REACHED)
        {
            cout<<"EE Pose reached ..."<<endl;

            //Check is it is collision free
            if(m_FeasibilityChecker->isConfigValid(ik_solution,true))
            {
                collision_free = true;
                cout<<"... and valid!"<<endl;
            }
            else
                cout<<"... but invalid!"<<endl;
        }

    }
    
    //Return last configuration of joint trajectory (placing the EE at the goal_ee_pose)
    return ik_solution;
}




//Compute IK for given endeffector goal pose (given a specific initial config)
vector<double> BiRRTstarPlanner::findIKSolution(vector<double> goal_ee_pose, vector<int> constraint_vec, vector<pair<double,double> > coordinate_dev, vector<double> mean_init_config, bool show_motion)
{

    //Init configuration validity
    bool collision_free = false;

    //Set Variable Constraint Vector (constraint variable and don't care variables)
    // -> Considered in error computation of control loop
    m_RobotMotionController->setVariableConstraints(constraint_vec,coordinate_dev);

    //Set standard deviation for gaussian sampling around mean config values
    double sampling_stddev = 0.05;

    //Run numerical IK Solver
    vector<double> ik_solution;
    while (collision_free == false)
    {
        //Output joint and endeffector trajectory from controller
        vector<vector<double> > joint_trajectory;
        vector<vector<double> > ee_trajectory;

        //Get  random start config
        vector<double> conf_init = sampleJointConfig_Vector(mean_init_config, sampling_stddev);


//        //Set the desired variables for initial config
//        for(int i = 0; i < conf_init.size() ; i++)
//        {
//            //If current variable is not "Don't care"
//            if(init_config[i] != 1000.0)
//                conf_init[i] = init_config[i];
//        }

        //Set new random start config
        m_RobotMotionController->setStartConf(conf_init);

        //Set ee goal pose for numerical IK solver
        //Note: "set_EE_goal_pose" needs to be called after "setStartConf" since this function also initializes the initial error
        m_RobotMotionController->set_EE_goal_pose(goal_ee_pose);

        //control loop execution
        kuka_motion_controller::Status connector_state = m_RobotMotionController->run_VDLS_Control_Connector(1000, joint_trajectory,ee_trajectory,show_motion,show_motion);

        //Set last configuration of joint trajectory as goal config / ik solution
        if(0 < joint_trajectory.size())
            ik_solution = joint_trajectory[joint_trajectory.size()-1];
        else
        {
            ROS_ERROR("Something went wrong in generating the IK solution with the VDLS_Control_Connector");
        }

        //Perform collision check when pose has been reached
        if(connector_state == kuka_motion_controller::REACHED)
        {
            cout<<"EE Pose reached ..."<<endl;

            //Check is it is collision free
            if(m_FeasibilityChecker->isConfigValid(ik_solution,true))
            {
                collision_free = true;
                cout<<"... and valid!"<<endl;
            }
            else
                cout<<"... but invalid!"<<endl;
        }
        else
        {
            cout<<"Failed to reach EE Pose!"<<endl;
        }

    }

    //Return last configuration of joint trajectory (placing the EE at the goal_ee_pose)
    return ik_solution;
}


//Generate a configuration for a given EE pose
vector<double> BiRRTstarPlanner::generate_config_from_ee_pose(vector<double> ee_pose, vector<int> constraint_vec_ee_pose, vector<pair<double,double> > coordinate_dev, bool show_motion)
{
    //Convert XYZ euler orientation of start and goal pose to quaternion
    vector<double> quat_ee_pose = m_RobotMotionController->convertEulertoQuat(ee_pose[3],ee_pose[4],ee_pose[5]);

    //Set up ee pose with orientation expressed by quaternion
    vector<double> ee_pose_quat_orient (7);
    ee_pose_quat_orient[0] = ee_pose[0]; //x
    ee_pose_quat_orient[1] = ee_pose[1]; //y
    ee_pose_quat_orient[2] = ee_pose[2]; //z
    ee_pose_quat_orient[3] = quat_ee_pose[0];  //quat_x
    ee_pose_quat_orient[4] = quat_ee_pose[1];  //quat_y
    ee_pose_quat_orient[5] = quat_ee_pose[2];  //quat_z
    ee_pose_quat_orient[6] = quat_ee_pose[3];  //quat_w

    //Find configuration for the endeffector pose
    vector<double> robot_config = findIKSolution(ee_pose_quat_orient, constraint_vec_ee_pose, coordinate_dev, show_motion);

    //Return start and goal config
    return robot_config;
}


//Generate a configuration for a given EE pose using a reference config
vector<double> BiRRTstarPlanner::generate_config_from_ee_pose_with_reference_config(vector<double> ee_pose, vector<int> constraint_vec_ee_pose, vector<pair<double,double> > coordinate_dev, vector<double> mean_init_config, bool show_motion)
{
    //Convert XYZ euler orientation of start and goal pose to quaternion
    vector<double> quat_ee_pose = m_RobotMotionController->convertEulertoQuat(ee_pose[3],ee_pose[4],ee_pose[5]);

    //Set up ee pose with orientation expressed by quaternion
    vector<double> ee_pose_quat_orient (7);
    ee_pose_quat_orient[0] = ee_pose[0]; //x
    ee_pose_quat_orient[1] = ee_pose[1]; //y
    ee_pose_quat_orient[2] = ee_pose[2]; //z
    ee_pose_quat_orient[3] = quat_ee_pose[0];  //quat_x
    ee_pose_quat_orient[4] = quat_ee_pose[1];  //quat_y
    ee_pose_quat_orient[5] = quat_ee_pose[2];  //quat_z
    ee_pose_quat_orient[6] = quat_ee_pose[3];  //quat_w

    //Find configuration for endeffector goal pose (using ik_sol_ee_start_pose as mean config for init_config sampling)
    vector<double> robot_config = findIKSolution(ee_pose_quat_orient, constraint_vec_ee_pose, coordinate_dev, mean_init_config, show_motion);

    //Return start and goal config
    return robot_config;
}


//Generate start and goal configuration for a given start and goal EE pose
vector<vector<double> > BiRRTstarPlanner::generate_start_goal_config(vector<double> start_pose, vector<int> constraint_vec_start_pose, vector<double> goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, bool show_motion )
{
    //Return value [0] = start config, [1] = start config
    vector<vector<double> > start_goal_config(2);

    //------------ Set file path ------------

    //Set path to the file that will store the start and goal config for each scenario
    char* file_path_start_goal_config;
    string folder_path = m_terminal_configs_path + "/Start_Goal_Configurations/"+ m_planning_group + "_" + m_planning_world->getSceneName() + "_start_goal_config.txt";
    file_path_start_goal_config = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_start_goal_config);
    file_path_start_goal_config[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_start_goal_config<<endl;

    // -------------------- Generate Star and Goal Config ----------------------------

    //Convert XYZ euler orientation of start and goal pose to quaternion
    vector<double> quat_start_pose = m_RobotMotionController->convertEulertoQuat(start_pose[3],start_pose[4],start_pose[5]);
    vector<double> quat_goal_pose = m_RobotMotionController->convertEulertoQuat(goal_pose[3],goal_pose[4],goal_pose[5]);

    //Set up start pose with orientation expressed by quaternion
    vector<double> start_ee_pose_quat_orient (7);
    start_ee_pose_quat_orient[0] = start_pose[0]; //x
    start_ee_pose_quat_orient[1] = start_pose[1]; //y
    start_ee_pose_quat_orient[2] = start_pose[2]; //z
    start_ee_pose_quat_orient[3] = quat_start_pose[0];  //quat_x
    start_ee_pose_quat_orient[4] = quat_start_pose[1];  //quat_y
    start_ee_pose_quat_orient[5] = quat_start_pose[2];  //quat_z
    start_ee_pose_quat_orient[6] = quat_start_pose[3];  //quat_w
    //Set up goal pose with orientation expressed by quaternion
    vector<double> goal_ee_pose_quat_orient (7);
    goal_ee_pose_quat_orient[0] = goal_pose[0]; //x
    goal_ee_pose_quat_orient[1] = goal_pose[1]; //y
    goal_ee_pose_quat_orient[2] = goal_pose[2]; //z
    goal_ee_pose_quat_orient[3] = quat_goal_pose[0];  //quat_x
    goal_ee_pose_quat_orient[4] = quat_goal_pose[1];  //quat_y
    goal_ee_pose_quat_orient[5] = quat_goal_pose[2];  //quat_z
    goal_ee_pose_quat_orient[6] = quat_goal_pose[3];  //quat_w




    //Find configuration for endeffector goal pose (using ik_sol_ee_start_pose as mean config for init_config sampling)
    //Note: -> Gaussian sampling for initial config is only performed for revolute joints
    //      -> Initial values for prismatic joint are sampled uniformly from their respective joint range)
    start_goal_config[1] = findIKSolution(goal_ee_pose_quat_orient, constraint_vec_goal_pose, coordinate_dev, show_motion);

    //Find configuration for endeffector start pose
    start_goal_config[0] = findIKSolution(start_ee_pose_quat_orient, constraint_vec_start_pose, coordinate_dev, start_goal_config[1], show_motion);


    //Write the start and goal configuration to file
    writeStartGoalConfig(file_path_start_goal_config,start_goal_config[0], start_goal_config[1]);

    //Return start and goal config
    return start_goal_config;

}




//Activate Tree Optimization Features
void BiRRTstarPlanner::activateTreeOptimization()
{
    m_tree_optimization_active = true;
}

//Deactivate Tree Optimization Features
void BiRRTstarPlanner::deactivateTreeOptimization()
{
    m_tree_optimization_active = false;
}


//Activate Informed Sampling Heuristic Features
void BiRRTstarPlanner::activateInformedSampling()
{
    m_informed_sampling_active = true;
}

//Deactivate Informed Sampling Heuristic Features
void BiRRTstarPlanner::deactivateInformedSampling()
{
    m_informed_sampling_active = false;
}



//Tree expansions step
bool BiRRTstarPlanner::expandTree(Rrt_star_tree *tree, Node nn_node, Node x_rand, Node &x_new, Edge &e_new, bool show_tree_vis)
{
    //Flag indicating whether a new edge from the nearest node towards x_rand has been found
    bool tree_extend_NN = false;

    //If Endeffector Constraints are Active
    if(m_constraint_active)
    {
        //-----------------------------------------------------------------------------

        //Generated node and edge by connectNodesInterpolation
        Node gen_node;
        Edge gen_edge;

        // Selected Via Nodes and Edges to x_rand
        vector<Node> selected_via_nodes;
        vector<Edge> selected_via_edges;

        //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while expading towards x_rand)
        int num_nodes_tree = tree->num_nodes;
        //Get current number of nodes in the tree (required to avoid incrementing the number of edges while expading towards x_rand)
        int num_edges_tree = tree->num_edges;

        //Is task frame position defined global or w.r.t near node frame
        if(m_task_pos_global == false)
        {
            //Set Task frame position to position of nearest neighbour ee pose
            m_task_frame.p.x(nn_node.ee_pose[0]); //Set X Position
            m_task_frame.p.y(nn_node.ee_pose[1]); //Set Y Position
            m_task_frame.p.z(nn_node.ee_pose[2]); //Set Z Position
        }
        //Is task frame orientation defined global or w.r.t near node frame
        if(m_task_orient_global == false)
        {
            KDL::Rotation ee_orient = KDL::Rotation::Quaternion(nn_node.ee_pose[3],nn_node.ee_pose[4],nn_node.ee_pose[5],nn_node.ee_pose[6]);
            m_task_frame.M = ee_orient;
        }


        //Variable for timer
        //gettimeofday(&m_timer, NULL);
        //double time_projection_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

        //Project original random sample onto constraint manifold
        bool projection_x_rand = m_RobotMotionController->run_config_retraction(x_rand.config, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev, m_max_projection_iter);

        //Get time elapsed
        //gettimeofday(&m_timer, NULL);
        //double time_projection_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
        //cout<<"Time Projection: "<<(time_projection_end - time_projection_start)<<endl;


        bool rand_sample_reached = false;
        while(rand_sample_reached == false)
        {
            //Perform a step from nn_node towards x_rand using a fixed step size (returns true when x_rand has been reached by expansion)
            Node exp_node_towards_rand_node = x_rand;
            rand_sample_reached = stepTowardsRandSample(nn_node, exp_node_towards_rand_node, m_constraint_extend_step_factor);

            //If x_rand has NOT been reached from current near node nn_node
            if(rand_sample_reached == false)
            {

                //Is task frame position defined global or w.r.t near node frame
                if(m_task_pos_global == false)
                {
                    //Set Task frame position to position of nearest neighbour ee pose
                    m_task_frame.p.x(nn_node.ee_pose[0]); //Set X Position
                    m_task_frame.p.y(nn_node.ee_pose[1]); //Set Y Position
                    m_task_frame.p.z(nn_node.ee_pose[2]); //Set Z Position
                }
                //Is task frame orientation defined global or w.r.t near node frame
                if(m_task_orient_global == false)
                {
                    KDL::Rotation ee_orient = KDL::Rotation::Quaternion(nn_node.ee_pose[3],nn_node.ee_pose[4],nn_node.ee_pose[5],nn_node.ee_pose[6]);
                    m_task_frame.M = ee_orient;
                }

                //Project sample onto constraint manifold
                bool projection_succeed = m_RobotMotionController->run_config_retraction(exp_node_towards_rand_node.config, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev, m_max_projection_iter);

                if(projection_succeed == true && m_FeasibilityChecker->isConfigValid(exp_node_towards_rand_node.config))
                {

                    //Check if config retraction has projected the expanded config back onto the config of the near node "nn_node"
                    // -> In this case the expansion is stuck, because each projected sample/config will be identical to the config of "nn_node"
                    double dist_near_to_projected_sample = m_Heuristic.euclidean_joint_space_distance(nn_node.config,exp_node_towards_rand_node.config);

                    //Check additionally whether expansion has made progress towards x_rand
                    // 1) checking whether sample has been projected back onto near sample (here nn_node.config)
                    // 2) checking whether projected sample has made progress towards target node (here x_rand.config)
                    double dist_x_near_to_x_rand = m_Heuristic.euclidean_joint_space_distance(nn_node.config,x_rand.config);
                    double dist_projected_sample_to_x_new = m_Heuristic.euclidean_joint_space_distance(exp_node_towards_rand_node.config,x_rand.config);

                    if(dist_near_to_projected_sample < m_min_projection_distance || dist_x_near_to_x_rand < dist_projected_sample_to_x_new)
                    {
                        //if(dist_near_to_projected_sample < m_min_projection_distance)
                        //    cout<<"expandTree: Sample projected back onto config of near node!"<<endl;
                        //else
                        //    cout<<"expandTree: Projected sample is further away from x_rand than x_near!"<<endl;

                        break;
                    }

                    //Connect nn_node to projected sample
                    bool connection_result = connectNodesInterpolation(tree, nn_node, exp_node_towards_rand_node, m_num_traj_segments_interp, gen_node, gen_edge);


                    //Check edge from current nn_node to exp_node_towards_rand_node for validity (EE constraints + collision check)
                    if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                    {

                        //Modify node data
                        gen_node.node_id = num_nodes_tree; //reset node ID using local nodes counter
                        num_nodes_tree++;
                        //Set near vertex as parent of gen_node
                        gen_node.parent_id = nn_node.node_id;

                        //Modify edge data
                        gen_edge.edge_id = num_edges_tree; //reset edge ID using local edge counter
                        num_edges_tree++;
                        //Reset edge child ID, required because exp_node_towards_rand_node is ID of x_rand
                        gen_edge.child_node_id = gen_node.node_id;

                        //Collect current expanded node and edge (later added to the tree)
                        selected_via_nodes.push_back(gen_node);
                        selected_via_edges.push_back(gen_edge);

                        //Update nn_node in order to step further towards x_rand
                        nn_node = gen_node;

                        //Set flag indicating that the tree has been expanded
                        tree_extend_NN = true;

                        //Leave loop after a single extension when ...
                        // -> ... the projection of the original x_rand failed : this is done because original x_rand is impossible to reach (it is not on the constraint manifold)
                        // -> ... the m_single_extend_step flag is set, indicating that single step expansions are desired
                        if(projection_x_rand == false || m_single_extend_step == true)
                            break;
                    }
                    else
                    {
                       //Expansion towards exp_node_towards_rand_node is invalid
                       break;
                    }
                 }
                 else
                 {
                    //Projection of / or config of exp_node_towards_rand_node is invalid
                    break;
                 }
            }
            else
            {
                //Connect current nn_node to x_rand
                bool connection_result = connectNodesInterpolation(tree, nn_node, x_rand, m_num_traj_segments_interp, gen_node, gen_edge);

                //Check edge from x_connect to exp_node_towards_rand_node for validity (collision check)
                if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                {
                    //Modify node data
                    gen_node.node_id = num_nodes_tree; //reset node ID using local nodes counter
                    //Set near vertex as parent of gen_node
                    gen_node.parent_id = nn_node.node_id;

                    //Modify edge data
                    gen_edge.edge_id = num_edges_tree; //reset edge ID using local edge counter
                    //Reset edge child ID, required because exp_node_towards_rand_node is ID of x_rand
                    gen_edge.child_node_id = gen_node.node_id;

                    //Collect current expanded node and edge (later added to the tree)
                    selected_via_nodes.push_back(gen_node);
                    selected_via_edges.push_back(gen_edge);

                    //Set flag indicating that the tree has been expanded
                    tree_extend_NN = true;


                }//Edge validity check

            }//Query Sample Reached FALSE

        }//END Loop While Sample Not Reached


        //Set x_new to x_rand when extension has failed (allowing to connect other near nodes to the new node)
        if(selected_via_nodes.size() == 0)
            x_new = x_rand;
        else
        {
            //Insert via nodes and edges growing the tree towards x_rand into tree
            // -> Note: Final node and edge will be added to tree in the main loop (see "run_planner" function)
            for(int i = 0 ; i < selected_via_nodes.size() ; i++)
            {
                //Set terminal node and egde (added in planner main loop)
                if( i == selected_via_nodes.size()-1)
                {
                    //Set new node data
                    x_new = selected_via_nodes[i];
                    //Set new edge data
                    e_new = selected_via_edges[i];
                }
                else //Insert via nodes and edges
                {
                    insertNode(tree, selected_via_edges[i] , selected_via_nodes[i], show_tree_vis);
                }
            }
        }

    } //end constraint extension
    else //Unconstraint motion planning
    {

        if(m_single_extend_step == true)
        {
            //Copy x_rand, because extend_node will be modified in the following
            Node extend_node;
            extend_node.config = x_rand.config;

            //Perform a step from current near node towards last valid config of edge using a fixed step size
            stepTowardsRandSample(nn_node, extend_node, m_unconstraint_extend_step_factor);

            //Connect Nodes (by interpolating configurations of nn_node and current x_rand)
            connectNodesInterpolation(tree, nn_node, extend_node, m_num_traj_segments_interp, x_new, e_new);

            //Set flag to true if edge between nn_node and extend_node is collision-free
            tree_extend_NN = m_FeasibilityChecker->isEdgeValid(e_new);

            if(tree_extend_NN)
            {
                //Modify node data
                x_new.node_id = tree->num_nodes;
                //Set near vertex as parent of x_new
                //x_new.parent_id = nn_node.node_id;

                //Modify edge data
                e_new.edge_id = tree->num_edges;
                //Reset child node ID of edge
                e_new.child_node_id = x_new.node_id;
            }
            else
            {
                //Set x_new to x_rand when extension has failed (allowing to connect other near nodes to the new node)
                x_new = x_rand;
            }
        }
        else
        {
            //Connect Nodes (by interpolating configurations of x_rand and nearest neighbour)
            connectNodesInterpolation(tree, nn_node, x_rand, m_num_traj_segments_interp, x_new, e_new);

            //Index of last valid node (when edge is invalid)
            int index_last_valid_node = 0;
            //Set flag to true if via nodes between nn_node and x_rand are collision-free
            m_FeasibilityChecker->isEdgeValid(e_new, index_last_valid_node);

            //Add via edges and nodes to the last valid config along the edge connecting nn_node and x_rand
            // Note: Last valid Node can be also x_rand itself (i.e. entire edge is valid)
            if(index_last_valid_node != 0)
            {
                //Tree extension from nearest neighbour succeeded
                tree_extend_NN = true;

                //Get last valid config along edge trajectory
                Node extend_node;
                extend_node.config = e_new.joint_trajectory[index_last_valid_node];

                bool rand_sample_reached = false;
                while(rand_sample_reached == false)
                {
                    //Perform a step from current near node towards last valid config of edge using a fixed step size
                    Node original_extend_node = extend_node; //copy extend_node, because original_extend_node will be modified in the following
                    rand_sample_reached = stepTowardsRandSample(nn_node, original_extend_node, m_unconstraint_extend_step_factor);

                    //Connect Nodes (by interpolating configurations of nn_node and current x_rand)
                    connectNodesInterpolation(tree, nn_node, original_extend_node, m_num_traj_segments_interp, x_new, e_new);

                    //Modify node data
                    x_new.node_id = tree->num_nodes;
                    //Set near vertex as parent of x_new
                    //x_new.parent_id = nn_node.node_id;

                    //Modify edge data
                    e_new.edge_id = tree->num_edges;
                    //Reset child node ID of edge
                    e_new.child_node_id = x_new.node_id;

                    //Perform only one tree extension if flag is set
                    if(m_single_extend_step == true)
                    {
                        //rand_sample_reached = true;
                        //tree_extend_NN = true;
                        break;
                    }

                    if(rand_sample_reached == false)
                    {
                        //Insert via node and edge
                        insertNode(tree,e_new,x_new,show_tree_vis);

                        //Update x_connect in order to step further towards x_new
                        nn_node = x_new;
                    }

                }//END while rand_sample_reached == false

            }//END if last valid config unequal start config of edge
            else
            {
                //Tree extension from nearest neighbour failed
                tree_extend_NN = false;

                //Set x_new to x_rand when extension has failed (allowing to connect other near nodes to the new node)
                x_new = x_rand;
            }
        }// end m_single_extend_step == true


    } //END query constraint active else


    //Return whether tree expansion has succeeded
    return tree_extend_NN;

}


//Try to connect the two RRT* Trees given the new node added to "tree_A" and its nearest neighbour in "tree_B"
void BiRRTstarPlanner::connectGraphsControl(Rrt_star_tree *tree, Node x_connect, Node x_new, bool show_tree_vis)
{
    //Flag indicating whether the tree has been only expanded towards the other tree or has connected to it
    bool tree_expand = false;

    //Step with for EXTEND backwards search
    int extend_step_width = 100;

    //Best Theoretical solution cost from near vertex
    double best_near_vertex_sol_cost = 10000.0;

    //Selected node and edge
    Node node_selected;
    Edge edge_selected;

    //Cost of potential solution path
    double cost_solution_path = m_cost_best_solution_path;

    //Generated node and edge by connectNodesInterpolation
    Node gen_node;
    Edge gen_edge;
    kuka_motion_controller::Status connector_result = connectNodesControl(tree, x_connect, x_new, gen_node, gen_edge);

    // Notes:
    // x_connect if from tree_B
    // x_new if from tree_A
    // gen_node.cost_reach.total = x_connect.cost_reach.total + edge cost to x_new (cost expressed w.r.t tree_B)
    // x_new.cost_reach.total = cost to reach node x_new (expressed w.r.t tree_A)

    double solution_cost = 0.0;
    if(connector_result == kuka_motion_controller::REACHED)
    {
        //Compute solution path cost
        solution_cost = gen_node.cost_reach.total + x_new.cost_reach.total;

        //Check validity of edge connecting nn_ndoe to x_rand
        if(solution_cost < cost_solution_path)
        {
            if(m_FeasibilityChecker->isEdgeValid(gen_edge))
            {
                //Set solution path cost (will be probably improved by on of the near nodes, see code below)
                cost_solution_path = solution_cost;

                //Set Node ID
                node_selected = gen_node;
                node_selected.node_id = tree->num_nodes;
                //Set near vertex as parent of gen_node
                node_selected.parent_id = x_connect.node_id;

                //Set selected edge
                edge_selected = gen_edge;
                edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                //Set flag indicating that the tree has established a connection to the other tree (not only expanded)
                tree_expand = false;

            }
            else
            {
                //TODO choose last valid node along edge as new node
                // -> Split edge into X subnodes
                // -> Try to connect x_connect to subnodes
                // -> When an subedge is valid set node_selected and edge_selected
                // ->

                //Node for connect extension (without reaching other tree)
                Node extend_node;

                //Generated node and edge by connectNodesInterpolation
                Node ext_node;
                Edge ext_edge;

                //Go backwards from penultimate edge point
                for (int i = 1 ; i < gen_edge.ee_trajectory.size()-1 ; i+=extend_step_width)
                {
                    //Get configuration from the invalid edge
                    extend_node.ee_pose = gen_edge.ee_trajectory[gen_edge.ee_trajectory.size()-1-i];
                    //Try to connect "x_connect" to current ee pose of "extend_node" by a valid edge
                    kuka_motion_controller::Status extend_result = connectNodesControl(tree, x_connect, extend_node, ext_node, ext_edge);

                    if(extend_result == kuka_motion_controller::REACHED)
                    {
                        if(m_FeasibilityChecker->isEdgeValid(ext_edge))
                        {
                            //Set Node ID
                            node_selected = ext_node;
                            node_selected.node_id = tree->num_nodes;
                            //Set near vertex as parent of ext_node
                            node_selected.parent_id = x_connect.node_id;

                            //Set selected edge
                            edge_selected = ext_edge;
                            edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                            //Set flag indicating that the tree has been expanded without connecting to the other tree
                            tree_expand = true;

                            //Set best Theoretical solution cost from nearest neighbour
                            best_near_vertex_sol_cost = solution_cost;

                            //Leave the loop when a valid edge has been found
                            break;
                        }

                    }//END of extend_result == kuka_motion_controller::REACHED
                } //End of backwards loop
            } //End of extend step
        } //End solution cost check
    } //END of connector_result == kuka_motion_controller::REACHED



    if(m_solution_path_available == true)
    {
        //Find the set of vertices in the vicinity of x_new
        vector<int> near_nodes = find_near_vertices_control(tree, x_new);


        //Choose Parent for Node (from tree_B), i.e. the one providing the lowest cost for reaching x_new
        for (int nv = 0; nv < near_nodes.size() ; nv++)
        {
           //Try to connect a near vertex to x_new
           kuka_motion_controller::Status connector_result = connectNodesControl(tree, tree->nodes[near_nodes[nv]], x_new, gen_node, gen_edge);

           double solution_cost = 0.0;
           if(connector_result == kuka_motion_controller::REACHED)
           {
               //Compute overall solution cost, i.e. cost of reaching x_new via near node (from tree_B) + cost of reaching x_new (from tree_A)
               solution_cost = gen_node.cost_reach.total + x_new.cost_reach.total;

               //If cost is smaller than the cost of reaching x_new via it's nearest neighbour
               if(solution_cost < cost_solution_path)
                {
                    //Only if edge to x_new is valid
                    if(m_FeasibilityChecker->isEdgeValid(gen_edge))
                    {
                        //Update solution path cost
                        cost_solution_path = solution_cost;

                        //Update Selected Node ID
                        node_selected = gen_node;
                        node_selected.node_id = tree->num_nodes;
                        //Set near vertex as parent of gen_node
                        node_selected.parent_id = tree->nodes[near_nodes[nv]].node_id;

                        //Set selected edge
                        edge_selected = gen_edge;
                        edge_selected.child_node_id = node_selected.node_id; //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                        //Set flag indicating that the tree has established a connection to the other tree (not only expanded)
                        tree_expand = false;

                    }
                    else
                    {
                        //Check if a near vertex has established a connection to the other tree (if yes, "cost_solution_path" will be unequal "m_cost_best_solution_path")
                        // -> thus partial extend will be unnecessary
                        if (cost_solution_path == m_cost_best_solution_path && solution_cost < best_near_vertex_sol_cost)
                        {
                            //Node for connect extension (without reaching other tree)
                            Node extend_node;

                            //Generated node and edge by connectNodesInterpolation
                            Node ext_node;
                            Edge ext_edge;

                            //Go backwards from penultimate edge point
                            for (int i = 1 ; i < gen_edge.ee_trajectory.size()-1 ; i+=extend_step_width)
                            {
                                //Get configuration from the invalid edge
                                extend_node.ee_pose = gen_edge.ee_trajectory[gen_edge.ee_trajectory.size()-1-i];
                                //Try to connect "x_connect" to current ee pose of "extend_node" by a valid edge
                                kuka_motion_controller::Status extend_result = connectNodesControl(tree, tree->nodes[near_nodes[nv]], extend_node, ext_node, ext_edge);

                                if(extend_result == kuka_motion_controller::REACHED)
                                {
                                    if(m_FeasibilityChecker->isEdgeValid(ext_edge))
                                    {
                                        //Set Node ID
                                        node_selected = ext_node;
                                        node_selected.node_id = tree->num_nodes;
                                        //Set near vertex as parent of ext_node
                                        node_selected.parent_id = near_nodes[nv];

                                        //Set selected edge
                                        edge_selected = ext_edge;
                                        edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                                        //Set flag indicating that the tree has been expanded without connecting to the other tree
                                        tree_expand = true;

                                        //Set best Theoretical solution cost from nearest neighbour
                                        best_near_vertex_sol_cost = solution_cost;

                                        //Leave the loop when a valid edge has been found
                                        break;
                                    }
                                }//END of extend_result == kuka_motion_controller::REACHED
                            }//End of backwards loop
                        } // END Check if a near vertex has established a connection to the other tree and whether the cost from that node to x_new is theoretically lower
                    }//End of extend step

                } //End Reduced cost test
           }//End of connector_result == kuka_motion_controller::REACHED
        } //End iteration through near vertices
    }//End if solution path is available




    //Check whether solution path cost of connected RRT* trees has been improved
    if(cost_solution_path < m_cost_best_solution_path)
    {
        //Store time to find first solution (afterwards "m_solution_path_available" remains always true)
        if (m_solution_path_available == false)
        {
            //Get time elapsed
            gettimeofday(&m_timer, NULL);
            double sol_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
            //printf("%.6lf seconds elapsed\n", sol_time-m_time_planning_start);
            m_time_first_solution = sol_time - m_time_planning_start;

            //Set last solution time to first solution time
            m_time_last_solution = m_time_first_solution;

            //Store iteration number when first solution is found
            m_first_solution_iter = m_executed_planner_iter;

            //Set last solution found to first solution iter
            m_last_solution_iter = m_first_solution_iter;
        }

        //A solution path has been found
        m_solution_path_available = true;

        //Insert Node in tree_B
        insertNode(tree, edge_selected , node_selected, show_tree_vis);

        //Store connection data of optimal path
        m_connected_tree_name = tree->name;
        m_node_tree_B = node_selected; //Node of the tree "m_connected_tree_name" that has connected "to m_node_tee_A"
        m_node_tree_A = x_new; //Node of the other tree that has been reached by tree "m_connected_tree_name"

        if (show_tree_vis == true)
        {
            //Perform search for best solution path and shows it as a line strip
            // -> "node_selected" (from tree_B) and "x_new" (from tree_A) are basically the same nodes
            showCurrentSolutionPath(tree->name, node_selected, x_new);
        }

        //Update best solution path cost found
        m_cost_best_solution_path = cost_solution_path;

        //Set last solution found to current planning iteration
        m_last_solution_iter = m_executed_planner_iter;

        //Update time when last solution has been found
        gettimeofday(&m_timer, NULL);
        double sol_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
        m_time_last_solution = sol_time - m_time_planning_start;

        //Output
        cout<<"New Best Solution Path cost (through CONNECT): "<<m_cost_best_solution_path<<endl;

    }
    else //No new solution found, but tree has been probably expanded towards the other tree
    {
        //Check if tree could be expanded towards the other tree by a valid edge
        if(tree_expand == true)
        {
            //Insert Node in tree_B
            insertNode(tree, edge_selected , node_selected, show_tree_vis);
        }
    }

}



//Try to connect the two RRT* Trees given the new node added to "tree_A" and its nearest neighbour in "tree_B"
void BiRRTstarPlanner::connectGraphsInterpolation(Rrt_star_tree *tree, Node x_connect, Node x_new, bool show_tree_vis)
{

//    //Variable for timer
//    gettimeofday(&m_timer, NULL);
//    double time_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);


    //Flag indicating whether the tree has been only expanded towards the other tree or has connected to it
    bool tree_expand = false;

    //Best Theoretical solution cost from near vertex
    double best_near_vertex_sol_cost = 10000.0;

    //Selected node and edge
    Node node_selected;
    Edge edge_selected;

    // Selected Via Nodes and Edges to x_new of other tree
    vector<Node> selected_via_nodes;
    vector<Edge> selected_via_edges;
    selected_via_nodes.clear();
    selected_via_edges.clear();

    //Cost of potential solution path (total + revolute and prismatic component)
    vector<double> cost_solution_path(3);
    cost_solution_path[0] = m_cost_best_solution_path;          //total
    cost_solution_path[1] = m_cost_best_solution_path_revolute; //revolute component
    cost_solution_path[2] = m_cost_best_solution_path_prismatic; //prismatic component


    if(m_constraint_active == false)
    {
        //Generated node and edge by connectNodesInterpolation
        Node gen_node;
        Edge gen_edge;
        connectNodesInterpolation(tree, x_connect, x_new, m_num_traj_segments_interp, gen_node, gen_edge);
        // Notes:
        // x_connect if from tree_B
        // x_new if from tree_A
        // gen_node.cost_reach.total = x_connect.cost_reach.total + edge cost to x_new (cost expressed w.r.t tree_B)
        // x_new.cost_reach.total = cost to reach node x_new (expressed w.r.t tree_A)

        //Compute solution path cost
        vector<double> solution_cost(3);
        solution_cost[0] = gen_node.cost_reach.total + x_new.cost_reach.total;          //total
        solution_cost[1] = gen_node.cost_reach.revolute + x_new.cost_reach.revolute;    //revolute
        solution_cost[2] = gen_node.cost_reach.prismatic + x_new.cost_reach.prismatic;  //prismatic

        //Check whether solution path cost is lower than best solution path cost
        if(solution_cost[0] < cost_solution_path[0])
        {
            //cout<<"connect"<<endl;

            //Index of last valid node (when edge is invalid)
            int index_last_valid_node = 0;
            //Check validity of edge connecting nn_node to x_new
            if(m_FeasibilityChecker->isEdgeValid(gen_edge, index_last_valid_node))
            {
                //Set solution path cost (will be probably improved by on of the near nodes, see code below)
                cost_solution_path[0] = solution_cost[0];   //total
                cost_solution_path[1] = solution_cost[1];   //revolute
                cost_solution_path[2] = solution_cost[2];   //prismatic

                //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for the lowest cost nearest neighbour)
                int num_nodes_tree = tree->num_nodes;
                //Get current number of nodes in the tree (required to avoid incrementing the number of edges while searching for the lowest cost nearest neighbour)
                int num_edges_tree = tree->num_edges;

                bool rand_sample_reached = false;
                while(rand_sample_reached == false)
                {
                    //Perform a step from current x_connect towards q_rand using a fixed step size (returns true when random sample has been reached by expansion)
                    Node original_x_new = x_new;
                    rand_sample_reached = stepTowardsRandSample(x_connect, original_x_new, m_unconstraint_extend_step_factor);

                    //Connect nodes
                    connectNodesInterpolation(tree, x_connect, original_x_new, m_num_traj_segments_interp, gen_node, gen_edge);

                    if(rand_sample_reached == false)
                    {

                        //Modify node data
                        gen_node.node_id = num_nodes_tree;
                        //Increment local tree nodes counter
                        num_nodes_tree++;
                        //Set near vertex as parent of gen_node
                        gen_node.parent_id = x_connect.node_id;

                        //Modify edge data
                        gen_edge.edge_id = num_edges_tree;
                        //Increment local tree nodes counter
                        num_edges_tree++;
                        //Reset child node ID of edge, because original original_x_new has not been reached
                        gen_edge.child_node_id = gen_node.node_id;  //Reset child node ID of edge, because original_x_new's ID is w.r.t. the other tree data structure

                        //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                        selected_via_nodes.push_back(gen_node);
                        selected_via_edges.push_back(gen_edge);

                        //Update x_connect in order to step further towards x_new
                        x_connect = gen_node;


                    }
                    else
                    {

                        //Set Node ID
                        node_selected = gen_node;
                        node_selected.node_id = num_nodes_tree;
                        //Set near vertex as parent of gen_node
                        node_selected.parent_id = x_connect.node_id;

                        //Set selected edge
                        edge_selected = gen_edge;
                        edge_selected.edge_id = num_edges_tree;
                        edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                        //Set flag indicating that the tree has established a connection to the other tree (not only expanded)
                        tree_expand = false;

                    }
                }
            } //End if edge valid
            else
            {

                //Node for connect extension (without reaching other tree)
                Node extend_node;

                //Generated node and edge by connectNodesInterpolation
                Node ext_node;
                Edge ext_edge;

                //If last valid config of edge is not the start config of the edge
                if(index_last_valid_node != 0)
                {
                    //Get last valid configuration from the invalid edge
                    extend_node.config = gen_edge.joint_trajectory[index_last_valid_node];

                    //Try to connect "x_connect" to current configuration of "extend_node" by a valid edge
                    //connectNodesInterpolation(tree, x_connect, extend_node, m_num_traj_segments_interp, ext_node, ext_edge);


                    //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for the lowest cost nearest neighbour)
                    int num_nodes_tree = tree->num_nodes;
                    //Get current number of nodes in the tree (required to avoid incrementing the number of edges while searching for the lowest cost nearest neighbour)
                    int num_edges_tree = tree->num_edges;

                    bool rand_sample_reached = false;
                    while(rand_sample_reached == false)
                    {
                        //Perform a step from current x_connect towards q_rand using a fixed step size (returns true when random sample has been reached by expansion)
                        Node original_extend_node = extend_node;
                        rand_sample_reached = stepTowardsRandSample(x_connect, original_extend_node, m_unconstraint_extend_step_factor);

                        //Connect nodes
                        connectNodesInterpolation(tree, x_connect, original_extend_node, m_num_traj_segments_interp, ext_node, ext_edge);

                        if(rand_sample_reached == false)
                        {
                            //Modify node data
                            ext_node.node_id = num_nodes_tree;
                            //Increment local tree nodes counter
                            num_nodes_tree++;
                            //Set near vertex as parent of ext_node
                            ext_node.parent_id = x_connect.node_id;

                            //Modify edge data
                            ext_edge.edge_id = num_edges_tree;
                            //Increment local tree nodes counter
                            num_edges_tree++;
                            //Reset child node ID of edge, because original original_extend_node has not been reached
                            ext_edge.child_node_id = ext_node.node_id;  //Reset child node ID of edge, because original_extend_node's ID is w.r.t. the other tree data structure

                            //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                            selected_via_nodes.push_back(ext_node);
                            selected_via_edges.push_back(ext_edge);

                            //Update x_connect in order to step further towards x_new
                            x_connect = ext_node;

                        }
                        else
                        {
                            //Set Node ID
                            node_selected = ext_node;
                            node_selected.node_id = num_nodes_tree;
                            //Set near vertex as parent of ext_node
                            node_selected.parent_id = x_connect.node_id;

                            //Set selected edge
                            edge_selected = ext_edge;
                            edge_selected.edge_id = num_edges_tree;
                            edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                            //Set flag indicating that the tree has been expanded without connecting to the other tree
                            tree_expand = true;

                            //Set best Theoretical solution cost from nearest neighbour
                            best_near_vertex_sol_cost = solution_cost[0];
                        }

                    }//END while rand_sample_reached == false
                }//END if last valid config unequal start config of edge
            } //End of extend step used when connection failed
        }//END solution path cost check


        //Additional best parent search (only used when a first solution has been found)
        if(m_solution_path_available == true)
        {
            //Find the set of vertices in the vicinity of x_new
            vector<int> near_nodes = find_near_vertices_interpolation(tree, x_new);

            //Choose Parent for Node (from tree_B), i.e. the one providing the lowest cost for reaching x_new
            for (int nv = 0; nv < near_nodes.size() ; nv++)
            {
                //Leave near vertices loop when "m_max_near_nodes" nodes have been considered as potential parents of x_new
                if (m_max_near_nodes == nv)
                    break;

                //Near Nodes of tree can be neglected if their cost-to-reach is already higher than the current cost of x_new, i.e. they can't provide a better path
                if(tree->nodes[near_nodes[nv]].cost_reach.total < x_new.cost_reach.total)
                {
                   //Generated node and edge by connectNodesInterpolation
                   Node ext_node;
                   Edge ext_edge;

                   //Try to connect a near vertex to x_new
                   connectNodesInterpolation(tree, tree->nodes[near_nodes[nv]], x_new, m_num_traj_segments_interp, ext_node, ext_edge);


                   //Compute overall solution cost, i.e. cost of reaching x_new via near node (from tree_B) + cost of reaching x_new (from tree_A)
                   //vector<double> solution_cost(3);
                   solution_cost[0] = ext_node.cost_reach.total + x_new.cost_reach.total;          //total
                   solution_cost[1] = ext_node.cost_reach.revolute + x_new.cost_reach.revolute;    //revolute
                   solution_cost[2] = ext_node.cost_reach.prismatic + x_new.cost_reach.prismatic;  //prismatic

                   //If total cost is smaller than the total cost of reaching x_new via it's nearest neighbour
                   if(solution_cost[0] < cost_solution_path[0])
                    {
                        //cout<<"alternative connect"<<endl;

                        //Index of last valid node (when edge is invalid)
                        int index_last_valid_node = 0;
                        //Only if edge to x_new is valid
                        if(m_FeasibilityChecker->isEdgeValid(ext_edge, index_last_valid_node))
                        {
//                            cout<<tree->name<<endl;
//                            cout<<tree->nodes[near_nodes[nv]].config[0]<<" "<<tree->nodes[near_nodes[nv]].config[1]<<endl;
//                            cout<< x_new.config[0]<<" "<< x_new.config[1]<<endl;
//                            //cout<< ext_edge.joint_trajectory[0][0]<<" "<< ext_edge.joint_trajectory[0][1]<<endl;

//                            for(int c = 0; c<ext_edge.joint_trajectory.size(); c++)
//                            {
//                                for(int k = 0; k<ext_edge.joint_trajectory[c].size(); k++)
//                                    cout<< ext_edge.joint_trajectory[c][k]<<" ";
//                                cout<<endl;
//                            }

//                            int t;
//                            cin>>t;

                            //Update solution path cost
                            cost_solution_path[0] = solution_cost[0];   //total
                            cost_solution_path[1] = solution_cost[1];   //revolute
                            cost_solution_path[2] = solution_cost[2];   //prismatic

                            //Clear selected via nodes and edges
                            selected_via_nodes.clear();
                            selected_via_edges.clear();

                            //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for the lowest cost nearest neighbour)
                            int num_nodes_tree = tree->num_nodes;
                            //Get current number of nodes in the tree (required to avoid incrementing the number of edges while searching for the lowest cost nearest neighbour)
                            int num_edges_tree = tree->num_edges;

                            //Init current near node
                            Node curr_near_node = tree->nodes[near_nodes[nv]];

                            bool rand_sample_reached = false;
                            while(rand_sample_reached == false)
                            {
                                //Perform a step from current curr_near_node towards x_new using a fixed step size (returns true when random sample has been reached by expansion)
                                Node original_x_new = x_new;
                                rand_sample_reached = stepTowardsRandSample(curr_near_node, original_x_new, m_unconstraint_extend_step_factor);

                                //Connect Nodes
                                connectNodesInterpolation(tree, curr_near_node, original_x_new, m_num_traj_segments_interp, ext_node, ext_edge);

                                if(rand_sample_reached == false)
                                {
                                    //Modify node data
                                    ext_node.node_id = num_nodes_tree;
                                    //Increment local tree nodes counter
                                    num_nodes_tree++;
                                    //Set near vertex as parent of ext_node
                                    ext_node.parent_id = curr_near_node.node_id;

                                    //Modify edge data
                                    ext_edge.edge_id = num_edges_tree;
                                    //Increment local tree nodes counter
                                    num_edges_tree++;
                                    //Reset child node ID of edge, because original original_x_new has not been reached
                                    ext_edge.child_node_id = ext_node.node_id;  //Reset child node ID of edge, because original_x_new's ID is w.r.t. the other tree data structure

                                    //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                                    selected_via_nodes.push_back(ext_node);
                                    selected_via_edges.push_back(ext_edge);

                                    //Update curr_near_node in order to step further towards x_new
                                    curr_near_node = ext_node;


                                }
                                else
                                {

                                    //Set Node ID
                                    node_selected = ext_node;
                                    node_selected.node_id = num_nodes_tree;
                                    //Set near vertex as parent of ext_node
                                    node_selected.parent_id = curr_near_node.node_id;

                                    //Set selected edge
                                    edge_selected = ext_edge;
                                    edge_selected.edge_id = num_edges_tree;
                                    edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                                    //Set flag indicating that the tree has established a connection to the other tree (not only expanded)
                                    tree_expand = false;
                                }
                            }


                            //Leave the loop when a valid edge has been found (next near vertices will have a higher cost)
                            break;

                        } //END if edge valid
                        else
                        {
                            //Check if a near vertex has established a connection to the other tree (if yes, "cost_solution_path[0]" will be unequal "m_cost_best_solution_path")
                            // -> thus partial extend will be unnecessary
                            if (cost_solution_path[0] == m_cost_best_solution_path && solution_cost[0] < best_near_vertex_sol_cost)
                            {
                                //Node for connect extension (without reaching other tree)
                                Node extend_node;

                                //If last valid config of edge is not the start config of the edge
                                if(index_last_valid_node != 0)
                                {
                                    //Get last valid configuration from the invalid edge
                                    extend_node.config = ext_edge.joint_trajectory[index_last_valid_node];

                                    //Try to connect "near node" to last valid configuration of the invalid valid edge
                                    //connectNodesInterpolation(tree, tree->nodes[near_nodes[nv]], extend_node, m_num_traj_segments_interp, ext_node, ext_edge);


                                    //Clear selected via nodes and edges
                                    selected_via_nodes.clear();
                                    selected_via_edges.clear();

                                    //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for the lowest cost nearest neighbour)
                                    int num_nodes_tree = tree->num_nodes;
                                    //Get current number of nodes in the tree (required to avoid incrementing the number of edges while searching for the lowest cost nearest neighbour)
                                    int num_edges_tree = tree->num_edges;

                                    //Init current near node
                                    Node curr_near_node = tree->nodes[near_nodes[nv]];

                                    bool rand_sample_reached = false;
                                    while(rand_sample_reached == false)
                                    {
                                        //Perform a step from current curr_near_node towards q_rand using a fixed step size (returns true when random sample has been reached by expansion)
                                        Node original_extend_node = extend_node;
                                        rand_sample_reached = stepTowardsRandSample(curr_near_node, original_extend_node, m_unconstraint_extend_step_factor);

                                        //Connect Nodes
                                        connectNodesInterpolation(tree, curr_near_node, original_extend_node, m_num_traj_segments_interp, ext_node, ext_edge);

                                        if(rand_sample_reached == false)
                                        {
                                            //Modify node data
                                            ext_node.node_id = num_nodes_tree;
                                            //Increment local tree nodes counter
                                            num_nodes_tree++;
                                            //Set near vertex as parent of ext_node
                                            ext_node.parent_id = curr_near_node.node_id;

                                            //Modify edge data
                                            ext_edge.edge_id = num_edges_tree;
                                            //Increment local tree nodes counter
                                            num_edges_tree++;
                                            //Reset child node ID of edge, because original original_extend_node has not been reached
                                            ext_edge.child_node_id = ext_node.node_id;  //Reset child node ID of edge, because original_extend_node's ID is w.r.t. the other tree data structure

                                            //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                                            selected_via_nodes.push_back(ext_node);
                                            selected_via_edges.push_back(ext_edge);

                                            //Update curr_near_node in order to step further towards x_new
                                            curr_near_node = ext_node;

                                        }
                                        else
                                        {
                                            //Set Node ID
                                            node_selected = ext_node;
                                            node_selected.node_id = num_nodes_tree;
                                            //Set near vertex as parent of ext_node
                                            node_selected.parent_id = curr_near_node.node_id;

                                            //Set selected edge
                                            edge_selected = ext_edge;
                                            edge_selected.edge_id = num_edges_tree;
                                            edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                                            //Set flag indicating that the tree has been expanded without connecting to the other tree
                                            tree_expand = true;

                                            //Set best Theoretical solution cost from nearest neighbour
                                            best_near_vertex_sol_cost = solution_cost[0];
                                        }
                                    }//END while rand_sample_reached == false
                                } //END if last valid config of edge unequal star config of edge
                            } // END Check if a near vertex has established a connection to the other tree
                        }//END else -> edge invalid
                    } //End Reduced cost test
              }//END Near node cost check
            } //End iteration through near vertices
        }//END Additional best parent search

        //Adding via nodes and edges to the tree for one of the following connections established:
        // -> x_connect connected to x_new of the other tree
        // -> x_connect has made progress towards x_new of the other tree
        // -> a near node of tree_B (other than x_connect) has connected to x_new of the other tree
        // -> a near node of tree_B (other than x_connect) has made progress towards x_new of the other tree

        //Insert via nodes and edges into tree
        // -> Note: Final node_selected and edge_selected will be added to tree in when a lower cost solution has been found (see code at bottom)
        for(int i = 0 ; i < selected_via_nodes.size() ; i++)
        {
             insertNode(tree, selected_via_edges[i] , selected_via_nodes[i], show_tree_vis);
        }



    }//END Query Constraint Active
    else
    {
        //Generated node and edge by connectNodesInterpolation
        Node gen_node;
        Edge gen_edge;

        bool rand_sample_reached = false;
        while(rand_sample_reached == false)
        {
            //cout<<"In while loop"<<endl;

            //Perform a step from x_connect towards x_new using a fixed step size (returns true when x_new of other tree has been reached by expansion)
            Node exp_node_towards_tree_A = x_new;
            rand_sample_reached = stepTowardsRandSample(x_connect, exp_node_towards_tree_A, m_constraint_extend_step_factor);

            //Projection only required if x_new has not been reached (because x_new.config is already on constraint manifold)
            if(rand_sample_reached == false)
            {
                //Is task frame position defined global or w.r.t near node frame
                if(m_task_pos_global == false)
                {
                    //Set Task frame position to position of nearest neighbour ee pose
                    m_task_frame.p.x(x_connect.ee_pose[0]); //Set X Position
                    m_task_frame.p.y(x_connect.ee_pose[1]); //Set Y Position
                    m_task_frame.p.z(x_connect.ee_pose[2]); //Set Z Position
                }
                //Is task frame orientation defined global or w.r.t near node frame
                if(m_task_orient_global == false)
                {
                    KDL::Rotation ee_orient = KDL::Rotation::Quaternion(x_connect.ee_pose[3],x_connect.ee_pose[4],x_connect.ee_pose[5],x_connect.ee_pose[6]);
                    m_task_frame.M = ee_orient;
                }

                //Project sample onto constraint manifold
                bool projection_succeed = m_RobotMotionController->run_config_retraction(exp_node_towards_tree_A.config, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev, m_max_projection_iter);

                if(projection_succeed == true && m_FeasibilityChecker->isConfigValid(exp_node_towards_tree_A.config))
                {

                    //Check if config retraction has projected the expanded config back onto the config of the near node "x_connect"
                    // -> In this case the expansion is stuck, because each projected sample/config will be identical to the config of "x_connect"
                    double dist_near_to_projected_sample = m_Heuristic.euclidean_joint_space_distance(x_connect.config,exp_node_towards_tree_A.config);

                    //Check additionally whether expansion has made progress towards x_new
                    // 1) checking whether sample has been projected back onto near sample (here x_connect.config)
                    // 2) checking whether projected sample has made progress towards target node (here x_new.config)
                    double dist_x_con_to_x_new = m_Heuristic.euclidean_joint_space_distance(x_connect.config,x_new.config);
                    double dist_projected_sample_to_x_new = m_Heuristic.euclidean_joint_space_distance(exp_node_towards_tree_A.config,x_new.config);

                    if(dist_near_to_projected_sample < m_min_projection_distance || dist_x_con_to_x_new < dist_projected_sample_to_x_new)
                    {
                        //if(dist_near_to_projected_sample < m_min_projection_distance)
                        //    cout<<"connectGraphsInterpolation: Sample projected back onto config of near node!"<<endl;
                        //else
                        //    cout<<"connectGraphsInterpolation: Projected sample is further away from x_new than x_connect!"<<endl;
                        break;
                    }
                    else
                    {
                        //cout<<"Projection generated new sample!"<<endl;
                        //cout<<dist_near_to_projected_sample<<endl;
                    }


                    //Connect x_connect to projected sample
                    bool connection_result = connectNodesInterpolation(tree, x_connect, exp_node_towards_tree_A, m_num_traj_segments_interp, gen_node, gen_edge);

                    //Check edge from x_connect to exp_node_towards_tree_A for validity (collision check)
                    if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                    {
                        //KDL::JntArray projected_sample = m_RobotMotionController->Vector_to_JntArray(exp_node_towards_tree_A.config);
                        //exp_node_towards_tree_A.ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,projected_sample);

                        //Modify node data
                        gen_node.node_id = tree->num_nodes;
                        //Set near vertex as parent of gen_node
                        gen_node.parent_id = x_connect.node_id;

                        //Modify edge data
                        gen_edge.child_node_id = gen_node.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                        //Insert Node in tree_B
                        insertNode(tree, gen_edge , gen_node, show_tree_vis);

                        //Update x_connect in order to step further towards x_new
                        x_connect = gen_node;

                        //cout<<"x_connect updated"<<endl;
                        //for(int i = 0 ; i < 7 ; i++)
                        //    cout<<x_connect.config[i]<<" ";
                        //cout<<endl;

                        //Set flag indicating that the tree has been expanded without connecting to the other tree
                        tree_expand = true;
                    }
                    else
                    {
                        //if(connection_result == false)
                        //    cout<<"Edge contains invalid samples"<<endl;
                        //else
                        //    cout<<"Edge config is in collision"<<endl;
                        break;
                    }
                 }
                else
                {
                    //if(projection_succeed == false)
                    //    cout<<"Projection failed"<<endl;
                    //else
                    //    cout<<"Projected sample is invalid"<<endl;
                    break;
                }
            }
            else
            {

                //cout<<"Sample reached!"<<endl;

                //Connect current x_connect to x_new
                bool connection_result = connectNodesInterpolation(tree, x_connect, x_new, m_num_traj_segments_interp, gen_node, gen_edge);


                //Compute overall solution cost, i.e. cost of reaching x_new via near node (from tree_B) + cost of reaching x_new (from tree_A)
                vector<double> solution_cost(3);
                solution_cost[0] = gen_node.cost_reach.total + x_new.cost_reach.total;          //total
                solution_cost[1] = gen_node.cost_reach.revolute + x_new.cost_reach.revolute;    //revolute
                solution_cost[2] = gen_node.cost_reach.prismatic + x_new.cost_reach.prismatic;  //prismatic


                //Check validity of edge connecting nn_node to x_new
                if(solution_cost[0] < cost_solution_path[0])
                {
                    //Check edge from x_connect to exp_node_towards_tree_A for validity (collision check)
                    if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                    {
                        //cout<<"sample reached"<<endl;
                        //cout<<endl;

                        //Set solution path cost
                        cost_solution_path[0] = solution_cost[0];
                        cost_solution_path[1] = solution_cost[1];
                        cost_solution_path[2] = solution_cost[2];


                        //Set Node ID
                        node_selected = gen_node;
                        node_selected.node_id = tree->num_nodes;
                        //Set near vertex as parent of gen_node
                        node_selected.parent_id = x_connect.node_id;

                        //Set selected edge
                        edge_selected = gen_edge;
                        edge_selected.child_node_id = node_selected.node_id;  //Reset child node ID of edge, because x_new's ID is w.r.t. the other tree data structure

                        //Set flag indicating that the tree has established a connection to the other tree (not only expanded)
                        tree_expand = false;

                        //Note: Selected edge and node is added to the tree in the final query (see code below)

                    }//Edge validity check
                    else
                    {
                        //cout<<"last segment not valid"<<endl;
                        //cout<<endl;
                    }

                }//Solution path cost check
            }//Query Sample Reached FALSE
        }//END Loop While Sample Not Reached

        //-- TODO
        //-> SEARCH FOR BEST PARENT FOR X_NEW WHEN CONSTRAINTS ARE ACTIVE

    }//END Query Constraint Active




    //-----------------------------------------------------

    //Check whether solution path cost of connected RRT* trees has been improved
    if(cost_solution_path[0] < m_cost_best_solution_path)
    {
        //Store time to find first solution (afterwards "m_solution_path_available" remains always true)
        if (m_solution_path_available == false)
        {
            //Get time elapsed
            gettimeofday(&m_timer, NULL);
            double sol_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
            //printf("%.6lf seconds elapsed\n", sol_time-m_time_planning_start);
            m_time_first_solution = sol_time - m_time_planning_start;

            //Set last solution time to first solution time
            m_time_last_solution = m_time_first_solution;

            //Store iteration number when first solution is found
            m_first_solution_iter = m_executed_planner_iter;

            //Set last solution found to first solution iter
            m_last_solution_iter = m_first_solution_iter;

            //Set planning success to true
            m_planner_success = 1;
        }

        //A solution path has been found
        m_solution_path_available = true;

        //Insert Node in tree_B
        insertNode(tree, edge_selected , node_selected, show_tree_vis);

        //Store connection data of optimal path
        m_connected_tree_name = tree->name;
        m_node_tree_B = node_selected; //Node of the tree "m_connected_tree_name" that has connected "to m_node_tee_A"
        m_node_tree_A = x_new; //Node of the other tree that has been reached by tree "m_connected_tree_name"

        //cout<<m_connected_tree_name<<endl;

        if (show_tree_vis == true)
        {
            //Perform search for best solution path and shows it as a line strip
            // -> "node_selected" (from tree_B) and "x_new" (from tree_A) are basically the same nodes
            showCurrentSolutionPath(tree->name, node_selected, x_new);
        }

        //Update best solution path cost found (total cost + cost of revolute and prismatic components)
        m_cost_best_solution_path = cost_solution_path[0];              //total
        m_cost_best_solution_path_revolute = cost_solution_path[1];     //revolute
        m_cost_best_solution_path_prismatic = cost_solution_path[2];    //prismatic

        //Set last solution found to current planning iteration
        m_last_solution_iter = m_executed_planner_iter;

        //Update time when last solution has been found
        gettimeofday(&m_timer, NULL);
        double sol_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
        m_time_last_solution = sol_time - m_time_planning_start;

        //Output
        //cout<<"**New Best Solution Path (through CONNECT)**"<<endl;
        //cout<<"Total Cost: "<<m_cost_best_solution_path<<endl;
        //cout<<"Revolute Cost: "<<m_cost_best_solution_path_revolute<<endl;
        //cout<<"Prismatic Cost: "<<m_cost_best_solution_path_prismatic<<endl;

        ROS_INFO_STREAM("New Best Solution Path (through CONNECT)");

    }
    else //No new solution found, but tree has been probably expanded towards the other tree
    {
        //Check if tree could be expanded towards the other tree by a valid edge
        //if(tree_expand == true && m_constraint_active == false)
        if(tree_expand == true && m_constraint_active == false)
        {
            //Insert Node in tree_B
            insertNode(tree, edge_selected , node_selected, show_tree_vis);
        }
    }


//    //Get time elapsed
//    gettimeofday(&m_timer, NULL);
//    double time_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//    printf("connectGraphsInterpolation %.6lf seconds elapsed\n", time_end - time_start);


}


//Insert Node and Edge into the RRT* Tree
void BiRRTstarPlanner::insertNode(Rrt_star_tree *tree, Edge e_new, Node x_new, bool show_tree_vis)
{
    //Assign ID to edge
    // -> equals the index of this edge in the edge marker array
    //e_new.edge_id = tree->num_edges;


//    cout<<"Add edge to insertNode:"<<endl;
//    cout<<x_new.parent_id<<endl;

//    cout<<"Edge data insertNode:"<<endl;
//    cout<<e_new.edge_id<<endl;
//    cout<<e_new.root_node_id<<endl;
//    cout<<e_new.child_node_id<<endl;

    //Store outgoing edge in near node providing the lowest cost for reaching x_new from start_node
    tree->nodes[x_new.parent_id].outgoing_edges.push_back(e_new);


//    cout<<"insertNode Parent data:"<<endl;
//    cout<<tree->nodes[x_new.parent_id].node_id<<endl;
//    cout<<tree->nodes[x_new.parent_id].cost_reach.total<<endl;
//    cout<<tree->nodes[x_new.parent_id].outgoing_edges.size()<<endl;


    //Store new node in RRT* tree
    tree->nodes.push_back(x_new);


    //-- Edge / EE Trajectory visualization --
    if(show_tree_vis == true)
    {
        //Add node to sphere_list
        add_tree_node_vis(tree->name, x_new);

        //cout<<" edge to visu added in INSERT_NODE"<<endl;

        //Add edge to marker array
        add_tree_edge_vis(tree->name, e_new);
    }

    //Increment number of nodes and edges
    tree->num_nodes++;
    tree->num_edges++;

}



////Check whether a solution has been found (not neccessarily the optimal one)
//bool BiRRTstarPlanner::solution_path_available()
//{
//    //Check if goal node has a parent node (goal node has parent_id = -1 until it has been reached by a tree node)
//    if(tree.nodes[0].parent_id > 0)
//    {
//        return true;
//    }
//    else
//        return false;

//}


//Sample Endeffector pose (used for Control-based tree expansion)
vector<double> BiRRTstarPlanner::sampleEEpose()
{
    //Get Random Pose for the endeffector
    vector<double> sampled_ee_pose = m_RobotMotionController->getRandEEPose();

    return sampled_ee_pose;
}


//Sample Endeffector pose from Informed Subset / Ellipse (used for Control-based tree expansion)
vector<double> BiRRTstarPlanner::sampleEEposefromEllipse()
{

    //Dimension task space
    int dim_task_space = m_ee_start_pose.size();

    //Random ee pose to be returned
    vector<double> rand_ee_pose_array(dim_task_space);
    Eigen::VectorXd  rand_ee_pose(3);


    //Random configuration sampled from 7-dimensional unit ball
    vector<double> ball_ee_pose_array;
    Eigen::VectorXd  ball_ee_pose(3);

    //Center of the 7-dimensional unit ball
    Eigen::VectorXd  ball_center(3);

    //Vectors to be used for computation of matrix "M"
    Eigen::VectorXd  a1(3);
    Eigen::VectorXd  id_m1(3);

    //Configuration validity checks
    bool collision_free = false;

    while (collision_free == false)
    {
        // ------------ START: Computation ball sample (ball_ee_pose) and center (ball_center)-----------

        //Get Random Pose for the endeffector (already guarantees that ee pose is above base platform)
        ball_ee_pose_array = m_RobotMotionController->getRandEEPose();

        //Compute norm of ee pose vector
        double sum_squares = 0.0;
        double ee_pose_vec_norm = 0.0;
        for (int j = 0 ; j < 3 ; j++)
        {
            sum_squares += (ball_ee_pose_array[j]*ball_ee_pose_array[j]);
        }
        //Norm of ee pose vector
        ee_pose_vec_norm = sqrt(sum_squares);

        //Sample a number between 0 and 1 to get a random point insider the 7-dimensional ball
        double point_scale = m_random_number_generator.uniformReal(0.0, 1.0);


        //Normalize and scale ee pose vector (to get sample on the surface of the 7-dimensional ball)
        // + Compute center of hyperellipsoid
        // + Compute Vectors to be used for computation of matrix "M"
        for (int j = 0 ; j < 3 ; j++)
        {
            ball_ee_pose[j] = point_scale * (ball_ee_pose_array[j] / ee_pose_vec_norm);
            ball_center[j] = (m_ee_start_pose[j] + m_ee_goal_pose[j]) / 2.0;
            a1[j] = (m_ee_goal_pose[j] - m_ee_start_pose[j]) / m_start_tree.nodes[0].cost_h.total; //m_tree.nodes[1].cost_h.total = heuristic for start node config
            if (j == 0)
                id_m1[j] = 1.0;
            else
                id_m1[j] = 0.0;
        }


        // ------------ START: Computation of the Transformation Matrix "L" and Rotation Matrix "C" -----------

        //Matrix required to compute matrix "C"
        Eigen::MatrixXd matrix_M (3, 3);
        matrix_M = a1*id_m1.transpose();

        // Compute SVD (as a result we obtain U*S*V)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix_M,  Eigen::ComputeFullU | Eigen::ComputeFullV);
        //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
        //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
        //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;

        //Diagonal matrix required to compute matrix "C"
        Eigen::MatrixXd diag_matrix (3, 3);

        //Transformation Matrix "L"
        Eigen::MatrixXd transformation_L (3, 3);

        //Rotation Matrix "C"
        Eigen::MatrixXd rotation_C (3, 3);


        //Set up matrix "L" and compute "diag_matrix" required for "C"
        for (int i = 0; i < transformation_L.cols(); i++)
        {
            for (int j = 0; j < transformation_L.cols(); j++)
            {
                if (i == j)
                {
                    //Computations for Matrix "L"
                    if(i == 0)
                        transformation_L(i,j) = m_cost_best_solution_path / 2.0; // m_cost_best_solution_path = cost of current best solution path
                    else
                        transformation_L(i,j) = sqrt((m_cost_best_solution_path*m_cost_best_solution_path) - (m_start_tree.nodes[0].cost_h.total*m_start_tree.nodes[0].cost_h.total)) / 2.0;  //m_tree.nodes[1].cost_h.total = Heurisitc cost of start configuration (node_id = 1), i.e. theoretical minimum cost reachable

                    //Computations for Matrix "M"
                    if(i < diag_matrix.cols() - 1)
                        diag_matrix(i,j) = 1.0;
                    else
                        diag_matrix(i,j) = svd.matrixU().determinant()*svd.matrixV().transpose().determinant();
                }
                else
                {
                    //Computations for Matrix "L"
                    transformation_L(i,j) = 0.0;
                    //Computations for Matrix "M"
                    diag_matrix(i,j) = 0.0;
                }
            }
        }

        //Matrix "C" representing rotation from the hyperellipsoid frame to the world frame
        rotation_C = svd.matrixU()*diag_matrix*svd.matrixV();

        // ------------ START: Computation of the random ee pose fro ellipse -----------

        //EE Pose/Sample (xf in paper)
        rand_ee_pose = rotation_C*transformation_L*ball_ee_pose + ball_center;

        //Convert random sample from Eigen Vector into KDL::JntArray
        for (int j = 0 ; j < dim_task_space ; j++)
        {
            if(j < 3)
                rand_ee_pose_array[j] = rand_ee_pose[j];
            else
                rand_ee_pose_array[j] = ball_ee_pose_array[j];
        }


        // ------------ START: Validity checks -----------
        //Check is it is collision free
        //if(m_FeasibilityChecker->isConfigValid(rand_conf_array))
            collision_free = true;


    }

    //Return random configuration from Ellipse
    return rand_ee_pose_array;
}



//Initialize constant parameters for Joint Config sampling from Ellipse
void BiRRTstarPlanner::jointConfigEllipseInitialization()
{

    //Vectors to be used for computation of matrix "M"
    Eigen::VectorXd  a1_rev;
    Eigen::VectorXd  id_m1_rev;
    if(m_num_joints_revolute != 0)
    {
        a1_rev.resize(m_num_joints_revolute);
        id_m1_rev.resize(m_num_joints_revolute);
    }

    Eigen::VectorXd  a1_prism;
    Eigen::VectorXd  id_m1_prism;
    if(m_num_joints_prismatic == 2)
    {
        a1_prism.resize(m_num_joints_prismatic);
        id_m1_prism.resize(m_num_joints_prismatic);
    }

    //Current Joint Index
    int joint_idx = 0;
    int joint_idx_rev = 0;
    int joint_idx_prism = 0;
    for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
     {
         if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
         {
             //Collect sum of squares for prismatic and revolute joints
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "RotAxis")
             {
                m_ball_center_prismatic[joint_idx_prism] = (m_config_start_pose[joint_idx] + m_config_goal_pose[joint_idx]) / 2.0;
                if(m_start_tree.nodes[0].cost_h.prismatic > 0.00001)
                    a1_prism[joint_idx_prism] = (m_config_goal_pose[joint_idx] - m_config_start_pose[joint_idx]) / m_start_tree.nodes[0].cost_h.prismatic; //m_start_tree.nodes[0].cost_h.total = heuristic for start node config
                else
                    a1_prism[joint_idx_prism] = 0.0;

                if (joint_idx_prism == 0)
                    id_m1_prism[joint_idx_prism] = 1.0;
                else
                    id_m1_prism[joint_idx_prism] = 0.0;

                joint_idx_prism++;
             }
             else
             {
                 m_ball_center_revolute[joint_idx_rev] = (m_config_start_pose[joint_idx] + m_config_goal_pose[joint_idx]) / 2.0;
                 if(m_start_tree.nodes[0].cost_h.revolute > 0.00001)
                    a1_rev[joint_idx_rev] = (m_config_goal_pose[joint_idx] - m_config_start_pose[joint_idx]) / m_start_tree.nodes[0].cost_h.revolute; //m_start_tree.nodes[0].cost_h.total = heuristic for start node config
                 else
                    a1_rev[joint_idx_rev] = 0.0;

                 if (joint_idx_rev == 0)
                     id_m1_rev[joint_idx_rev] = 1.0;
                 else
                     id_m1_rev[joint_idx_rev] = 0.0;

                 joint_idx_rev++;
             }

             joint_idx++;
         }
    }


//    for (int j = 0 ; j < m_num_joints_revolute ; j++)
//    {
//        m_ball_center[j] = (m_config_start_pose[j] + m_config_goal_pose[j]) / 2.0;
//        a1_rev[j] = (m_config_goal_pose[j] - m_config_start_pose[j]) / m_start_tree.nodes[0].cost_h.total; //m_start_tree.nodes[0].cost_h.total = heuristic for start node config
//        if (j == 0)
//            id_m1_rev[j] = 1.0;
//        else
//            id_m1_rev[j] = 0.0;
//    }


    //Matrix required to compute matrix "C"
    Eigen::MatrixXd matrix_M_rev;
    //Diagonal matrix required to compute matrix "C"
    Eigen::MatrixXd diag_matrix_rev;
    if(m_num_joints_revolute != 0)
    {
        matrix_M_rev.resize(m_num_joints_revolute, m_num_joints_revolute);
        matrix_M_rev = a1_rev*id_m1_rev.transpose();

        diag_matrix_rev.resize(m_num_joints_revolute, m_num_joints_revolute);

        // Compute SVD (as a result we obtain U*S*V)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_rev(matrix_M_rev,  Eigen::ComputeFullU | Eigen::ComputeFullV);
        //std::cout << "Its singular values are:" << std::endl << svd_rev.singularValues() << std::endl;
        //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd_rev.matrixU() << std::endl;
        //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd_rev.matrixV() << std::endl;

        //Compute "diag_matrix" required for "C"
        for (int i = 0; i < diag_matrix_rev.cols(); i++)
        {
            for (int j = 0; j < diag_matrix_rev.cols(); j++)
            {
                if (i == j)
                {
                    //Computations for Matrix "M"
                    if(i < diag_matrix_rev.cols() - 1)
                        diag_matrix_rev(i,j) = 1.0;
                    else
                        diag_matrix_rev(i,j) = svd_rev.matrixU().determinant()*svd_rev.matrixV().transpose().determinant();
                }
                else
                {
                    //Computations for Matrix "M"
                    diag_matrix_rev(i,j) = 0.0;
                }
            }
        }

        //Matrix "C" representing rotation from the hyperellipsoid frame to the world frame
        m_rotation_C_revolute = svd_rev.matrixU()*diag_matrix_rev*svd_rev.matrixV();

    }

    //Matrix required to compute matrix "C"
    Eigen::MatrixXd matrix_M_prism;
    //Diagonal matrix required to compute matrix "C"
    Eigen::MatrixXd diag_matrix_prism;
    if(m_num_joints_prismatic == 2)
    {
        matrix_M_prism.resize(m_num_joints_prismatic, m_num_joints_prismatic);
        matrix_M_prism = a1_prism*id_m1_prism.transpose();

        diag_matrix_prism.resize(m_num_joints_prismatic, m_num_joints_prismatic);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_prism(matrix_M_prism,  Eigen::ComputeFullU | Eigen::ComputeFullV);
        //std::cout << "Its singular values are:" << std::endl << svd_prism.singularValues() << std::endl;
        //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd_prism.matrixU() << std::endl;
        //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd_prism.matrixV() << std::endl;

        for (int i = 0; i < diag_matrix_prism.cols(); i++)
        {
            for (int j = 0; j < diag_matrix_prism.cols(); j++)
            {
                if (i == j)
                {
                    //Computations for Matrix "M"
                    if(i < diag_matrix_prism.cols() - 1)
                        diag_matrix_prism(i,j) = 1.0;
                    else
                        diag_matrix_prism(i,j) = svd_prism.matrixU().determinant()*svd_prism.matrixV().transpose().determinant();
                }
                else
                {
                    //Computations for Matrix "M"
                    diag_matrix_prism(i,j) = 0.0;
                }
            }
        }

        //Matrix "C" representing rotation from the hyperellipsoid frame to the world frame
        m_rotation_C_prismatic = svd_prism.matrixU()*diag_matrix_prism*svd_prism.matrixV();
    }

}



//Sample Configuration (as JntArray) from Ellipse containing configurations that may improve current solution
KDL::JntArray BiRRTstarPlanner::sampleJointConfigfromEllipse_JntArray()
{

//    //Variable for timer
//    gettimeofday(&m_timer, NULL);
//    double time_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

    //Random config sampled from ellipse -> to be returned by the function
    KDL::JntArray rand_conf_array;
    rand_conf_array = KDL::JntArray(m_num_joints);

    //Random configuration sampled from 7-dimensional unit ball
    KDL::JntArray ball_conf_array;
    KDL::JntArray ball_conf_array_revolute;
    KDL::JntArray ball_conf_array_prismatic;
    ball_conf_array = KDL::JntArray(m_num_joints);
    ball_conf_array_revolute = KDL::JntArray(m_num_joints_revolute);
    ball_conf_array_prismatic = KDL::JntArray(m_num_joints_prismatic);

    //Random configuration sampled from joint range
    Eigen::VectorXd  ball_conf_revolute(m_num_joints_revolute);
    Eigen::VectorXd  ball_conf_prismatic(m_num_joints_prismatic);


    //Configuration validity checks
    bool collision_free = false;
    //bool above_platform = false;
    bool inside_environment = false;

    //Sample until a collision free / above base platform / inside the environment configuration has been found
    //while (collision_free == false || above_platform == false || inside_environment == false)
    while (collision_free == false || inside_environment == false)
    {
        // ------------ START: Computation ball sample (ball_conf) and center (ball_center)-----------

        //Get random configuration (for entire robot)
        ball_conf_array = m_RobotMotionController->getRandomConf(m_env_size_x, m_env_size_y);

        //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
        //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
        if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute > 0) //-> m_num_joints_prismatic == 2, m_num_joints_revolute > 1 means robot base is involved in planning
        {
            transform_sample_to_base_link_frame(ball_conf_array);
        }

        //Split config vector into config of revolute and prismatic joints
        int joint_idx = 0;
        int joint_idx_rev = 0;
        int joint_idx_prism = 0;
        for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
         {
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
             {
                 if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() == "RotAxis")
                 {
                     ball_conf_array_revolute(joint_idx_rev) = ball_conf_array(joint_idx);
                     joint_idx_rev++;
                 }
                 else
                 {
                     ball_conf_array_prismatic(joint_idx_prism) = ball_conf_array(joint_idx);
                     joint_idx_prism++;
                 }

                 //Next joint
                 joint_idx++;
             }
        }


        //Compute norm of configuration vector
        double sum_squares_revolute = 0.0;
        double sum_squares_prismatic = 0.0;
        double conf_norm_revolute = 0.0;
        double conf_norm_prismatic = 0.0;
        //Compute norm of vector
        for (int i = 0 ; i < m_num_joints_revolute; i++)
        {
            sum_squares_revolute += (ball_conf_array_revolute(i)*ball_conf_array_revolute(i));
        }
        for (int j = 0 ; j < m_num_joints_prismatic; j++)
        {
            sum_squares_prismatic += (ball_conf_array_prismatic(j)*ball_conf_array_prismatic(j));
        }
        //Norm of configuration vector
        conf_norm_revolute = sqrt(sum_squares_revolute);
        conf_norm_prismatic= sqrt(sum_squares_prismatic);


        //Sample a number between 0 and 1 to get a random point insider the 7-dimensional ball
        //double point_scale = m_random_number_generator.uniformReal(-1.0, 1.0);
        double point_scale = m_random_number_generator.uniformReal(0.0, 1.0);

        //cout<<"PointScale: "<<point_scale<<endl;


        //Normalize and scale configuration vector (for revolute and prismatic joints)
        for (int j = 0 ; j < m_num_joints_revolute ; j++)
        {
            ball_conf_revolute[j] = point_scale * (ball_conf_array_revolute(j) / conf_norm_revolute);
        }
        for (int i = 0 ; i < m_num_joints_prismatic ; i++)
        {
            ball_conf_prismatic[i] = point_scale * (ball_conf_array_prismatic(i) / conf_norm_prismatic);
        }


        // ------------ START: Computation of the Transformation Matrix "L" -----------

        //Transformation Matrix "L"
        Eigen::MatrixXd transformation_L_rev (m_num_joints_revolute, m_num_joints_revolute);
        Eigen::MatrixXd transformation_L_prism (m_num_joints_prismatic, m_num_joints_prismatic);


        //Set up matrix "L" and compute "diag_matrix" required for "C"
        for (int i = 0; i < transformation_L_rev.cols(); i++)
        {
            for (int j = 0; j < transformation_L_rev.cols(); j++)
            {
                if (i == j)
                {
                    //Computations for Matrix "L"
                    if(i == 0)
                        transformation_L_rev(i,j) = m_cost_best_solution_path_revolute / 2.0; // m_cost_best_solution_path = cost of current best solution path
                    else
                    {
                        transformation_L_rev(i,j) = sqrt((m_cost_best_solution_path_revolute*m_cost_best_solution_path_revolute) - (m_start_tree.nodes[0].cost_h.revolute*m_start_tree.nodes[0].cost_h.revolute)) / 2.0;  //m_start_tree.nodes[0].cost_h = Heurisitc cost of start configuration (node_id = 1), i.e. theoretical minimum cost reachable
                    }
                }
                else
                {
                    //Computations for Matrix "L"
                    transformation_L_rev(i,j) = 0.0;
                }
            }
        }


        for (int i = 0; i < transformation_L_prism.cols(); i++)
        {
            for (int j = 0; j < transformation_L_prism.cols(); j++)
            {
                if (i == j)
                {
                    //Computations for Matrix "L"
                    if(i == 0)
                        transformation_L_prism(i,j) = m_cost_best_solution_path_prismatic / 2.0; // m_cost_best_solution_path = cost of current best solution path
                    else
                    {
                        transformation_L_prism(i,j) = sqrt((m_cost_best_solution_path_prismatic*m_cost_best_solution_path_prismatic) - (m_start_tree.nodes[0].cost_h.prismatic*m_start_tree.nodes[0].cost_h.prismatic)) / 2.0;  //m_start_tree.nodes[0].cost_h = Heurisitc cost of start configuration (node_id = 1), i.e. theoretical minimum cost reachable
                    }
                }
                else
                {
                    //Computations for Matrix "L"
                    transformation_L_prism(i,j) = 0.0;
                }
            }
        }


        // ------------ START: Computation of the random configuration fro ellipse ----------

        //Configuration/Sample (xf in paper)
        Eigen::VectorXd  rand_conf_rev(m_num_joints_revolute);
        Eigen::VectorXd  rand_conf_prism(m_num_joints_prismatic);
        if(0 < m_num_joints_revolute)
            rand_conf_rev = m_rotation_C_revolute * transformation_L_rev * ball_conf_revolute + m_ball_center_revolute;
        if(0 < m_num_joints_prismatic)
        rand_conf_prism = m_rotation_C_prismatic * transformation_L_prism * ball_conf_prismatic + m_ball_center_prismatic;


        // ------------ START: Convert EigenVector to KDL::JntArray ----------

        //Convert random sample from Eigen Vector into KDL::JntArray -> to be returned by the function
        int joint_index = 0;
        for (int j = 0 ; j < m_num_joints_prismatic ; j++)
        {
            rand_conf_array(joint_index) = rand_conf_prism[j];
            joint_index++;
            //cout<<rand_conf_array(j)<<endl;
        }
        for (int j = 0 ; j < m_num_joints_revolute ; j++)
        {
            rand_conf_array(joint_index) = rand_conf_rev[j];
            joint_index++;
            //cout<<rand_conf_array(j)<<endl;
        }
        //cout<<endl;


        // ------------ START: Validity checks -----------
        // Get endeffector pose for random config
        vector<double> ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,rand_conf_array);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        //if(0.0 <= ee_pose[2])
        //    above_platform = true;
        //else
        //    above_platform = false;

        //The robot has a base moving in x,y direction (i.e. has 2 prismatic joints)
        if(m_num_joints_prismatic == 2 && m_num_joints_revolute > 0)
        {

            //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
            KDL::JntArray copy_rand_conf_array = rand_conf_array;
            if(m_planning_frame == "/map")
                transform_sample_to_map_frame(copy_rand_conf_array);

            //Sampled Base position is ok if...
            // 1) The base position is within the confined environment borders
            // 2) The environment is not confined (i.e. infinite space for maneuvering available)
            if( ( copy_rand_conf_array(0) < m_env_size_x[1] && copy_rand_conf_array(0) > m_env_size_x[0] && copy_rand_conf_array(1) < m_env_size_y[1] && copy_rand_conf_array(1) > m_env_size_y[0] ) || (m_env_size_x[0] == 0.0 && m_env_size_x[1] == 0.0 && m_env_size_y[0] == 0.0 && m_env_size_y[1] == 0.0) )
                inside_environment = true;
            else
                inside_environment = false;
        }
        else
        {
            //No inside environment check required
            inside_environment = true;
        }

//        //The robot has a base moving in x,y direction (i.e. has at least 2 prismatic joints)
//        if(2 <= m_num_joints_prismatic)
//        {
//            //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
//            KDL::JntArray copy_rand_conf_array = rand_conf_array;
//            //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
//            if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
//            {
//                transform_sample_to_map_frame(copy_rand_conf_array);
//            }

//            //Sampled Base position is ok if...
//            // 1) The base position is within the confined environment borders
//            // 2) The environment is not confined (i.e. infinite space for maneuvering available)
//            if( ( copy_rand_conf_array(0) < m_env_size_x[1] && copy_rand_conf_array(0) > m_env_size_x[0] && copy_rand_conf_array(1) < m_env_size_y[1] && copy_rand_conf_array(1) > m_env_size_y[0] ) || (m_env_size_x[0] == 0.0 && m_env_size_x[1] == 0.0 && m_env_size_y[0] == 0.0 && m_env_size_y[1] == 0.0) )
//                inside_environment = true;
//            else
//                inside_environment = false;
//        }

        //Check if it is collision free
        //if(m_FeasibilityChecker->isConfigValid(rand_conf_array))
            collision_free = true;
        //else
        //    collision_free = false;


    }


//    //Get time elapsed
//    gettimeofday(&m_timer, NULL);
//    double time_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//    printf("sampleJointConfigfromEllipse_JntArray %.6lf seconds elapsed\n", time_end - time_start);

    //Return random configuration from Ellipse
    return rand_conf_array;

}



//Sample Configuration
KDL::JntArray BiRRTstarPlanner::sampleJointConfig_JntArray()
{
    KDL::JntArray rand_conf;
    rand_conf = KDL::JntArray(m_num_joints);

    //Configuration validity checks
    bool collision_free = false;
    //bool above_platform = false;

    //while (collision_free == false || above_platform == false)
    while (collision_free == false)
    {
        //Get random configuration
        rand_conf = m_RobotMotionController->getRandomConf(m_env_size_x, m_env_size_y);
	
        //cout<<"-------------------------"<<endl;
        //cout<<"rand config in map frame:"<<endl;
        //for(int i = 0 ; i < m_num_joints ; i++)
        //    cout<<rand_conf(i)<<endl;
        //cout<<endl;

        //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
        //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
        if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
        {
            transform_sample_to_base_link_frame(rand_conf);
        }

        //cout<<"rand config in base frame:"<<endl;
        //for(int i = 0 ; i < m_num_joints ; i++)
        //    cout<<rand_conf(i)<<endl;

        //cout<<"-------------------------"<<endl;

        // Get endeffector pose for random config
        vector<double> ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        //if(0.0 <= ee_pose[2])
        //    above_platform = true;

        //Check is it is collision free
        //if(m_FeasibilityChecker->isConfigValid(rand_conf))
            collision_free = true;
    }

    return rand_conf;
}


//Sample Configuration
KDL::JntArray BiRRTstarPlanner::sampleJointConfig_JntArray(vector<double> mean_config, double std_dev)
{
    KDL::JntArray rand_conf;
    rand_conf = KDL::JntArray(m_num_joints);

    //Configuration validity checks
    bool collision_free = false;
    //bool above_platform = false;

    //while (collision_free == false || above_platform == false)
    while (collision_free == false)
    {
        //Get random configuration
        rand_conf = m_RobotMotionController->getRandomConf(mean_config, std_dev, m_env_size_x, m_env_size_y);

        //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
        //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
        if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
        {
            transform_sample_to_base_link_frame(rand_conf);
        }

        //cout<<"rand config:"<<endl;
        //for(int i = 0 ; i < m_num_joints ; i++)
        //    cout<<rand_conf(i)<<endl;

        //cout<<endl;

        // Get endeffector pose for random config
        vector<double> ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,rand_conf);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        //if(0.0 <= ee_pose[2])
        //    above_platform = true;

        //Check is it is collision free
        //if(m_FeasibilityChecker->isConfigValid(rand_conf))
            collision_free = true;
    }

    return rand_conf;
}


//Sample Configuration
vector<double> BiRRTstarPlanner::sampleJointConfig_Vector()
{

    //Config as JntArray and Std::Vector
    KDL::JntArray rand_sample;
    rand_sample = KDL::JntArray(m_num_joints);
    vector<double> rand_conf_vec(m_num_joints);

    //Collision check
    bool collision_free = false;

    //Above table check
    //bool above_platform = false;

    //while (collision_free == false || above_platform == false)
    while (collision_free == false)
    {
        //Reset flags
        collision_free = false;
        //above_platform = false;

        //Get random configuration
        rand_sample = m_RobotMotionController->getRandomConf(m_env_size_x,m_env_size_y);

        //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
        //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
        if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
        {
            transform_sample_to_base_link_frame(rand_sample);
        }

        //cout<<rand_sample(0)<<" "<<rand_sample(1)<<" "<<rand_sample(2)<<" "<<rand_sample(3)<<" "<<rand_sample(4)<<" "<<rand_sample(5)<<" "<<rand_sample(6)<<" "<<rand_sample(7)<<" "<<rand_sample(8)<<" "<<rand_sample(9)<<endl;

        vector<double> ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,rand_sample);

        //Check whether end-effector is located above the base platform (i.e. z position > 0.0)
        //if(0.0 <= ee_pose[2])
        //    above_platform = true;

        //Check is it is collision free
        //if(m_FeasibilityChecker->isConfigValid(rand_sample))
            collision_free = true;
        //else
        //    cout<<"invalid"<<endl;
    }

    for(int i = 0 ; i < m_num_joints ; i++)
    {
        //cout<<rand_sample(i)<<endl;
        rand_conf_vec[i] = rand_sample(i);
    }

    return rand_conf_vec;
}


//Sample Configuration
vector<double> BiRRTstarPlanner::sampleJointConfig_Vector(vector<double> mean_config, double std_dev)
{

    //Config as JntArray and Std::Vector
    KDL::JntArray rand_sample;
    rand_sample = KDL::JntArray(m_num_joints);
    vector<double> rand_conf_vec(m_num_joints);

    //Collision check
    bool collision_free = false;

    //Above table check
    //bool above_platform = false;

    //while (collision_free == false || above_platform == false)
    while (collision_free == false)
    {
        //Reset flags
        collision_free = false;
        //above_platform = false;

        //Get random configuration
        rand_sample = m_RobotMotionController->getRandomConf(mean_config, std_dev, m_env_size_x, m_env_size_y);

        //Transform config sample from map frame into base_link frame (used when planning is performed with a real robot running a localization)
        //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
        if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
        {
            transform_sample_to_base_link_frame(rand_sample);
        }

        //for(int i = 0; i < rand_sample.rows(); i++)
        //    cout<<rand_sample(i)<<" ";
        //cout<<endl;

        vector<double> ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,rand_sample);

        //Check whether endeffector is located above the base platform (i.e. z position > 0.0)
        //if(0.0 <= ee_pose[2])
        //    above_platform = true;

        //Check is it is collision free
        //if(m_FeasibilityChecker->isConfigValid(rand_sample))
            collision_free = true;

        //cout<<"collision free: "<<collision_free<<endl;
        //cout<<"above_platform: "<<above_platform<<endl;

    }

    for(int i = 0 ; i < m_num_joints ; i++)
    {
        //cout<<rand_sample(i)<<endl;
        rand_conf_vec[i] = rand_sample(i);
    }

    return rand_conf_vec;
}



Node BiRRTstarPlanner::find_nearest_neighbour_control(Rrt_star_tree *tree, vector<double> x_rand)
{
    //Index of nearest neighbour node
    int id_nearest_neighbour = 0;

    //Current distance
    double curr_node_dist = 0.0;

    //Minimum distance found so far
    double min_node_dist = 10000.0;

    for (int n = 0 ; n < tree->nodes.size() ; n++)
    {
        //Do not consider goal node as closest node (goal node id = 0)
        //if (tree.nodes[n].parent_id != -1)
        //if (tree.nodes[n].node_id > 0)
        //{
            //Distance between sampled ee pose and ee pose of the current node
            curr_node_dist = m_Heuristic.euclidean_pose_distance(tree->nodes[n].ee_pose, x_rand);

            if(curr_node_dist < min_node_dist)
            {
                //cout<<"Nearest neighbour distance"<<endl;
                //cout<<curr_node_dist<<endl;

                //Reset ID of nearest node to index of current node
                id_nearest_neighbour = tree->nodes[n].node_id;
                //Update minimum distance found
                min_node_dist = curr_node_dist;
            }
        //}
    }

    //cout<<"Nearest neighbour ID: "<<id_nearest_neighbour<<endl;

    return tree->nodes[id_nearest_neighbour];
}


//Find the nearest neighbour (node storing the ee pose closest to the sampled config)
Node BiRRTstarPlanner::find_nearest_neighbour_interpolation(Rrt_star_tree *tree, vector<double> x_rand)
{

//    //Variable for timer
//    gettimeofday(&m_timer, NULL);
//    double time_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

    //Index of nearest neighbour node
    int id_nearest_neighbour = 0;

    //Current distance
    double curr_node_dist = 0.0;

    //Minimum distance found so far
    double min_node_dist = 10000.0;

    #pragma omp parallel for    //parallelize search for nearest neighbour
    for (int n = 0 ; n < tree->nodes.size() ; n++)
    {
        //Do not consider goal node as closest node (goal node id = 0)
        //if (tree->nodes[n].parent_id != -1)
        //if (tree->nodes[n].node_id > 0)
        //{

        //Flag indicating whether near vertex is inside informed subset
        bool inside_ellipsoidal_subset = true;

        if(m_solution_path_available)
        {
//            //Distance to start and goal tree root
//            vector<double> dist_to_start_tree_root = m_Heuristic.euclidean_joint_space_distance(m_manipulator_chain, m_config_start_pose, tree->nodes[n].config);
//            vector<double> dist_to_goal_tree_root = m_Heuristic.euclidean_joint_space_distance(m_manipulator_chain, tree->nodes[n].config, m_config_goal_pose);
//            //Length of path between start and goal tree roots via near vertex
//            double via_near_node_path_length = dist_to_start_tree_root[0] + dist_to_goal_tree_root[0];
//            if(m_cost_best_solution_path < via_near_node_path_length)
//                inside_ellipsoidal_subset = false;
        }

        //Consider near vertex only if it is inside the ellipsoidal subset
        // Note: all near vertices are considered until a solution path is found
        if(inside_ellipsoidal_subset == true)
        {
            //Distance between sampled ee pose and ee pose of the current node
            curr_node_dist = m_Heuristic.euclidean_joint_space_distance(tree->nodes[n].config, x_rand);

            #pragma omp critical(closervertex) //Only one thread can set a new closest node at a time
            if(curr_node_dist < min_node_dist)
            {
                //Reset ID of nearest node to index of current node
                id_nearest_neighbour = tree->nodes[n].node_id;
                //Update minimum distance found
                min_node_dist = curr_node_dist;
            }
        }
    }

    //cout<<"Nearest neighbour ID: "<<id_nearest_neighbour<<endl;

//    //Get time elapsed
//    gettimeofday(&m_timer, NULL);
//    double time_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//    printf("find_nearest_neighbour_interpolation %.6lf seconds elapsed\n", time_end - time_start);

    return tree->nodes[id_nearest_neighbour];
}



//Compute the cost of an edge connecting two nodes
double BiRRTstarPlanner::compute_edge_cost_control(vector<vector<double> > ee_traj)
{
    //Edge cost to be returned
    double edge_cost = 0.0;
    //Sum of squares of distance between two waypoints
    double wp_dist = 0.0;

    for (int wp = 0; wp < ee_traj.size()-1; wp++)
    {
        //Reset sum of squares to zero
        wp_dist = 0.0;

        //Cartesian distance
        for (int vc = 0; vc < ee_traj[0].size() ; vc++)
        {
            //Distance between current and next waypoint
            wp_dist += (ee_traj[wp+1][vc] - ee_traj[wp][vc]) * (ee_traj[wp+1][vc] - ee_traj[wp][vc]);
        }
        //Compute norm of cartesian distance (length of vector connecting ee pose of near_node and x_new)
        edge_cost += sqrt(wp_dist);
    }

    return edge_cost;
}



//Compute the cost of an edge connecting two nodes
vector<double> BiRRTstarPlanner::compute_edge_cost_interpolation(vector<vector<double> > joint_traj)
{
    //Cost vector to be returned
    vector<double> cost_list(3);

    //Edge cost for prismatic joints
    double edge_cost_prism = 0.0;
    //Edge cost for revolute joints
    double edge_cost_rev = 0.0;
    //Total Edge cost
    double edge_cost_total = 0.0;

    //Sum of squares of distance between two waypoints (in total, for prismatic and revolute joints)
    double wp_dist_total = 0.0;
    double wp_dist_prism = 0.0;
    double wp_dist_rot = 0.0;

    //Current Joint Index
    int joint_idx = 0;
    for (int wp = 0; wp < joint_traj.size()-1; wp++)
    {
        //Reset joint index
        joint_idx = 0;

        //Reset sum of squares to zero
        wp_dist_total = 0.0;
        wp_dist_rot = 0.0;
        wp_dist_prism = 0.0;

        for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
         {
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
             {
                 //Squared distance between coordinates
                 double coordinate_dist = (joint_traj[wp+1][joint_idx] - joint_traj[wp][joint_idx]) * (joint_traj[wp+1][joint_idx] - joint_traj[wp][joint_idx]);

                 //Collect distance between current and next waypoint coordinate (for prismatic and revolute joints)
                 // -> + weights individual costs by "m_edge_cost_weights"
                 wp_dist_total += coordinate_dist * m_edge_cost_weights[joint_idx];

                 //Collect distance between current and next waypoint coordinate (only revolute joints)
                 if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() == "RotAxis")
                     wp_dist_rot += coordinate_dist ;
                 else //Collect distance between current and next waypoint coordinate (only prismatic joints)
                     wp_dist_prism += coordinate_dist;

                 //Select next coordinate
                 joint_idx++;
             }
        }
        //Collect norm of edge sections (edge length considering revolute and prismatic joints)
        edge_cost_total += sqrt(wp_dist_total);
        //Collect norm of edge sections (edge length considering only revolute joints)
        edge_cost_rev += sqrt(wp_dist_rot);
        //Collect norm of edge sections (edge length considering only prismatic joints)
        edge_cost_prism += sqrt(wp_dist_prism);
    }

    //Set costs (total, revolute, prismatic)
    cost_list[0] = edge_cost_total;
    cost_list[1] = edge_cost_rev;
    cost_list[2] = edge_cost_prism;

//    for (int wp = 0; wp < joint_traj.size()-1; wp++)
//    {
//        //Reset sum of squares to zero
//        wp_dist = 0.0;

//        //Cartesian distance between two configurations
//        for (int vc = 0; vc < joint_traj[0].size() ; vc++)
//        {
//            //Distance between current and next waypoint
//            wp_dist += (joint_traj[wp+1][vc] - joint_traj[wp][vc]) * (joint_traj[wp+1][vc] - joint_traj[wp][vc]);
//        }
//        //Compute norm of cartesian distance (length of vector connecting ee pose of near_node and x_new)
//        edge_cost_total += sqrt(wp_dist);
//    }

    return cost_list;
}



//Find the set of vertices in the vicinity of x_new
vector<int> BiRRTstarPlanner::find_near_vertices_control(Rrt_star_tree *tree, Node x_new)
{
    //Vertices near x_new to be returned
    //vector<Node> n_vertices;
    vector<int> n_vertices;

    for (int n = 0 ; n < tree->nodes.size() ; n++)
    {
        double ee_pose_dist = m_Heuristic.euclidean_pose_distance(tree->nodes[n].ee_pose, x_new.ee_pose);

        //cout<<"cost curr vertices: "<<ee_pose_dist<<endl;

        //Add vertices to set of near vertices if it is close enough
        if(ee_pose_dist < m_near_threshold_control)
        {
            n_vertices.push_back(tree->nodes[n].node_id);
        }

    }

    //cout<<"Number of near vertices: "<<n_vertices.size()<<endl;

    //Return set of vertices
    return n_vertices;
}



//Find the set of vertices in the vicinity of x_new
vector<int> BiRRTstarPlanner::find_near_vertices_interpolation(Rrt_star_tree *tree, Node x_new)
{

//    //Variable for timer
//    gettimeofday(&m_timer, NULL);
//    double time_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

    //Near Vertices to x_new to be returned
    vector<Node> n_vertices; //Near Nodes Vector required to sort near nodes in ascending order of cost_reach
    vector<int> n_vertices_index; //Index of near nodes

    #pragma omp parallel for
    for (int n = 0 ; n < tree->nodes.size() ; n++)
    {
        //Node of tree can be neglected if its cost is already higher than the current cost of x_new
        // Remark: -> NOT POSSIBLE: Otherwise high cost nodes won't be considered in REWIRE
        //if(tree->nodes[n].cost_reach.total < x_new.cost_reach.total)
        //{
            double joint_space_dist = m_Heuristic.euclidean_joint_space_distance(tree->nodes[n].config, x_new.config);

            //cout<<"C space dist: "<<joint_space_dist<<endl;

            //Add vertices to set of near vertices if it is close enough and not the node itself
            if(joint_space_dist < m_near_threshold_interpolation && x_new.node_id != tree->nodes[n].node_id)
            {
                #pragma omp critical(nodeinsertion) //Only one thread can add a vertex at a time
                n_vertices.push_back(tree->nodes[n]);
            }
        //}

    }


    //cout<<"Number of near vertices: "<<n_vertices.size()<<endl;


    // Sort near vertices in order of ascending "cost_reach"
    //std::sort(n_vertices.begin(), n_vertices.end(), boost::bind(&Node::cost_reach::total, _1) < boost::bind(&Node::cost_reach::total, _2));
    sort(n_vertices.begin(), n_vertices.end());

    //Collect only ID's of near nods
    for (int nv = 0 ; nv < n_vertices.size() ; nv++)
    {
        n_vertices_index.push_back(n_vertices[nv].node_id);
        //cout<<n_vertices[nv].cost_reach.total<<endl;
    }

//    //Get time elapsed
//    gettimeofday(&m_timer, NULL);
//    double time_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//    printf("find_near_vertices_interpolation %.6lf seconds elapsed\n", time_end - time_start);


    //Return set of vertices
    return n_vertices_index;
}



//Connect Nodes (running Variable DLS Controller)
kuka_motion_controller::Status BiRRTstarPlanner::connectNodesControl(Rrt_star_tree *tree, Node near_node, Node end_node, Node &x_new, Edge &e_new)
{
    //Set start configuration and goal pose
    m_RobotMotionController->setStartConf(near_node.config);
    m_RobotMotionController->set_EE_goal_pose(end_node.ee_pose);

    //Array storing the joint and endeffector trajectory
    vector<vector<double> > joint_trajectory;
    vector<vector<double> > ee_trajectory;
    kuka_motion_controller::Status connector_state = m_RobotMotionController->run_VDLS_Control_Connector(500, joint_trajectory,ee_trajectory,0,0);


    //Set ID for node (0 = goal node , 1 = root node)
    //x_new.node_id = tree->nodes.size();
    x_new.node_id = end_node.node_id;

    //Set parent ID of new node to ID of near node
    x_new.parent_id = near_node.node_id;


    // --- Set Edge data ---
    //Assign ID to edge
    e_new.edge_id = tree->num_edges;

    //Compute cost of edge from near_node to x_new
    double edge_cost = compute_edge_cost_control(ee_trajectory);
    e_new.cost_g = edge_cost;

    //Set Start/End node ID's and joint and endeffector trajectory
    e_new.root_node_id = near_node.node_id;
    e_new.child_node_id = x_new.node_id;
    e_new.joint_trajectory = joint_trajectory;
    e_new.ee_trajectory = ee_trajectory;


    // --- Set Node data ---
    //Compute cost of reaching the "x_new" from the "start_node" via node "near_node"
    x_new.cost_reach.total = near_node.cost_reach.total + edge_cost;

    //cout<<"Cost from "<<near_node.node_id<<" to "<<end_node.node_id<<" is "<<x_new.cost_reach.total<<endl;

//    cout<<"Joint Traj size: "<<joint_trajectory.size()<<endl;
//    cout<<"EE Traj size: "<<ee_trajectory.size()<<endl;

    //Get index of last endeffector trajectory element
    int last_ee_traj_entry = ee_trajectory.size() - 1;
    x_new.ee_pose = ee_trajectory[last_ee_traj_entry];
    //Get index of last joint trajectory element
    int last_joint_traj_entry = joint_trajectory.size() - 1;
    x_new.config = joint_trajectory[last_joint_traj_entry];


    //Return connection state (TRAPPED/ADVANCED/REACHED)
    return connector_state;
}



//Connect Nodes (by interpolating configurations)
bool BiRRTstarPlanner::connectNodesInterpolation(Rrt_star_tree *tree, Node near_node, Node end_node, int num_points, Node &x_new, Edge &e_new)
{

    //Array storing the joint and endeffector trajectory
    vector<vector<double> > joint_trajectory;
    vector<vector<double> > ee_trajectory;

    //cout<<tree->name<<endl;

    //Interpolate configurations of near_node and end_node
    bool valid_interpolation = interpolateConfigurations(near_node,end_node, num_points ,joint_trajectory,ee_trajectory);

//    if(tree->name == "GOAL")
//    {
//        int t;
//        cin>>t;
//    }


    //If constraint is violated by an intermediate node
    if(m_constraint_active == true && valid_interpolation == false)
    {
        return false;
    }

    //Set ID for node (0 = goal node , 1 = root node)
    //x_new.node_id = tree->nodes.size();
    x_new.node_id = end_node.node_id;

    //Set parent ID of new node to ID of near node
    x_new.parent_id = near_node.node_id;


    // --- Set Edge data ---

    //Assign ID to edge
    e_new.edge_id = tree->num_edges;

    //Compute cost of edge from near_node to x_new
    vector<double> edge_costs = compute_edge_cost_interpolation(joint_trajectory);
    e_new.cost_g = edge_costs[0];
    //Set Start/End node ID's and joint and endeffector trajectory
    e_new.root_node_id = near_node.node_id;
    e_new.child_node_id = x_new.node_id;
    e_new.joint_trajectory = joint_trajectory;
    e_new.ee_trajectory = ee_trajectory;





    // --- Set Node data ---

    //Compute cost of reaching the "x_new" from the "start_node" via node "near_node"
    x_new.cost_reach.total = near_node.cost_reach.total + edge_costs[0];            //total
    x_new.cost_reach.revolute = near_node.cost_reach.revolute + edge_costs[1];      //revolute
    x_new.cost_reach.prismatic = near_node.cost_reach.prismatic + edge_costs[2];    //prismatic
    //Get index of last endeffector trajectory element
    int last_ee_traj_entry = ee_trajectory.size() - 1;
    x_new.ee_pose = ee_trajectory[last_ee_traj_entry];
    //Get index of last joint trajectory element
    int last_joint_traj_entry = joint_trajectory.size() - 1;
    x_new.config = joint_trajectory[last_joint_traj_entry];


    //Compute heuristic for final x_new's config
    bool tree_idx = tree->name == "START" ? true : false;
    if(tree_idx == true)
        x_new.cost_h.total = m_Heuristic.euclidean_joint_space_distance(x_new.config, m_config_goal_pose);
    else
        x_new.cost_h.total = m_Heuristic.euclidean_joint_space_distance(x_new.config, m_config_start_pose);


    //Return true if interpolation worked fine
    return true;


}





//Interpolate configurations of near_node and end_node
bool BiRRTstarPlanner::interpolateConfigurations(Node near_node, Node end_node, int num_points ,vector<vector<double> > &joint_traj, vector<vector<double> > &ee_traj)
{

    //Interpolation process result
    bool interpolation_success = true;


    //Start and end config
    vector<double> start_conf = near_node.config;
    vector<double> end_conf = end_node.config;

    //Store first config and ee pose
    //joint_traj.push_back(start_conf);
    //ee_traj.push_back(near_node.ee_pose);

    //    cout<<"Start config: "<<endl;
    //    for(int j = 0 ; j < m_num_joints ; j++)
    //    {
    //        cout<<start_conf[j]<<"  ";
    //    }
    //    cout<<endl;

    //Compute
    vector<double> c_space_dist(m_num_joints);
    vector<double> c_space_step_width(m_num_joints);


    //cout<<"joint step width traj: "<<endl;
    for(int j = 0 ; j < m_num_joints ; j++)
    {
        c_space_dist[j] = end_conf[j] - start_conf[j];
        c_space_step_width[j] = c_space_dist[j] / double(num_points);
        //cout<<c_space_dist[j] / double(num_points)<<endl;
    }

    //cout<<endl;

    vector<double> c_space_via_point_vector(m_num_joints);

    KDL::JntArray c_space_via_point_array;
    c_space_via_point_array = KDL::JntArray(m_num_joints);


    for(int inc = 0 ; inc <= num_points ; inc++)
    {
        //cout<<"Config on joint traj: "<<endl;
        for(int j = 0 ; j < m_num_joints ; j++)
        {
            c_space_via_point_vector[j] = start_conf[j] + inc * c_space_step_width[j];
            //cout<<c_space_via_point_vector[j]<<endl;
            c_space_via_point_array(j) = c_space_via_point_vector[j];
        }

        //cout<<endl;


        //Store joint configurations in trajectory
        joint_traj.push_back(c_space_via_point_vector);

        //Store ee pose in trajectory
        vector<double> curr_ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,c_space_via_point_array);
        ee_traj.push_back(curr_ee_pose);

//        if(inc == 0)
//        {
//            cout<<"Current config: ";
//            for(int i = 0; i< c_space_via_point_array.rows() ; i++)
//                cout<<c_space_via_point_array(i)<<"  ";
//            cout<<endl;

//            cout<<"Current ee pose: ";
//            for(int i = 0; i< curr_ee_pose.size() ; i++)
//                cout<<curr_ee_pose[i]<<"  ";
//            cout<<endl;

//        }

        //-------------------
        //Check wheather intermediate node obeys the constraints
        if(m_constraint_active == true)
        {

            //Compute task error
            vector<double> task_error_interpolation = m_RobotMotionController->compute_task_error(c_space_via_point_vector, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev);
            //vector<double> task_error_interpolation = m_RobotMotionController->compute_task_error(m_config_goal_pose, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev);
            //double task_error_interpolation_norm =m_RobotMotionController->getVectorNorm(task_error_interpolation);
            bool task_error_within_bounds = m_RobotMotionController->is_error_within_bounds(task_error_interpolation);

//            if(inc == 0)
//            {
//                cout<<"Current task_error: ";
//                for(int i = 0; i< task_error_interpolation.size() ; i++)
//                    cout<<task_error_interpolation[i]<<"  ";
//                cout<<endl;

//                int t;
//                cin>>t;
//            }

            //Check whether task error out of tolerance
            //if(m_max_task_error_interpolation < task_error_interpolation_norm)
            if(task_error_within_bounds == false)
            {
                //Set interpolation result to false
                interpolation_success = false;

                cout<<"Error for "<<inc<<"-th joint config along edge is not within constraint coordinate bounds!!"<<endl;

                //Leave interpolation loop
                break;
            }
            else
            {
                //cout<<"Error ok!!"<<endl;

            }

        }
        //-------------------

    }

    //Return Interpolation process result
    return interpolation_success;


//    cout<<"End EE pose IN:"<<endl;
//    cout<<end_node.ee_pose[0]<<endl;
//    cout<<end_node.ee_pose[1]<<endl;
//    cout<<end_node.ee_pose[2]<<endl;

//    cout<<"End EE pose Out:"<<endl;
//    cout<<ee_traj[ee_traj.size()-1][0]<<endl;
//    cout<<ee_traj[ee_traj.size()-1][1]<<endl;
//    cout<<ee_traj[ee_traj.size()-1][2]<<endl;

}



//Choose Parent for x_new minimizing the cost of reaching x_new (given the set of vertices surrounding x_new as potential parents)
bool BiRRTstarPlanner::choose_node_parent_control(Rrt_star_tree *tree, vector<int> near_vertices, Node nn_node, Edge &e_new , Node &x_new)
{
    //-- Initialize --
    bool parent_found = false;

    //Set nearest node id as parent id for new node
    x_new.parent_id = nn_node.node_id;


    //Choose Parent for Node, i.e. the one providing the lowest cost for reaching x_new from the start_node
    for (int nv = 0; nv < near_vertices.size() ; nv++)
    {
        //Do not consider goal node as parent node (goal node id = 0)
        //if (near_vertices[nv] > 0)
        //{

            //Node and edge generated by "connectNodesControl"
            Node gen_node;
            Edge gen_edge;
            //Try to connect a near vertex to x_new
            kuka_motion_controller::Status connector_result = connectNodesControl(tree, tree->nodes[near_vertices[nv]],x_new, gen_node, gen_edge);


            //Only if x_new has been reached (considering some error threshold) a potential better path (via a node other than the nearest node) has been found
            if(connector_result == kuka_motion_controller::REACHED)
            {
                //If cost is smaller than the cost of reaching x_new via it's nearest neighbour (Note: "x_new.cost_reach" initially set by expansion towards x_rand
                if(gen_node.cost_reach.total <= x_new.cost_reach.total)
                {
                    //Only if edge to x_new is valid
                    if(m_FeasibilityChecker->isEdgeValid(gen_edge))
                    {
                        //A parent for the new node has been found
                        parent_found = true;

                        //Set near vertex as parent of x_new
                        x_new.parent_id = tree->nodes[near_vertices[nv]].node_id;
                        //Set config and ee_pose of x_new to gen_node config
                        // -> done because ee_pose of x_new is defined as reached considering some cartesian threshold
                        x_new.config = gen_node.config;
                        x_new.ee_pose = gen_node.ee_pose;
                        //Update cost of reaching x_new
                        x_new.cost_reach.total = gen_node.cost_reach.total;

                        //Update edge
                        e_new = gen_edge;

                    }//End is edgevalid

                } //End Reduced cost test
            } //End Node reached test
        //} //End if exclusion goal node
    } //End iteration through near vertices


    //Compute heuristic for final x_new's ee_pose
    x_new.cost_h.total = m_Heuristic.euclidean_pose_distance(x_new.ee_pose,m_ee_goal_pose);


    //Note: when calling controller to reach x_new from near vertices the return value must be always REACHED otherwise run_VDLS_Control_Connector will
    // probably return very different x_new's

    //Changes to the edge and x_new are directly performed in this funtion

    //Return flag whether a parent has been found
    return parent_found;
}



//Choose Parent for x_new minimizing the cost of reaching x_new (given the set of vertices surrounding x_new as potential parents)
bool BiRRTstarPlanner::choose_node_parent_interpolation(Rrt_star_tree *tree, vector<int> near_vertices, Node nn_node, Edge &e_new, Node &x_new, bool show_tree_vis)
{

//    //Variable for timer
//    gettimeofday(&m_timer, NULL);
//    double time_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

    //Return if there are no near nodes
    if(near_vertices.empty())
        return false;

    //-- Initialize --
    bool parent_found = false;

    // Selected Via Nodes and Edges to x_new
    // -> only used when constraints are active and first order retraction is required to connect near vertices to x_new
    vector<Node> selected_via_nodes;
    vector<Edge> selected_via_edges;

    //Set nearest node id as parent id for new node
    x_new.parent_id = nn_node.node_id;

    //Choose Parent for Node, i.e. the one providing the lowest cost for reaching x_new from the start_node
    for (int nv = 0; nv < near_vertices.size() ; nv++)
    {

       //Leave near vertices loop when "m_max_near_nodes" nodes have been considered as potential parents of x_new
       if (m_max_near_nodes == nv)
            break;

       //Near Nodes of tree can be neglected if their cost-to-reach is already higher than the current cost of x_new, i.e. they can't provide a better path
       if(tree->nodes[near_vertices[nv]].cost_reach.total < x_new.cost_reach.total)
       {

           //No Motion Constraints Active
           if(m_constraint_active == false)
           {

               //Node and edge generated by "connectNodesInterpolation"
               Node gen_node;
               Edge gen_edge;
               //Try to connect a near vertex to x_new
               connectNodesInterpolation(tree, tree->nodes[near_vertices[nv]],x_new, m_num_traj_segments_interp, gen_node, gen_edge);

               //If cost is smaller than the cost of reaching x_new via it's nearest neighbour (Note: "x_new.cost_reach" initially set by expansion towards x_rand
               if(gen_node.cost_reach.total <= x_new.cost_reach.total)
               {
                    //Only if edge to x_new is valid
                    int last_valid_node_idx = 0; //Not used but required by m_FeasibilityChecker->isEdgeValid-Function
                    if(m_FeasibilityChecker->isEdgeValid(gen_edge,last_valid_node_idx))
                    {
                        //A parent for the new node has been found
                        parent_found = true;

                        //-----------------------------------------------------
                        //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for the lowest cost nearest neighbour)
                        int num_nodes_tree = tree->num_nodes;
                        //Get current number of nodes in the tree (required to avoid incrementing the number of edges while searching for the lowest cost nearest neighbour)
                        int num_edges_tree = tree->num_edges;

                        //Set current near node
                        Node curr_near_node = tree->nodes[near_vertices[nv]];

                        bool rand_sample_reached = false;
                        while(rand_sample_reached == false)
                        {
                            //Perform a step from current curr_near_node towards x_new using a fixed step size (returns true when random sample has been reached by expansion)
                            Node original_x_new = x_new;
                            rand_sample_reached = stepTowardsRandSample(curr_near_node, original_x_new, m_unconstraint_extend_step_factor);

                            //Connect Nodes
                            connectNodesInterpolation(tree, curr_near_node, original_x_new, m_num_traj_segments_interp, gen_node, gen_edge);

                            if(rand_sample_reached == false)
                            {
                                //Modify node data
                                gen_node.node_id = num_nodes_tree;
                                //Increment local tree nodes counter
                                num_nodes_tree++;
                                //Set near vertex as parent of gen_node
                                gen_node.parent_id = curr_near_node.node_id;

                                //Modify edge data
                                gen_edge.edge_id = num_edges_tree;
                                //Increment local tree nodes counter
                                num_edges_tree++;
                                //Reset child node ID of edge, because original original_x_new has not been reached
                                gen_edge.child_node_id = gen_node.node_id;  //Reset child node ID of edge, because original_x_new's ID is w.r.t. the other tree data structure

                                //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                                selected_via_nodes.push_back(gen_node);
                                selected_via_edges.push_back(gen_edge);

                                //Update curr_near_node in order to step further towards x_new
                                curr_near_node = gen_node;


                            }
                            else
                            {

                                //Reset Node ID
                                x_new.node_id = num_nodes_tree;
                                //Set near vertex as parent of x_new
                                x_new.parent_id = curr_near_node.node_id;
                                //Set config and ee_pose of x_new to gen_node config
                                x_new.config = gen_node.config;
                                x_new.ee_pose = gen_node.ee_pose;
                                //Update cost of reaching x_new
                                x_new.cost_reach.total = gen_node.cost_reach.total;
                                x_new.cost_reach.revolute = gen_node.cost_reach.revolute;
                                x_new.cost_reach.prismatic = gen_node.cost_reach.prismatic;


                                //Update edge
                                e_new = gen_edge;
                                e_new.edge_id = num_edges_tree;
                                e_new.child_node_id = x_new.node_id;
                            }

                        }

                        //Exit loop when first valid edge providing a lower cost path has been found
                        // -> valid because near vertices are stored in order of ascending cost_reach
                        break;

                        //-----------------------------------------------------


//                        //Set near vertex as parent of x_new
//                        x_new.parent_id = tree->nodes[near_vertices[nv]].node_id;
//                        //Set config and ee_pose of x_new to gen_node config
//                        x_new.config = gen_node.config;
//                        x_new.ee_pose = gen_node.ee_pose;
//                        //Update cost of reaching x_new
//                        x_new.cost_reach.total = gen_node.cost_reach.total;

//                        //Update edge
//                        e_new = gen_edge;

//                        //Exit loop when first valid edge providing a lower cost path has been found
//                        // -> valid because near vertices are stored in order of ascending cost_reach
//                        break;


                    } //End Reduced cost test
               } //End Node reached test
           }
           else //Motion Constraints Active
           {

               //Generated node and edge by connectNodesInterpolation
               Node gen_node;
               Edge gen_edge;

               // Via Nodes and Edges to x_new from current near vertex/node
               vector<Node> via_nodes;
               vector<Edge> via_edges;

               //Set current near node
               Node curr_x_near = tree->nodes[near_vertices[nv]];

               //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for the lowest cost nearest neighbour)
               int num_nodes_tree = tree->num_nodes;

               //Get current number of nodes in the tree (required to avoid incrementing the number of edges while searching for the lowest cost nearest neighbour)
               int num_edges_tree = tree->num_edges;


               bool new_sample_reached = false;
               while(new_sample_reached == false)
               {
                   //cout<<"In while loop"<<endl;

                   //Perform a step from curr_x_near towards x_new using a fixed step size (returns true when x_new of other tree has been reached by expansion)
                   Node exp_node_towards_x_new = x_new;
                   new_sample_reached = stepTowardsRandSample(curr_x_near, exp_node_towards_x_new, m_constraint_extend_step_factor);

                   //Projection only required if x_new has not been reached (because x_new.config is already on constraint manifold)
                   if(new_sample_reached == false)
                   {
                       //Is task frame position defined global or w.r.t near node frame
                       if(m_task_pos_global == false)
                       {
                           //Set Task frame position to position of nearest neighbour ee pose
                           m_task_frame.p.x(curr_x_near.ee_pose[0]); //Set X Position
                           m_task_frame.p.y(curr_x_near.ee_pose[1]); //Set Y Position
                           m_task_frame.p.z(curr_x_near.ee_pose[2]); //Set Z Position
                       }
                       //Is task frame orientation defined global or w.r.t near node frame
                       if(m_task_orient_global == false)
                       {
                           KDL::Rotation ee_orient = KDL::Rotation::Quaternion(curr_x_near.ee_pose[3],curr_x_near.ee_pose[4],curr_x_near.ee_pose[5],curr_x_near.ee_pose[6]);
                           m_task_frame.M = ee_orient;
                       }

                       //Project sample onto constraint manifold
                       bool projection_succeed = m_RobotMotionController->run_config_retraction(exp_node_towards_x_new.config, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev, m_max_projection_iter);


                       if(projection_succeed == true && m_FeasibilityChecker->isConfigValid(exp_node_towards_x_new.config))
                       {
                           //Check if config retraction has projected the expanded config back onto the config of the near node "curr_x_near"
                           // -> In this case the expansion is stuck, because each projected sample/config will be identical to the config of "curr_x_near"
                           double dist_near_to_projected_sample = m_Heuristic.euclidean_joint_space_distance(curr_x_near.config,exp_node_towards_x_new.config);

                           //Check additionally whether expansion has made progress towards x_new
                           double dist_curr_x_near_to_x_new = m_Heuristic.euclidean_joint_space_distance(curr_x_near.config,x_new.config);
                           double dist_projected_sample_to_x_new = m_Heuristic.euclidean_joint_space_distance(exp_node_towards_x_new.config,x_new.config);

                           if(dist_near_to_projected_sample < m_min_projection_distance || dist_curr_x_near_to_x_new < dist_projected_sample_to_x_new)
                           {
                               //if(dist_near_to_projected_sample < m_min_projection_distance)
                               //    cout<<"choose_node_parent_Interpolation: Sample projected back onto config of near node!"<<endl;
                               //else
                               //    cout<<"choose_node_parent_Interpolation: Projected sample is further away from x_new than x_near!"<<endl;
                               break;
                           }
                           else
                           {
                               //cout<<"Projection generated new sample!"<<endl;
                               //cout<<dist_near_to_projected_sample<<endl;
                           }

                           //Connect curr_x_near to projected sample
                           bool connection_result = connectNodesInterpolation(tree, curr_x_near, exp_node_towards_x_new, m_num_traj_segments_interp, gen_node, gen_edge);

                           //Check edge from curr_x_near to exp_node_towards_x_new for validity (collision check)
                           if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                           {

                               //Modify node data
                               gen_node.node_id = num_nodes_tree;
                               //Increment local tree nodes counter
                               num_nodes_tree++;
                               //Set near vertex as parent of gen_node
                               gen_node.parent_id = curr_x_near.node_id;

                               //Modify edge data
                               gen_edge.edge_id = num_edges_tree;
                               //Increment local tree nodes counter
                               num_edges_tree++;
                               //Reset child node ID of edge, because original exp_node_towards_x_new has not been reached
                               gen_edge.child_node_id = gen_node.node_id;

                               //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                               via_nodes.push_back(gen_node);
                               via_edges.push_back(gen_edge);

                               //Update curr_x_near in order to step further towards x_new
                               curr_x_near = gen_node;

                           }
                           else
                           {
                               //cout<<"Edge to projected sample invalid"<<endl;
                               break;
                           }
                        }
                       else
                       {
                           //cout<<"Projection failed"<<endl;
                           break;
                       }
                   }
                   else
                   {

                       //cout<<"Sample reached!"<<endl;

                       //Connect current curr_x_near to x_new
                       bool connection_result = connectNodesInterpolation(tree, curr_x_near, x_new, m_num_traj_segments_interp, gen_node, gen_edge);

                       //Compute solution path cost
                       //double solution_cost = gen_node.cost_reach.total + x_new.cost_reach.total;

                       //Check whether cost of path to x_new is lower thatn current cost of x_new
                       if(gen_node.cost_reach.total <= x_new.cost_reach.total)
                       {
                           //Check edge from curr_x_near to exp_node_towards_x_new for validity (collision check)
                           if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                           {
                               //A parent for the new node has been found
                               parent_found = true;

                               //Reset ID of x_new, because via nodes have been generated between the near vertex and x_new
                               x_new.node_id = num_nodes_tree;

                               //Set near vertex as parent of x_new
                               x_new.parent_id = curr_x_near.node_id;
                               //Set config and ee_pose of x_new to gen_node config
                               x_new.config = gen_node.config;
                               x_new.ee_pose = gen_node.ee_pose;
                               //Update cost of reaching x_new
                               x_new.cost_reach.total = gen_node.cost_reach.total;
                               x_new.cost_reach.revolute = gen_node.cost_reach.revolute;
                               x_new.cost_reach.prismatic = gen_node.cost_reach.prismatic;

                               //Update edge
                               e_new = gen_edge;
                               //Reset ID of x_new, because via nodes have been generated between the near vertex and x_new
                               e_new.edge_id = num_edges_tree;
                               //Required since ID of e_new has changed due to via edges entered between near vertex and x_new
                               e_new.child_node_id = x_new.node_id;

                               //Set via nodes and edges of path providing the lowest cost to x_new
                               selected_via_nodes.clear();
                               selected_via_edges.clear();
                               selected_via_nodes = via_nodes;
                               selected_via_edges = via_edges;


                               //Exit loop when first valid edge providing a lower cost path has been found
                               // -> valid because near vertices are stored in order of ascending cost_reach
                               //break;

                               //Note: Final x_new and e_new will be added to tree in main planning loop !!!(see "run_planner" function)

                           }//Edge validity check

                       }//Solution path cost check
                   }//Query Sample Reached FALSE 
               }//END Loop While Sample Not Reached

               //Exit near vertex loop when first valid edge providing a lower cost path has been found
               // -> valid because near vertices are stored in order of ascending cost_reach
               if(parent_found == true)
                    break;

           }// End Else Constraints Active

        }// END Cost check x_near.cost_reach.total < x_new.cost_reach.total
        else //We can leave the loop because all following near nodes will have higher cost-to-reach than x_new
        {
            break;
        }

    }//End iteration through near vertices



    //Add via nodes and edges to the tree if a near vertex has found to provide a lower cost path to x_new
    //if(m_constraint_active == true && parent_found == true)
    if(parent_found == true)
    {
        //Insert via nodes and edges into tree
        // -> Note: Final x_new and e_new will be added to tree in main planning loop !!!(see "run_planner" function)
        for(int i = 0 ; i < selected_via_nodes.size() ; i++)
            insertNode(tree, selected_via_edges[i] , selected_via_nodes[i], show_tree_vis);
    }


//    cout<<"Add edge to choose parent:"<<endl;
//    cout<<x_new.parent_id<<endl;

//    cout<<"Edge data choose parent:"<<endl;
//    cout<<e_new.edge_id<<endl;
//    cout<<e_new.root_node_id<<endl;
//    cout<<e_new.child_node_id<<endl;


//    cout<<"choose parent Parent data:"<<endl;
//    cout<<"Tree name: "<<tree->name<<endl;
//    cout<<tree->nodes[x_new.parent_id].node_id<<endl;
//    cout<<tree->nodes[x_new.parent_id].cost_reach.total<<endl;
//    cout<<tree->nodes[x_new.parent_id].outgoing_edges.size()<<endl;


//    //Get time elapsed
//    gettimeofday(&m_timer, NULL);
//    double time_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//    printf("choose_node_parent_interpolation %.6lf seconds elapsed\n", time_end - time_start);


    //Return flag whether a parent has been found
    return parent_found;
}



//Rewire Nodes of the Tree, i.e. checking if some if the nodes can be reached by a lower cost path
void BiRRTstarPlanner::rewireTreeControl(Rrt_star_tree *tree, vector<int> near_vertices, Node x_new, bool show_tree_vis)
{
    //Cost reduction obtained by connecting x_new to near_vertices[nn_num]
    vector<double> cost_reduction(3);
    cost_reduction[0] = 0.0;
    cost_reduction[1] = 0.0;
    cost_reduction[2] = 0.0;

    //Iterate through vertics near x_new's ee_pose
    for (int nn_num = 0 ; nn_num < near_vertices.size() ; nn_num++)
    {
        //Do not consider the parent node of x_new for rewiring operation
        if(near_vertices[nn_num] != x_new.parent_id)
        {
            //Node and edge generated by "connectNodesControl"
            Node gen_node;
            Edge gen_edge;
            //Try to connect ee_pose of x_new to ee_pose of a near vertex
            kuka_motion_controller::Status connector_result = connectNodesControl(tree, x_new, tree->nodes[near_vertices[nn_num]], gen_node, gen_edge);

            if(connector_result == kuka_motion_controller::REACHED)
            {
                //cout<<"Rewire: Node reached!!!"<<endl;
                //cout<<"Old cost: "<<tree->nodes[near_vertices[nn_num]].cost_reach.total<<endl;
                //cout<<"New cost: "<<gen_node.cost_reach.total<<endl;

                //If cost of reaching the near vertex via x_new is smaller than the current cost to reach the near vertex
                if(gen_node.cost_reach.total < tree->nodes[near_vertices[nn_num]].cost_reach.total)
                {
                    //cout<<"Rewire: Lower cost found!!!"<<endl;

                    //Compute cost reduction
                    cost_reduction[0] = gen_node.cost_reach.total - tree->nodes[near_vertices[nn_num]].cost_reach.total;


                    //cout<<"Parent Node ID: "<<tree->nodes[near_vertices[nn_num]].parent_id<<endl;
                    //cout<<"Number of outgoing edges: "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.size()<<endl;


                    //Index of the outgoing edge from near_vertices[nn_num].parent_id to be erased
                    int out_edge_erase_index = 0;

                    //Delete edge leading from previous parent of near node to the near node
                    for( int eg_num = 0 ; eg_num < tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.size() ; eg_num++)
                    {

                        //Find edge in parent node leading to near node
                        if(tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].child_node_id == tree->nodes[near_vertices[nn_num]].node_id)
                        {
                            //cout<<"Rewire: Removing edge!!!"<<endl;

                            //Remember which edge needs to be erased
                            out_edge_erase_index = eg_num;

                            //cout<<"Edge: "<<tree->nodes[near_vertices[nn_num]].parent_id<<" to "<<tree->nodes[near_vertices[nn_num]].node_id<<" removed" <<endl;


                            //Decrement total numbe of edges in tree
                            tree->num_edges--;

                            //Leave loop when edge of current parent node leading to the near vertex has been found
                            break;

                        }

                    }

                    //RRT* Tree Visu.
                    if(show_tree_vis == true)
                    {
                        //Remove edge RRT* edge marker array
                        remove_tree_edge_vis(tree->name, tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index]);
                    }

                    //Assign ID of removed edge to new edge
                    gen_edge.edge_id = tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index].edge_id;
                    //Remove edge from RRT* tree
                    tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.erase(tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.begin() + out_edge_erase_index);



                    //Modify data of near node
                    // -> Update Parent ID
                    tree->nodes[near_vertices[nn_num]].parent_id = x_new.node_id;

                    // -> Update Config (now slightly different from previous one due to controller reach tolerance)
                    tree->nodes[near_vertices[nn_num]].config = gen_node.config;

                    // -> Update EE Pose (now slightly different from previous one due to controller reach tolerance)
                    tree->nodes[near_vertices[nn_num]].ee_pose = gen_node.ee_pose;

                    // -> Update Heuristic value (now slightly different from previous one due to controller reach tolerance)
                    tree->nodes[near_vertices[nn_num]].cost_h.total = m_Heuristic.euclidean_pose_distance(gen_node.ee_pose, m_ee_goal_pose);

                    //Add outgoing edge to x_new (leading to near node)
                    tree->nodes[x_new.node_id].outgoing_edges.push_back(gen_edge);


                    // -> Update cost to reach near_vertex via x_new node and the cost following the near vertex
                    //tree->nodes[near_vertices[nn_num].node_id].cost_reach.total = gen_node.cost_reach.total;
                    recursiveNodeCostUpdate(tree, tree->nodes[near_vertices[nn_num]], cost_reduction, show_tree_vis);


                    //Clear array of nodes visited in recursiveNodeCostUpdate
                    testing_.clear();



                    //RRT* Tree Visu.
                    if(show_tree_vis == true)
                    {
                        //Add edge to tree visualization
                        add_tree_edge_vis(tree->name, gen_edge);
                    }


                    //Increment total number of edges in tree, because a new edge is added (to the goal node) without removing an edge previously
                    tree->num_edges++;


                    //Incement number of rewire operations
                    tree->num_rewire_operations++;


                    //cout<<"A Node has been rewired"<<endl;

                    //cout<<"Edge: "<<x_new.node_id<<" to "<<near_vertices[nn_num]<<" added" <<endl;


                } // end if new cost_reach smaller than current cost_reach
            } //End if connector state
        } //End if exclusion parent node
    } //End iteration through near nodes

}




//Rewire Nodes of the Tree, i.e. checking if some if the nodes can be reached by a lower cost path
void BiRRTstarPlanner::rewireTreeInterpolation(Rrt_star_tree *tree, vector<int> near_vertices, Node x_new, bool show_tree_vis)
{

//    //Variable for timer
//    gettimeofday(&m_timer, NULL);
//    double time_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);


//    //Cost reduction obtained by connecting x_new to near_vertices[nn_num] (total + for revolute and primatic components)
//    vector<double> cost_reduction(3);
//    cost_reduction[0]= 0.0; //total
//    cost_reduction[1]= 0.0; //revolute
//    cost_reduction[2]= 0.0; //prismatic

    //Number of near nodes
    int num_near_nodes = near_vertices.size();

    //Return if there are no near nodes
    if(near_vertices.empty())
    {
        //cout<<"No near vertices"<<endl;
        return ;
    }


    //Local nodes and edge counters (used when planning with constraints, thereby generating temporary via nodes and edges)
    int num_nodes_tree = 0;
    int num_edges_tree = 0;


    //Determine how many of the near nodes need to be considered for the rewire operation
    int near_vertices_counter = 0;
    //Check whether the number of near vertices is higher than the m_max_near_nodes to avoid negative indices
    int lower_index = num_near_nodes >= m_max_near_nodes ? (num_near_nodes-m_max_near_nodes) : 0;

    for (int nn_num = num_near_nodes-1 ; nn_num >= lower_index  ; nn_num--)
    {
        //Near Nodes of tree can be neglected for the REWIRE operation if their cost-to-reach is already ...
        // lower than the cost-to-reach of x_new, i.e. x_new cannot provide an alternative path of lower cost to the near node
        if(x_new.cost_reach.total < tree->nodes[near_vertices[nn_num]].cost_reach.total)
        {
            near_vertices_counter++;
        }
    }


    //Iterate through vertics near x_new's ee_pose
    //#pragma omp parallel for //Paralellize the rewire operation
    for (int nn_num = num_near_nodes-1 ; nn_num >= (num_near_nodes-near_vertices_counter)  ; nn_num--)
    {
        //TODO:
        // - Rewire only nodes that are inside the ellipsoidal subset
        //


        //Do not consider
        // -> the parent node of x_new for rewiring operation
        // -> the near nodes that are connected to the start node (there can't be a new connection via x_new to them providing lower cost)
        if(near_vertices[nn_num] != x_new.parent_id && tree->nodes[near_vertices[nn_num]].parent_id != 0)
        {
            //Near Nodes of tree can be neglected for the REWIRE operation if their cost-to-reach is already ...
            // lower than the cost-to-reach of x_new, i.e. x_new cannot provide an alternative path of lower cost to the near node
            //if(x_new.cost_reach.total < tree->nodes[near_vertices[nn_num]].cost_reach.total)
            //{

                //If no constraints are active (simple linear interpolation, no first order retraction , i.e. sample projection, required)
                if(m_constraint_active == false)
                {
                    //Node and edge generated by "connectNodesControl"
                    Node gen_node;
                    Edge gen_edge;
                    //Try to connect ee_pose of x_new to ee_pose of a near vertex
                    connectNodesInterpolation(tree, x_new, tree->nodes[near_vertices[nn_num]], m_num_traj_segments_interp, gen_node, gen_edge);

                    //If cost of reaching the near vertex via x_new is smaller than the current cost to reach the near vertex
                    if(gen_node.cost_reach.total < tree->nodes[near_vertices[nn_num]].cost_reach.total)
                    {
                        //cout<<"Rewire: Node reached!!!"<<endl;
                        //cout<<"Old cost: "<<tree->nodes[near_vertices[nn_num]].cost_reach.total<<endl;
                        //cout<<"New cost: "<<gen_node.cost_reach.total<<endl;

                        //If edge from x_new to near vertex is valid
                        //int last_valid_node_idx = 0; //Not used but required by m_FeasibilityChecker->isEdgeValid-Function
                        if(m_FeasibilityChecker->isEdgeValid(gen_edge))
                        {
                            //cout<<"Rewire: Lower cost found!!!"<<endl;

                            //Compute cost reduction (total + for revolute and primatic components)
                            // -> used to update cost of children nodes
                            vector<double> cost_reduction(3);
                            cost_reduction[0] = gen_node.cost_reach.total - tree->nodes[near_vertices[nn_num]].cost_reach.total;
                            cost_reduction[1] = gen_node.cost_reach.revolute - tree->nodes[near_vertices[nn_num]].cost_reach.revolute;
                            cost_reduction[2] = gen_node.cost_reach.prismatic - tree->nodes[near_vertices[nn_num]].cost_reach.prismatic;

                            //Index of the outgoing edge from tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id] to be erased
                            int out_edge_erase_index = -1;

                            //Delete edge leading from previous parent of near node to the near node
                            for( int eg_num = 0 ; eg_num < tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.size() ; eg_num++)
                            {

                                //cout<<"Edge: "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].root_node_id<<" to "<<tree->nodes[near_vertices[nn_num].parent_id].outgoing_edges[eg_num].child_node_id<<endl;

                                //Find edge in parent node leading to near node
                                if(tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].child_node_id == tree->nodes[near_vertices[nn_num]].node_id)
                                {
                                    //cout<<"Rewire: Removing edge!!!"<<endl;

                                    //Remember which edge needs to be erased
                                    out_edge_erase_index = eg_num;

                                    //cout<<"Edge: "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].root_node_id<<" to "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].child_node_id<<" removed from node: "<< tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].node_id<<endl;


                                    //Decrement total numbe of edges in tree
                                    tree->num_edges--;

                                    //Leave loop when edge of current parent node leading to the near vertex has been found
                                    break;
                                }

                            }


                            if(out_edge_erase_index == -1)
                            {
                                cout<<"Node ID: "<<tree->nodes[near_vertices[nn_num]].node_id<<endl;
                                cout<<"No edge leading to near vertex found!!!"<<endl;
                                int zet;
                                cin>>zet;
                            }

                            //RRT* Tree Visu.
                            if(show_tree_vis == true)
                            {
                                //Remove edge RRT* edge marker array
                                remove_tree_edge_vis(tree->name, tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index]);
                            }

                            //Assign ID of removed edge to new edge
                            gen_edge.edge_id = tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index].edge_id;
                            //Remove edge from RRT* tree
                            //#pragma omp critical(edgeoperation)
                            tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.erase(tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.begin() + out_edge_erase_index);


                            //Modify data of near node
                            // -> Update Parent ID in tree
                            tree->nodes[near_vertices[nn_num]].parent_id = x_new.node_id;

                            //Check whether one of the connecting nodes has been rewired (-> we need to update their parent)
                            if(m_solution_path_available == true)
                            {
                                // -> Update Parent ID if rewired node is on of the nodes establishing the connecting between the trees
                                if(tree->nodes[near_vertices[nn_num]].node_id == m_node_tree_B.node_id && tree->name == m_connected_tree_name)
                                    m_node_tree_B.parent_id = x_new.node_id;
                                else if(tree->nodes[near_vertices[nn_num]].node_id == m_node_tree_A.node_id && tree->name != m_connected_tree_name)
                                    m_node_tree_A.parent_id = x_new.node_id;
                                else
                                {
                                    //cout<<"Rewired node is not one of the tree connecting nodes"<<endl;
                                }
                            }


                            // -> Update Config (now slightly different from previous one due to controller reach tolerance)
                            tree->nodes[near_vertices[nn_num]].config = gen_node.config;

                            // -> Update EE Pose (now slightly different from previous one due to controller reach tolerance)
                            tree->nodes[near_vertices[nn_num]].ee_pose = gen_node.ee_pose;

                            // -> Update Heuristic value (now slightly different from previous one due to controller reach tolerance)
                            if(tree->name == "START")
                                tree->nodes[near_vertices[nn_num]].cost_h.total = m_Heuristic.euclidean_joint_space_distance(gen_node.config, m_config_goal_pose);
                            else
                                tree->nodes[near_vertices[nn_num]].cost_h.total = m_Heuristic.euclidean_joint_space_distance(gen_node.config, m_config_start_pose);

                            //Add outgoing edge to x_new (leading to near node)
                            //#pragma omp critical(edgeoperation) //Only one thread can add an outgoing edge from the new node at a time
                            tree->nodes[x_new.node_id].outgoing_edges.push_back(gen_edge);


                            //cout<<"Edge: "<<gen_edge.root_node_id<<" to "<<gen_edge.child_node_id<<" added to node: "<< x_new.node_id<<endl;


                            // -> Update cost to reach near_vertex via x_new node and the cost following the near vertex
                            //tree->nodes[near_vertices[nn_num].node_id].cost_reach.total = gen_node.cost_reach.total;
                            //#pragma omp critical(edgeoperation)
                            recursiveNodeCostUpdate(tree, tree->nodes[near_vertices[nn_num]], cost_reduction, show_tree_vis);


                            //Clear array of nodes visited in recursiveNodeCostUpdate
                            testing_.clear();


                            //RRT* Tree Visu.
                            if(show_tree_vis == true)
                            {
                                //Add edge to tree visualization
                                add_tree_edge_vis(tree->name, gen_edge);
                            }


                            //Increment total number of edges in tree, because a new edge is added (to the goal node) without removing an edge previously
                            tree->num_edges++;

                            //Incement number of rewire operations
                            tree->num_rewire_operations++;


                        } // end edge valid check
                    } //End if cost check connection via x_new (i.e. x_new.cost_reach.total + edge_cost < near_node.cost_reach.total)
                }//END if Constraint Active == FALSE
                else  //Constraint Active == TRUE
                {

                    //Generated node and edge by connectNodesInterpolation
                    Node gen_node;
                    Edge gen_edge;

                    // Via Nodes and Edges to x_new from current near vertex/node
                    vector<Node> via_nodes;
                    vector<Edge> via_edges;

                    //Set current x_new node
                    Node curr_x_new = x_new;

                    //#pragma omp critical(insertNodeOperation)
                    //{
                        //Get current number of nodes in the tree (required to avoid incrementing the number of nodes while searching for a lowest cost path to x_near via x_new)
                        num_nodes_tree = tree->num_nodes;

                        //Get current number of edges in the tree (required to avoid incrementing the number of edges while searching for a lowest cost path to x_near via x_new)
                        num_edges_tree = tree->num_edges;
                    //}

//                    //Variable for timer
//                    gettimeofday(&m_timer, NULL);
//                    double time_while_loop_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

//                    double total_time_config_retraction = 0.0;
//                    double total_time_config_interpolation = 0.0;
//                    double total_time_recursive_cost_update = 0.0;
//                    double total_time_progress_check = 0.0;
//                    double total_time_step_towards_random_sample = 0.0;
//                    double total_time_add_remove_edge = 0.0;
//                    double total_time_collision_check = 0.0;
//                    int total_num_iter_while_loop = 0;



                    bool new_sample_reached = false;
                    while(new_sample_reached == false)
                    {
                        //total_num_iter_while_loop++;

                        //Variable for timer
                        //gettimeofday(&m_timer, NULL);
                        //double time_step_towards_rand_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);


                        //Perform a step from curr_x_new towards x_near using a fixed step size (returns true when x_near has been reached by expansion)
                        Node exp_node_towards_x_near = tree->nodes[near_vertices[nn_num]];
                        new_sample_reached = stepTowardsRandSample(curr_x_new, exp_node_towards_x_near, m_constraint_extend_step_factor);

                        //Get time elapsed
                        //gettimeofday(&m_timer, NULL);
                        //double time_step_towards_rand_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                        //total_time_step_towards_random_sample += (time_step_towards_rand_end - time_step_towards_rand_start);

                        //Projection only required if x_new has not been reached (because x_new.config is already on constraint manifold)
                        if(new_sample_reached == false)
                        {
                            //Is task frame position defined global or w.r.t near node frame
                            if(m_task_pos_global == false)
                            {
                                //Set Task frame position to position of nearest neighbour ee pose
                                m_task_frame.p.x(curr_x_new.ee_pose[0]); //Set X Position
                                m_task_frame.p.y(curr_x_new.ee_pose[1]); //Set Y Position
                                m_task_frame.p.z(curr_x_new.ee_pose[2]); //Set Z Position
                            }
                            //Is task frame orientation defined global or w.r.t near node frame
                            if(m_task_orient_global == false)
                            {
                                KDL::Rotation ee_orient = KDL::Rotation::Quaternion(curr_x_new.ee_pose[3],curr_x_new.ee_pose[4],curr_x_new.ee_pose[5],curr_x_new.ee_pose[6]);
                                m_task_frame.M = ee_orient;
                            }


                            //Variable for timer
                            //gettimeofday(&m_timer, NULL);
                            //double time_config_retraction_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                            //Project sample onto constraint manifold
                            bool projection_succeed = m_RobotMotionController->run_config_retraction(exp_node_towards_x_near.config, m_task_frame, m_grasp_transform, m_constraint_vector, m_coordinate_dev, m_max_projection_iter);

                            //Get time elapsed
                            //gettimeofday(&m_timer, NULL);
                            //double time_config_retraction_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                            //total_time_config_retraction += (time_config_retraction_end - time_config_retraction_start);

                            //Variable for timer
                            //gettimeofday(&m_timer, NULL);
                            //double time_collision_check_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                            //Check whether projection succeded and whether the resulting config is valid
                            if(projection_succeed == true && m_FeasibilityChecker->isConfigValid(exp_node_towards_x_near.config))
                            {

                                //Get time elapsed
                                //gettimeofday(&m_timer, NULL);
                                //double time_collision_check_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                                //total_time_collision_check += (time_collision_check_end - time_collision_check_start);

                                //Variable for timer
                                //gettimeofday(&m_timer, NULL);
                                //double time_progress_check_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                                //Check if config retraction has projected the expanded config back onto the config of the new node "curr_x_new"
                                // -> In this case the expansion is stuck, because each projected sample/config will be identical to the config of "curr_x_new"
                                double dist_near_to_projected_sample = m_Heuristic.euclidean_joint_space_distance(curr_x_new.config,exp_node_towards_x_near.config);

                                //Check additionally whether expansion has made progress towards x_near
                                double dist_curr_x_new_to_x_near = m_Heuristic.euclidean_joint_space_distance(curr_x_new.config,tree->nodes[near_vertices[nn_num]].config);
                                double dist_projected_sample_to_x_near = m_Heuristic.euclidean_joint_space_distance(exp_node_towards_x_near.config,tree->nodes[near_vertices[nn_num]].config);

                                if(dist_near_to_projected_sample < m_min_projection_distance || dist_curr_x_new_to_x_near < dist_projected_sample_to_x_near)
                                {
                                    //cout<<"Sample projected back onto config of new node, Expansion stuck, Exit Expansion!"<<endl;
                                    break;
                                }
                                else
                                {
                                    //cout<<"Projection generated new sample!"<<endl;
                                    //cout<<dist_near_to_projected_sample<<endl;
                                }

                                //Get time elapsed
                                //gettimeofday(&m_timer, NULL);
                                //double time_progress_check_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                                //total_time_progress_check += (time_progress_check_end - time_progress_check_start);


                                //Variable for timer
                                //gettimeofday(&m_timer, NULL);
                                //double time_nodes_interpolation_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);


                                //Connect curr_x_new to projected sample
                                bool connection_result = connectNodesInterpolation(tree, curr_x_new, exp_node_towards_x_near, m_num_traj_segments_interp, gen_node, gen_edge);

                                //Get time elapsed
                                //gettimeofday(&m_timer, NULL);
                                //double time_nodes_interpolation_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                                //total_time_config_interpolation += (time_nodes_interpolation_end - time_nodes_interpolation_start);

                                //Check whether cost of projected sample is lower than current cost of x_near
                                if(gen_node.cost_reach.total <= tree->nodes[near_vertices[nn_num]].cost_reach.total)
                                {
                                    //Variable for timer
                                    //gettimeofday(&m_timer, NULL);
                                    //double time_collision_check2_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                                    //Check edge from curr_x_new to exp_node_towards_x_near for validity (collision check)
                                    if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                                    {
                                        //Get time elapsed
                                        //gettimeofday(&m_timer, NULL);
                                        //double time_collision_check2_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                                        //total_time_collision_check += (time_collision_check2_end - time_collision_check2_start);


                                        //#pragma omp critical(addViaNode)
                                        //{
                                            //Modify node data
                                            gen_node.node_id = num_nodes_tree;
                                            //Increment local tree nodes counter
                                            num_nodes_tree++;
                                        //}
                                        //Set curr_x_new vertex as parent of gen_node
                                        gen_node.parent_id = curr_x_new.node_id;

                                        //#pragma omp critical(addViaEdge)
                                        //{
                                            //Modify edge data
                                            gen_edge.edge_id = num_edges_tree;
                                            //Increment local tree nodes counter
                                            num_edges_tree++;
                                        //}
                                        //Reset child node ID of edge, because original exp_node_towards_x_near has not been reached
                                        gen_edge.child_node_id = gen_node.node_id;

                                        //Collect current expanded node and edge (later added to the tree if x_new has been reached by a legal path)
                                        via_nodes.push_back(gen_node);
                                        via_edges.push_back(gen_edge);

                                        //Update curr_x_new in order to step further towards x_near
                                        curr_x_new = gen_node;

                                    }
                                    else
                                    {
                                        //cout<<"Edge to projected sample invalid"<<endl;
                                        break;
                                    }
                                }
                                else
                                {
                                    //cout<<"Cost of Projected sample exeeded the cost of near node"<<endl;
                                    break;
                                }
                            }
                            else
                            {
                                //cout<<"Projection failed"<<endl;
                                break;
                            }
                        }
                        else
                        {

                            //cout<<"Near Vertex reached!"<<endl;

                            //Connect current curr_x_new to x_new
                            bool connection_result = connectNodesInterpolation(tree, curr_x_new, tree->nodes[near_vertices[nn_num]], m_num_traj_segments_interp, gen_node, gen_edge);

                            //Check whether cost of reaching x_near via x_new is lower than current cost of x_near
                            if(gen_node.cost_reach.total <= tree->nodes[near_vertices[nn_num]].cost_reach.total)
                            {
                                //Check edge from curr_x_new to exp_node_towards_x_near for validity (collision check)
                                if(connection_result == true && m_FeasibilityChecker->isEdgeValid(gen_edge))
                                {
                                    //Variable for timer
                                    //gettimeofday(&m_timer, NULL);
                                    //double time_add_remove_edge_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                                    //Compute cost reduction (total + for revolute and primatic components)
                                    // -> used to update cost of children nodes
                                    vector<double> cost_reduction(3);
                                    cost_reduction[0] = gen_node.cost_reach.total - tree->nodes[near_vertices[nn_num]].cost_reach.total;
                                    cost_reduction[1] = gen_node.cost_reach.revolute - tree->nodes[near_vertices[nn_num]].cost_reach.revolute;
                                    cost_reduction[2] = gen_node.cost_reach.prismatic - tree->nodes[near_vertices[nn_num]].cost_reach.prismatic;


                                    //Index of the outgoing edge from tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id] to be erased
                                    int out_edge_erase_index = -1;

                                    //Delete edge leading from previous parent of near node to the near node
                                    for( int eg_num = 0 ; eg_num < tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.size() ; eg_num++)
                                    {

                                        //cout<<"Edge: "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].root_node_id<<" to "<<tree->nodes[near_vertices[nn_num].parent_id].outgoing_edges[eg_num].child_node_id<<endl;

                                        //Find edge in parent node leading to near node
                                        if(tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].child_node_id == tree->nodes[near_vertices[nn_num]].node_id)
                                        {
                                            //cout<<"Rewire: Removing edge!!!"<<endl;

                                            //Remember which edge needs to be erased
                                            out_edge_erase_index = eg_num;

                                            //cout<<"Edge: "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].root_node_id<<" to "<<tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[eg_num].child_node_id<<" removed from node: "<< tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].node_id<<endl;


                                            //Decrement total number of edges in tree
                                            tree->num_edges--;

                                            //Leave loop when edge of current parent node leading to the near vertex has been found
                                            break;
                                        }

                                    }

                                    if(out_edge_erase_index == -1)
                                    {
                                        cout<<"Node ID: "<<tree->nodes[near_vertices[nn_num]].node_id<<endl;
                                        cout<<"No edge leading to near vertex found!!!"<<endl;
                                        int zet;
                                        cin>>zet;
                                    }


                                    //Assign ID of removed edge to new edge
                                    gen_edge.edge_id = tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index].edge_id;

                                    //RRT* Tree Visu.
                                    if(show_tree_vis == true)
                                    {                                     
                                        //#pragma omp critical(insertNodeOperation)
                                        //{
                                            //Remove edge RRT* edge marker array
                                            remove_tree_edge_vis(tree->name, tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index]);
                                            //Add edge to tree visualization
                                            add_tree_edge_vis(tree->name, gen_edge);
                                        //}
                                    }

                                    //Assign ID of removed edge to new edge
                                    //gen_edge.edge_id = tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges[out_edge_erase_index].edge_id;


                                    //Remove edge from RRT* tree
                                    //#pragma omp critical(insertNodeOperation)
                                    tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.erase(tree->nodes[tree->nodes[near_vertices[nn_num]].parent_id].outgoing_edges.begin() + out_edge_erase_index);


                                    //Modify data of near node
                                    // -> Update Parent ID in tree
                                    tree->nodes[near_vertices[nn_num]].parent_id = curr_x_new.node_id;

                                    //Check whether one of the connecting nodes has been rewired (-> we need to update their parent)
                                    if(m_solution_path_available == true)
                                    {
                                        // -> Update Parent ID if rewired node is on of the nodes establishing the connecting between the trees
                                        if(tree->nodes[near_vertices[nn_num]].node_id == m_node_tree_B.node_id && tree->name == m_connected_tree_name)
                                            m_node_tree_B.parent_id = curr_x_new.node_id;
                                        else if(tree->nodes[near_vertices[nn_num]].node_id == m_node_tree_A.node_id && tree->name != m_connected_tree_name)
                                            m_node_tree_A.parent_id = curr_x_new.node_id;
                                        else
                                        {
                                            //cout<<"Rewired node is not one of the tree connecting nodes"<<endl;
                                        }
                                    }

                                    // -> Update Config (now slightly different from previous one due to controller reach tolerance)
                                    tree->nodes[near_vertices[nn_num]].config = gen_node.config;

                                    // -> Update EE Pose (now slightly different from previous one due to controller reach tolerance)
                                    tree->nodes[near_vertices[nn_num]].ee_pose = gen_node.ee_pose;

                                    // -> Update Heuristic value (now slightly different from previous one due to controller reach tolerance)
                                    if(tree->name == "START")
                                        tree->nodes[near_vertices[nn_num]].cost_h.total = m_Heuristic.euclidean_joint_space_distance(gen_node.config, m_config_goal_pose);
                                    else
                                        tree->nodes[near_vertices[nn_num]].cost_h.total = m_Heuristic.euclidean_joint_space_distance(gen_node.config, m_config_start_pose);



//                                    //RRT* Tree Visu.
//                                    if(show_tree_vis == true)
//                                    {
//                                        //cout<<" edge to visu added in REWIRE"<<endl;
//                                        //Add edge to tree visualization
//                                        #pragma omp critical(insertNodeOperation)
//                                        add_tree_edge_vis(tree->name, gen_edge);
//                                    }


                                    //Insert via nodes and edges into tree (generated by stepping from x_new to x_near + first order projection)
                                    //#pragma omp critical(insertNodeOperation)
                                    //{
                                        for(int i = 0 ; i < via_nodes.size() ; i++)
                                        {
                                            //cout<<"insert node with id: "<<via_nodes[i].node_id<<endl;
                                            //cout<<"insert edge with id: "<<via_edges[i].edge_id<<endl;
                                            insertNode(tree, via_edges[i] , via_nodes[i], show_tree_vis);

                                        }
                                        //Add final outgoing edge connecting curr_x_new to x_near to the tree structure
                                        tree->nodes[curr_x_new.node_id].outgoing_edges.push_back(gen_edge);

                                        //Increment total number of edges in tree, because a new edge connecting curr_x_new to x_near has been is added
                                        //Note: all other edges are added in insertNode above + num_edges counter is incremented there
                                        tree->num_edges++;
                                    //}

                                    //Get time elapsed
                                    //gettimeofday(&m_timer, NULL);
                                    //double time_add_remove_edge_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                                    //total_time_add_remove_edge += (time_add_remove_edge_end - time_add_remove_edge_start);



                                    //Variable for timer
                                    //gettimeofday(&m_timer, NULL);
                                    //double time_cost_update_start = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);

                                    // -> Update cost to reach near_vertex via x_new node and the cost following the near vertex
                                    //tree->nodes[near_vertices[nn_num].node_id].cost_reach.total = gen_node.cost_reach.total;
                                    //#pragma omp critical(edgeoperation)

                                    recursiveNodeCostUpdate(tree, tree->nodes[near_vertices[nn_num]], cost_reduction, show_tree_vis);

                                    //Get time elapsed
                                    //gettimeofday(&m_timer, NULL);
                                    //double time_cost_update_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
                                    //total_time_recursive_cost_update += (time_cost_update_end - time_cost_update_start);


                                    //Clear array of nodes visited in recursiveNodeCostUpdate
                                    testing_.clear();


                                    //Incement number of rewire operations
                                    tree->num_rewire_operations++;


                                }//Edge validity check

                            }//Solution path cost check
                        }//Query Sample Reached FALSE
                    }//END Loop While Sample Not Reached

//                    //Get time elapsed
//                    gettimeofday(&m_timer, NULL);
//                    double time_while_loop_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//                    double total_time_while_loop = time_while_loop_end - time_while_loop_start;
//                    printf("rewireTreeInterpolation (While loop iterations) %i iterations\n", total_num_iter_while_loop);
//                    printf("rewireTreeInterpolation (While loop) %.6lf seconds elapsed\n", total_time_while_loop);
//                    printf("rewireTreeInterpolation (total time config retraction) %.6lf seconds elapsed\n", total_time_config_retraction);
//                    printf("rewireTreeInterpolation (total time config interpolation) %.6lf seconds elapsed\n", total_time_config_interpolation);
//                    printf("rewireTreeInterpolation (total time recursive cost update) %.6lf seconds elapsed\n", total_time_recursive_cost_update);
//                    printf("rewireTreeInterpolation (total time progress check) %.6lf seconds elapsed\n", total_time_progress_check);
//                    printf("rewireTreeInterpolation (total time step towards sample) %.6lf seconds elapsed\n", total_time_step_towards_random_sample);
//                    printf("rewireTreeInterpolation (total time add remove edge) %.6lf seconds elapsed\n", total_time_add_remove_edge);
//                    printf("rewireTreeInterpolation (total time collision checks) %.6lf seconds elapsed\n", total_time_collision_check);

//                    if(total_time_while_loop > 3.0)
//                    {
//                        cout<<"Stopped due to long while loop"<<endl;
//                        int z;
//                        cin>>z;
//                    }


                }//END else Constraint Active

//          }//End Cost check x_new.cost_reach.total < near_node.cost_reach.total
//          else //We can leave the loop since all following near nodes have lower cost-to-reach than x_new (rewire cannot improve path cost to near nodes)
//          {
//            break;
//          }
        } //End if exclusion parent and root node
    } //End iteration through near nodes


//    //Get time elapsed
//    gettimeofday(&m_timer, NULL);
//    double time_end = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
//    printf("rewireTreeInterpolation %.6lf seconds elapsed\n", time_end - time_start);

}




//Update cost of nodes following to near_vertices[nn_num] (needs to be done since all following nodes will have reduced cost now)
void BiRRTstarPlanner::recursiveNodeCostUpdate(Rrt_star_tree *tree, Node tree_vertex, vector<double> cost_reduction, bool show_tree_vis)
{ 

    //----- Testing for Loops in the Tree ------------
    for (int ids = 0 ; ids < testing_.size() ; ids++)
    {
        //cout<<testing_[ids]<<"  "<<tree->nodes[testing_[ids]].cost_reach.total<<endl;
        if(testing_[ids] == tree_vertex.node_id)
        {
            cout<<"ERROR Node: "<<tree_vertex.node_id<<" REVISITED"<<endl;
            cout<<"There's a loop in the tree!!!!!!!!!"<<endl;

            cout<<"Parent Node: "<<tree_vertex.parent_id<<endl;

            int t;
            cin>>t;
        }
    }


   testing_.push_back(tree_vertex.node_id);
    //----- Testing for Loops in the Tree ------------



   //Update cost to reach current node
   tree->nodes[tree_vertex.node_id].cost_reach.total = tree_vertex.cost_reach.total + cost_reduction[0];        //new total cost
   tree->nodes[tree_vertex.node_id].cost_reach.revolute = tree_vertex.cost_reach.revolute + cost_reduction[1];  //new cost for revolute joints
   tree->nodes[tree_vertex.node_id].cost_reach.prismatic = tree_vertex.cost_reach.prismatic + cost_reduction[2];//new cost for prismatic joints



   //Check if solution path is affected by the rewired node (that is the case if the rewired node is on the solution path)
   if(m_solution_path_available == true)
   {
       if(tree_vertex.node_id == m_node_tree_B.node_id && tree->name == m_connected_tree_name)
       {

           //Set global solution path cost(total + for revolute and prismatic joints)
           m_cost_best_solution_path = m_cost_best_solution_path + cost_reduction[0];
           m_cost_best_solution_path_revolute = m_cost_best_solution_path_revolute + cost_reduction[1];
           m_cost_best_solution_path_prismatic =  m_cost_best_solution_path_prismatic + cost_reduction[2];


           //Update cost of node from "m_connected_tree_name"
           m_node_tree_B.cost_reach.total = tree->nodes[tree_vertex.node_id].cost_reach.total;
           m_node_tree_B.cost_reach.revolute = tree->nodes[tree_vertex.node_id].cost_reach.revolute;
           m_node_tree_B.cost_reach.prismatic = tree->nodes[tree_vertex.node_id].cost_reach.prismatic;

           //Set last solution found to current planning iteration
           m_last_solution_iter = m_executed_planner_iter;

           //Update time when last solution has been found
           gettimeofday(&m_timer, NULL);
           double sol_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
           m_time_last_solution = sol_time - m_time_planning_start;

           //Output
           //cout<<"**New Best Solution Path (through REWIRE)**"<<endl;
           //cout<<"Total Cost: "<<m_cost_best_solution_path<<endl;
           //cout<<"Revolute Cost: "<<m_cost_best_solution_path_revolute<<endl;
           //cout<<"Prismatic Cost: "<<m_cost_best_solution_path_prismatic<<endl;

           ROS_INFO_STREAM("New Best Solution Path (through REWIRE)");


           //Show updated solution path (updated by rewireing a node on the solution path)
           if (show_tree_vis == true)
           {
               //Perform search for best solution path and shows it as a line strip
               // -> "m_node_tree_B" (from tree_B) or "m_node_tree_A" (from tree_A) has an updated cost due to a parent that has been rewired
               showCurrentSolutionPath(m_connected_tree_name, m_node_tree_B, m_node_tree_A);
           }

       }


       if(tree_vertex.node_id == m_node_tree_A.node_id && tree->name != m_connected_tree_name)
       {

           //Set global solution path cost(total + for revolute and prismatic joints)
           m_cost_best_solution_path = m_cost_best_solution_path + cost_reduction[0];
           m_cost_best_solution_path_revolute = m_cost_best_solution_path_revolute + cost_reduction[1];
           m_cost_best_solution_path_prismatic =  m_cost_best_solution_path_prismatic + cost_reduction[2];

           //Update cost of node that has been reached by "m_connected_tree_name"
           m_node_tree_A.cost_reach.total = tree->nodes[tree_vertex.node_id].cost_reach.total;
           m_node_tree_A.cost_reach.revolute = tree->nodes[tree_vertex.node_id].cost_reach.revolute;
           m_node_tree_A.cost_reach.prismatic = tree->nodes[tree_vertex.node_id].cost_reach.prismatic;

           //Set last solution found to current planning iteration
           m_last_solution_iter = m_executed_planner_iter;

           //Update time when last solution has been found
           gettimeofday(&m_timer, NULL);
           double sol_time = m_timer.tv_sec+(m_timer.tv_usec/1000000.0);
           m_time_last_solution = sol_time - m_time_planning_start;

           //Output
           //cout<<"**New Best Solution Path (through REWIRE)**"<<endl;
           //cout<<"Total Cost: "<<m_cost_best_solution_path<<endl;
           //cout<<"Revolute Cost: "<<m_cost_best_solution_path_revolute<<endl;
           //cout<<"Prismatic Cost: "<<m_cost_best_solution_path_prismatic<<endl;

           ROS_INFO_STREAM("New Best Solution Path (through REWIRE)");

           //Show updated solution path (updated by rewireing a node on the solution path)
           if (show_tree_vis == true)
           {
               //Perform search for best solution path and shows it as a line strip
               // -> "m_node_tree_B" (from tree_B) or "m_node_tree_A" (from tree_A) has an updated cost due to a parent that has been rewired
               showCurrentSolutionPath(m_connected_tree_name, m_node_tree_B, m_node_tree_A);
           }

       }

   }


    //Iterate through children of near vertex
    for(int edg = 0 ; edg < tree_vertex.outgoing_edges.size() ; edg++)
    {

        //Call recursiveNodeCostUpdate for child node of current node corresponding to near_vertex
        recursiveNodeCostUpdate(tree, tree->nodes[tree_vertex.outgoing_edges[edg].child_node_id], cost_reduction, show_tree_vis);
    }

}


// ------------------------------ Constraint Motion Planning -----------------------------------------------------

//setTaskFrameConstraints with task_frame expressed in base_link frame
// -> Note: if no task_frame (w.r.t base_link) is given the task_frame will be identical to the end-effector frame at the start config
void BiRRTstarPlanner::setTaskFrameConstraints(vector<int> cv, vector<pair<double, double> > coordinate_dev, bool task_pos_global, bool task_orient_global, boost::optional<KDL::Frame> task_frame)
{
    //Check dimension of constraint vector
    if(cv.size() > 6)
        ROS_ERROR("Dimension of constraint vector larger than task space dimension!!!");
    if(cv.size() < 6)
        ROS_ERROR("Dimension of constraint vector smaller than task space dimension!!!");

    //Flag indicating whether constraint motion planning is active
    m_constraint_active = true;

    //Flags indicating whether the constraint is global (i.e the task frame always corresponds to the start ee frame)
    //or local (i.e the task frame always corresponds to the current ee frame)
    m_task_pos_global = task_pos_global;
    m_task_orient_global = task_orient_global;

    //------------- Set Constraint Selection Vector and Permitted Deviation-------------

    // -> specifying which axes of the task frame permit valid displacement
    // -> specifies an lower and upper deviation boundary
    for(int i = 0 ; i < cv.size() ; i++)
    {
        m_constraint_vector[i] = cv[i];
        m_coordinate_dev[i].first = coordinate_dev[i].first;
        m_coordinate_dev[i].second = coordinate_dev[i].second;
    }

    //------------- Init Task Frame -------------

    //Set maximum number of near nodes considered in "choose_node_parent_iterpolation" and "rewireTreeInterpolation"
    m_max_near_nodes = 10;

    //If no task_frame is given as input -> task frame corresponds to end-effector frame in start config
    // using boost::optional for "task_frame" argument the value of "task_frame" is false if no value has been passed to the function for this element
    if(!task_frame)
    {
        //Convert start configuration into KDL Joint Array
        KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(m_config_start_pose);

        //Set Task frame to pose of endeffector frame at start config
        m_task_frame = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);

        //Task frame and start endeffector frame are identical -> thus grasp_transform is the identity
        m_grasp_transform = KDL::Frame::Identity();

        ROS_INFO_STREAM("Task Frame identical with EE Frame in start config ...");
    }
    else //a task_frame in the base_link frame has been specified
    {
        //Set Task frame to pose of endeffector frame at start config
        m_task_frame = *task_frame;

        //Convert start configuration into KDL Joint Array
        KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(m_config_start_pose);
        //Get end-effector pose in grasp / start configuration w.r.t global / world frame
        KDL::Frame start_ee_pose = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);

        //Get pose of task frame in end-effector frame
        m_grasp_transform = start_ee_pose.Inverse() * (*task_frame);

        ROS_INFO_STREAM("Task Frame set manually ...");
    }

}


////Function to set parameterized task frame
//void BiRRTstarPlanner::setParameterizedTaskFrame(vector<int> cv, vector<pair<double, double> > coordinate_dev, bool task_pos_global, bool task_orient_global)
//{
//    //Check dimension of constraint vector
//    if(cv.size() > 6)
//        ROS_ERROR("Dimension of constraint vector larger than task space dimension!!!");
//    if(cv.size() < 6)
//        ROS_ERROR("Dimension of constraint vector smaller than task space dimension!!!");

//    //Flag indicating whether constraint motion planning is active
//    m_constraint_active = true;

//    //Flags indicating whether the constraint is global (i.e the task frame always corresponds to the start ee frame)
//    //or local (i.e the task frame always corresponds to the current ee frame)
//    m_task_pos_global = task_pos_global;
//    m_task_orient_global = task_orient_global;

//    //------------- Set Constraint Selection Vector and Permitted Deviation-------------

//    // -> specifying which axes of the task frame permit valid displacement
//    // -> specifies an lower and upper deviation boundary
//    for(int i = 0 ; i < cv.size() ; i++)
//    {
//        m_constraint_vector[i] = cv[i];
//        m_coordinate_dev[i].first = coordinate_dev[i].first;
//        m_coordinate_dev[i].second = coordinate_dev[i].second;
//    }

//    //------------- Init Task Frame -------------

//    //Convert start configuration into KDL Joint Array
//    KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(m_config_start_pose);

//    //Set Task frame to pose of endeffector frame at start config
//    m_task_frame = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);

//    //Task frame and start endeffector frame are identical -> thus grasp_transform is the identity
//    m_grasp_transform = KDL::Frame::Identity();


//    //Set maximum number of near nodes considered in "choose_node_parent_iterpolation" and "rewireTreeInterpolation"
//    m_max_near_nodes = 10;

//}


////Function to set fixed task frame
//void BiRRTstarPlanner::setFixedTaskFrame(vector<int> cv, vector<pair<double,double> > coordinate_dev, KDL::Frame task_frame)
//{
//    //Check dimension of constraint vector
//    if(cv.size() > 6)
//        ROS_ERROR("Dimension of constraint vector larger than task space dimension!!!");
//    if(cv.size() < 6)
//        ROS_ERROR("Dimension of constraint vector smaller than task space dimension!!!");

//    //Flag indicating whether constraint motion planning is active
//    m_constraint_active = true;

//    //Set Flags to true indicating that a fixed task frame is set globally
//    m_task_pos_global = true;
//    m_task_orient_global = true;

//    //------------- Set Constraint Selection Vector and Permitted Deviation-------------

//    // -> specifying which axes of the task frame permit valid displacement
//    // -> specifies an lower and upper deviation boundary
//    for(int i = 0 ; i < cv.size() ; i++)
//    {
//        m_constraint_vector[i] = cv[i];
//        m_coordinate_dev[i].first = coordinate_dev[i].first;
//        m_coordinate_dev[i].second = coordinate_dev[i].second;
//    }

//    //------------- Init Task Frame -------------

//    //Set Task frame to pose of endeffector frame at start config
//    m_task_frame = task_frame;

//    //Convert start configuration into KDL Joint Array
//    KDL::JntArray start_configuration = m_RobotMotionController->Vector_to_JntArray(m_config_start_pose);
//    //Get end-effector pose in grasp / start configuration w.r.t global / world frame
//    KDL::Frame start_ee_pose = m_KDLRobotModel->compute_FK_frame(m_manipulator_chain, start_configuration);

//    //Get pose of task frame in end-effector frame
//    m_grasp_transform = start_ee_pose.Inverse() * task_frame;

//    //Set maximum number of near nodes considered in "choose_node_parent_iterpolation" and "rewireTreeInterpolation"
//    m_max_near_nodes = 10;
//}


//Function to set edge cost weights (to punish certain joint motions)
void BiRRTstarPlanner::setEdgeCostWeights(vector<double> ecw)
{
    if(ecw.size() < m_num_joints)
        ROS_ERROR("Dimension of edge cost weights vector is smaller than the number of Joints of the Kinematic Chain");

    //Set Edge Cost Weights
    for(int j = 0 ; j < m_num_joints ; j++)
    {
        m_edge_cost_weights[j] = ecw[j];
    }
}


////Function to set the configuration projection error threshold (used in constraint satisfaction function)
//void BiRRTstarPlanner::setConfigProjectionErrorThreshold(double threshold)
//{
//    m_projection_error_threshold = threshold;
//}


//Perform a step from nearest neighbour towards random sample
bool BiRRTstarPlanner::stepTowardsRandSample(Node nn_node, Node &x_rand, double extend_step_factor)
{
    //Flag to be returned
    bool rand_sample_reached = true;

    //Compute distance between configurations
    vector<double> eucl_dist(nn_node.config.size());

    //Sum Squares for revolute and prismatic joints
    double sum_squares_rev_joints = 0.0;
    double sum_squares_prism_joints = 0.0;


    //Current Joint Index
    int joint_idx = 0;
    for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
     {
         if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
         {
             //Variable Distance
             eucl_dist[joint_idx] = x_rand.config[joint_idx] - nn_node.config[joint_idx];

             //Distance Squared
             double dist_sqrt = eucl_dist[joint_idx] *  eucl_dist[joint_idx];

             //Collect sum of squares for prismatic and revolute joints
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "RotAxis")
                sum_squares_prism_joints += dist_sqrt ;
             else
                sum_squares_rev_joints += dist_sqrt;

             joint_idx++;
         }
    }


    //Length of vector for revolute and prismatic joints
    double vec_length_rot = sqrt(sum_squares_rev_joints);
    double vec_length_prism = sqrt(sum_squares_prism_joints);

    //Check whether revolute or prismatic joints have reached the target (in the previous iteration)
    bool rot_joint_target_reached = false;
    bool prism_joint_target_reached = false;
    if(vec_length_rot < 0.001)
            rot_joint_target_reached = true;
    if(vec_length_prism < 0.001)
            prism_joint_target_reached = true;


    //Normalize direction vector and compute new extend config
    vector<double> extend_config(nn_node.config.size());

    //For computing distance between nn_node config and extend config
    // (prismatic and revolute joint expansion treated seperately)
    sum_squares_rev_joints = 0.0;
    sum_squares_prism_joints = 0.0;

//    cout<<vec_length_rot<<endl;
//    cout<<vec_length_prism<<endl;

//    cout<<rot_joint_target_reached<<endl;
//    cout<<prism_joint_target_reached<<endl;




    //Current Joint Index
    joint_idx = 0;
    //Compute extended config for prismatic and revolute joints
    for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
     {
         if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
         {
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "RotAxis")
             {
                 if(prism_joint_target_reached == false)
                 {
                     //Normalize component
                     eucl_dist[joint_idx] = eucl_dist[joint_idx]/vec_length_prism;

                     //Perform step from nn_node config towards x_rand (for prismatic joints)
                     double c_step = extend_step_factor * eucl_dist[joint_idx];
                     //cout<<c_step<<endl;
                     extend_config[joint_idx] = nn_node.config[joint_idx] + c_step;

                     //Collect sum of squares
                     sum_squares_prism_joints += c_step * c_step;
                 }
             }
             else
             {
                 if(rot_joint_target_reached == false)
                 {
                     //Normalize component
                     eucl_dist[joint_idx] = eucl_dist[joint_idx]/vec_length_rot;

                     //Perform step from nn_node config towards x_rand (for revolute joints)
                     double c_step = extend_step_factor * eucl_dist[joint_idx];
                     //cout<<c_step<<endl;
                     extend_config[joint_idx] = nn_node.config[joint_idx] + c_step;

                     //Collect sum of squares
                     sum_squares_rev_joints += c_step * c_step;
                 }
             }

             joint_idx++;
         }
    }
    //cout<<endl;

    //Get length of extension for prismatic and revolute joints (set to very large value if config for rev. or prism. joints has already been reached)
    double extend_vec_length_prism = sum_squares_prism_joints == 0.0 ? 1000.0 : sqrt(sum_squares_prism_joints);
    double extend_vec_length_rot = sum_squares_rev_joints == 0.0 ? 1000.0 : sqrt(sum_squares_rev_joints);


    //------------------------------------

    //Check whether revolute joints have reached the end node config (otherwise revolute joint values of current x_rand node remain untouched)
    if(extend_vec_length_rot < vec_length_rot)
    {
        //Reset current Joint Index
        joint_idx = 0;
        //Set new config for revolute joints
        for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
         {
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
             {

                 if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() == "RotAxis")
                 {
                     x_rand.config[joint_idx] = extend_config[joint_idx];
                 }

                 //Increment joint index counter
                 joint_idx++;
             }
        }

        //Set flag indicating whether end node config has been reached to false
        rand_sample_reached = false;
    }

    //Check whether prismatic joints have reached the end node config (otherwise prismatic joint values of current x_rand node remain untouched)
    if(extend_vec_length_prism < vec_length_prism)
    {
        //Reset current Joint Index
        joint_idx = 0;
        //Set new config for rotational joints
        for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
         {
             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
             {
                 if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "RotAxis")
                 {
                     x_rand.config[joint_idx] = extend_config[joint_idx];
                 }

                 //Increment joint index counter
                 joint_idx++;
             }
        }

        //Set flag indicating whether end node config has been reached to false
        rand_sample_reached = false;
    }

    //------------------------------------


//    //Modify config of x_rand if initial config of x_rand has not been reached
//    if(extend_vec_length_rot < vec_length_rot && extend_vec_length_prism < vec_length_prism)
//    {
//        //Set new config for x_rand
//        x_rand.config = extend_config;

//        //Set new ee pose for x_rand
//        KDL::JntArray ext_configuration = m_RobotMotionController->Vector_to_JntArray(extend_config);
//        x_rand.ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,ext_configuration);

//    }
//    else
//    {
//        //Configuration of x_rand has been reached (no modification to x_rand data performeds)
//        rand_sample_reached = true;
//    }


    return rand_sample_reached;

}


// ------------------------------ Tree Visualization -----------------------------------------------------
void BiRRTstarPlanner::add_tree_edge_vis(string tree_name, Edge new_edge)
{


//    cout<<"Add edge: "<<endl;
//    cout<<new_edge.root_node_id<<endl;
//    cout<<new_edge.child_node_id<<endl;
//    cout<<new_edge.edge_id<<endl;


    geometry_msgs::Point ep;


    //Set marker data for current Line_strip/edge
    visualization_msgs::Marker add_edge_marker_;

    //Set edge marker ID
    //add_edge_marker_.id = tree->num_edges;
    add_edge_marker_.id = new_edge.edge_id;


    //Set up properties for add edge marker
    add_edge_marker_.header.stamp = ros::Time::now();
    add_edge_marker_.header.frame_id = m_base_link_name;
    add_edge_marker_.ns = "line_list";
    add_edge_marker_.action = visualization_msgs::Marker::ADD;
    add_edge_marker_.pose.orientation.w = 1.0;
    //Set Marker Type
    add_edge_marker_.type = visualization_msgs::Marker::LINE_LIST; //Not LINE_STRIP, otherwise all points in the MarkerArray will be connected
    //LINE_LIST markers use only the x component of scale, for the line/edge width
    add_edge_marker_.scale.x = m_edge_marker_scale;


//    //Generate Line_strip for new edge
//    for (int tp = 0 ; tp < edge.ee_trajectory.size() ; tp++)
//    {
//        //Set current endeffector position (at point "tp" of the edge)
//        ep.x = edge.ee_trajectory[tp][0]; //X
//        ep.y = edge.ee_trajectory[tp][1]; //Y
//        ep.z = edge.ee_trajectory[tp][2]; //Z

//        //Enter current ee position into marker vector
//        add_edge_marker_.points.push_back(ep);
//    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //NOTE : LINE_STRIP CAUSES RVIZ TO CRASH WHEN NUMBER OF POINTS IN MARKER ARRAY IS HIGHER THAN 8200 !!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //Set first and last endeffector position to represent edge
    ep.x = new_edge.ee_trajectory[0][0]; //X
    ep.y = new_edge.ee_trajectory[0][1]; //Y
    ep.z = new_edge.ee_trajectory[0][2]; //Z
    add_edge_marker_.points.push_back(ep);
    ep.x = new_edge.ee_trajectory[new_edge.ee_trajectory.size()-1][0]; //X
    ep.y = new_edge.ee_trajectory[new_edge.ee_trajectory.size()-1][1]; //Y
    ep.z = new_edge.ee_trajectory[new_edge.ee_trajectory.size()-1][2]; //Z
    add_edge_marker_.points.push_back(ep);


    //Insert edge to be added
    if (tree_name == "START")
    {
        //Set colour for tree edges
        add_edge_marker_.color.r = 0.0;
        add_edge_marker_.color.g = 0.0;
        add_edge_marker_.color.b = 1.0;
        add_edge_marker_.color.a = 1.0;

        m_start_tree_add_edge_marker_array_msg.markers.insert(m_start_tree_add_edge_marker_array_msg.markers.begin() + new_edge.edge_id, add_edge_marker_);
    }
    else if (tree_name == "GOAL")
    {
        //Set colour for tree edges
        add_edge_marker_.color.r = 1.0;
        add_edge_marker_.color.g = 0.0;
        add_edge_marker_.color.b = 0.0;
        add_edge_marker_.color.a = 1.0;

        m_goal_tree_add_edge_marker_array_msg.markers.insert(m_goal_tree_add_edge_marker_array_msg.markers.begin() + new_edge.edge_id, add_edge_marker_);
    }
    else
        ROS_ERROR("UNKOWN TREE NAME");

}



void BiRRTstarPlanner::remove_tree_edge_vis(string tree_name, Edge old_edge)
{
    //Remove edge from RRT* tree edge visualization
    if (tree_name == "START")
         m_start_tree_add_edge_marker_array_msg.markers.erase(m_start_tree_add_edge_marker_array_msg.markers.begin() + old_edge.edge_id);
    else if (tree_name == "GOAL")
         m_goal_tree_add_edge_marker_array_msg.markers.erase(m_goal_tree_add_edge_marker_array_msg.markers.begin() + old_edge.edge_id);
    else
        ROS_ERROR("UNKOWN TREE NAME");




    //if(old_edge.child_node_id == 0)
    //{
//        cout<<"Remove edge: "<<endl;
//        //cout<<old_edge.root_node_id<<endl;
//        //cout<<old_edge.child_node_id<<endl;
//        cout<<old_edge.edge_id<<endl;
    //}

    //Set marker data for current Line_strip/edge
//    visualization_msgs::Marker remove_edge_marker_;
//    remove_edge_marker_.header.stamp = ros::Time::now();
//    //Set up properties for remove edge marker
//    remove_edge_marker_.action = visualization_msgs::Marker::DELETE;
//    remove_edge_marker_.header.frame_id = m_base_link_name;
//    remove_edge_marker_.ns = "line_list";

//    //Set marker ID
//    //remove_edge_marker_.id = 0;
//    remove_edge_marker_.id = old_edge.edge_id;
//    //remove_edge_marker_.id = m_tree.num_edge_operations;

//    //Set colour for tree edges
//    remove_edge_marker_.color.r = 0.0;
//    remove_edge_marker_.color.g = 1.0;
//    remove_edge_marker_.color.b = 0.0;
//    remove_edge_marker_.color.a = 1.0;


//    //Set first and last endeffector position to represent edge to be removed
//    geometry_msgs::Point ep;
//    ep.x = old_edge.ee_trajectory[0][0]; //X
//    ep.y = old_edge.ee_trajectory[0][1]; //Y
//    ep.z = old_edge.ee_trajectory[0][2]; //Z
//    remove_edge_marker_.points.push_back(ep);
//    ep.x = old_edge.ee_trajectory[old_edge.ee_trajectory.size()-1][0]; //X
//    ep.y = old_edge.ee_trajectory[old_edge.ee_trajectory.size()-1][1]; //Y
//    ep.z = old_edge.ee_trajectory[old_edge.ee_trajectory.size()-1][2]; //Z
//    remove_edge_marker_.points.push_back(ep);

//    //Insert edge to be deleted
//    remove_edge_marker_array_msg_.markers.push_back(remove_edge_marker_);


//    cout<<"Remove edge"<<endl;
//    for(int m = 0 ; m < remove_edge_marker_.points.size() ; m++)
//    {
//        cout<<"X: "<<remove_edge_marker_.points[m].x<<endl;
//        cout<<"Y: "<<remove_edge_marker_.points[m].y<<endl;
//        cout<<"Z: "<<remove_edge_marker_.points[m].z<<endl;
//    }
//    cout<<endl;


}


void BiRRTstarPlanner::add_tree_node_vis(string tree_name, Node new_node)
{
    geometry_msgs::Point tn;
    tn.x = new_node.ee_pose[0];
    tn.y = new_node.ee_pose[1];
    tn.z = new_node.ee_pose[2];

    //Add node to marker
    bool tree_idx = tree_name == "START" ? true : false;
    if(tree_idx == true)
    {
//        //Set color for tree nodes
//        m_start_tree_add_nodes_marker.color.r = 0.0;
//        m_start_tree_add_nodes_marker.color.g = 0.0;
//        m_start_tree_add_nodes_marker.color.b = 1.0;

        m_start_tree_add_nodes_marker.points.push_back(tn);
    }
    else
    {
          m_goal_tree_add_nodes_marker.points.push_back(tn);
    }

}


void BiRRTstarPlanner::add_tree_node_vis(string tree_name, Node new_node, vector<double> color_rgb)
{
    if(color_rgb.size() != 3)
      ROS_ERROR("Erroneous colour vector for node!!!");

    geometry_msgs::Point tn;
    tn.x = new_node.ee_pose[0];
    tn.y = new_node.ee_pose[1];
    tn.z = new_node.ee_pose[2];

    //Set colour for tree nodes
    m_start_tree_add_nodes_marker.color.r = color_rgb[0];
    m_start_tree_add_nodes_marker.color.g = color_rgb[1];
    m_start_tree_add_nodes_marker.color.b = color_rgb[2];


    //Add node to marker
    bool tree_idx = tree_name == "START" ? true : false;
    if(tree_idx == true)
          m_start_tree_add_nodes_marker.points.push_back(tn);
    else
          m_goal_tree_add_nodes_marker.points.push_back(tn);

}


//Compute the current solution path (computes solution trajectory and line strip for visualization)
// "tree_name" tells us to which tree the node_tree_B belongs to (start or goal tree)
void BiRRTstarPlanner::showCurrentSolutionPath(string connected_tree_name, Node node_tree_B, Node node_tree_A)
{

    // Marker for endeffector solution path (LINE_STRIP)
    visualization_msgs::Marker ee_solution_path_marker;
    //Set up properties for solution path marker(LINE_STRIP)
    ee_solution_path_marker.header.frame_id = m_base_link_name;
    ee_solution_path_marker.ns = "ee_solution";
    ee_solution_path_marker.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    ee_solution_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    //LINE_STRIP markers use only the x component of scale, for the line/edge width
    ee_solution_path_marker.scale.x = m_solution_path_ee_marker_scale;
    ee_solution_path_marker.pose.orientation.w = 1.0;
    //Set colour for solution path line
    ee_solution_path_marker.color.r = 0.25;
    ee_solution_path_marker.color.g = 0.78;
    ee_solution_path_marker.color.b = 0.78;
    ee_solution_path_marker.color.a = 1.0;

    // Marker for base solution path (LINE_STRIP)
    visualization_msgs::Marker base_solution_path_marker;
    //Set up properties for solution path marker(LINE_STRIP)
    base_solution_path_marker.header.frame_id = m_base_link_name;
    base_solution_path_marker.ns = "base_solution";
    base_solution_path_marker.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    base_solution_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    //LINE_STRIP markers use only the x component of scale, for the line/edge width
    base_solution_path_marker.scale.x = m_solution_path_base_marker_scale;
    base_solution_path_marker.pose.orientation.w = 1.0;
    //Set colour for solution path line
    base_solution_path_marker.color.r = 1.0;
    base_solution_path_marker.color.g = 0.67;
    base_solution_path_marker.color.b = 0.0;
    base_solution_path_marker.color.a = 1.0;


    //Get node belonging to the start and goal tree
    // -> required because it is not known to which tree (start and goal tree) corresponds to tree_A and tree_B (due to swapping operation)
    Node start_tree_node;
    Node goal_tree_node;
    bool tree_idx = connected_tree_name == "START" ? true : false;
    if(tree_idx == true)
    {
        start_tree_node = node_tree_B;
        goal_tree_node = node_tree_A;

    }
    else
    {
        start_tree_node = node_tree_A;
        goal_tree_node = node_tree_B;
    }


    //cout<<"Solution path search (start tree): "<<start_tree_node.node_id<<endl;

    //Go backwards from connected start tree node along the tree until start node has been reached (start node  = m_start_tree.nodes[0].id = 0)
    while(start_tree_node.node_id != 0)
    {
        //Get Number of outgoing edges from parent node
        int num_out_edges = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges.size();

        //Search for edge leading to current node
        for (int ed = 0 ; ed < num_out_edges ; ed++)
        {
            if(m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].child_node_id == start_tree_node.node_id)
            {

                //A point on the line strip
                geometry_msgs::Point ep;

                //Get index of last element of outgoing edge EE trajectory
                int ee_traj_last_element_idx = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory.size() - 1 ;

                //End point of edge
                ep.x = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[ee_traj_last_element_idx][0];
                ep.y = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[ee_traj_last_element_idx][1];
                ep.z = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[ee_traj_last_element_idx][2];
                ee_solution_path_marker.points.push_back(ep);

                //Start point of edge
                ep.x = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[0][0];
                ep.y = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[0][1];
                ep.z = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[0][2];
                ee_solution_path_marker.points.push_back(ep);


                //Get config of base joints for current edge
                if(m_num_joints_prismatic > 0)
                {
                    //Get index of last element of outgoing edge EE trajectory
                    int joint_traj_last_element_idx = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].joint_trajectory.size() - 1 ;

                    //End point of edge
                    ep.x = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[joint_traj_last_element_idx][0];
                    ep.y = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[joint_traj_last_element_idx][1];
                    ep.z = 0.0;
                    base_solution_path_marker.points.push_back(ep);

                    //Start point of edge
                    ep.x = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[0][0];
                    ep.y = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[0][1];
                    ep.z = 0.0;
                    base_solution_path_marker.points.push_back(ep);
                }


                //Leave loop when edge of parent node leading to the current start_tree_node has been found
                break;

            }
        }


        //Set current node to parent of current node (going one step backwards along the tree)
        start_tree_node = m_start_tree.nodes[start_tree_node.parent_id];

         //cout<<"Solution path search (start tree): "<<start_tree_node.node_id<<endl;
    }



    //cout<<"Solution path search (goal tree): "<<goal_tree_node.node_id<<endl;

    //Go backwards from connected goal tree node along the tree until goal node has been reached (start node  = m_goal_tree.nodes[0].id = 0)
    while(goal_tree_node.node_id != 0)
    {
        //Get Number of outgoing edges from parent node
        int num_out_edges = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges.size();

        //Search for edge leading to current node
        for (int ed = 0 ; ed < num_out_edges ; ed++)
        {
            if(m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].child_node_id == goal_tree_node.node_id)
            {

                //A point on the line strip
                geometry_msgs::Point ep;

                //Get index of last element of outgoing edge EE trajectory
                int ee_traj_last_element_idx = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory.size() - 1 ;

                //End point of edge
                ep.x = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[ee_traj_last_element_idx][0];
                ep.y = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[ee_traj_last_element_idx][1];
                ep.z = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[ee_traj_last_element_idx][2];
                ee_solution_path_marker.points.insert(ee_solution_path_marker.points.begin(), ep);

                //Start point of edge
                ep.x = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[0][0];
                ep.y = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[0][1];
                ep.z = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].ee_trajectory[0][2];
                ee_solution_path_marker.points.insert(ee_solution_path_marker.points.begin(), ep);


                //Get config of base joints for current edge
                if(m_num_joints_prismatic > 0)
                {
                    //Get index of last element of outgoing edge EE trajectory
                    int joint_traj_last_element_idx = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].joint_trajectory.size() - 1 ;

                    //End point of edge
                    ep.x = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[joint_traj_last_element_idx][0];
                    ep.y = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[joint_traj_last_element_idx][1];
                    ep.z = 0.0;
                    base_solution_path_marker.points.insert(base_solution_path_marker.points.begin(), ep);

                    //Start point of edge
                    ep.x = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[0][0];
                    ep.y = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].joint_trajectory[0][1];
                    ep.z = 0.0;
                    base_solution_path_marker.points.insert(base_solution_path_marker.points.begin(), ep);
                }

                //Leave loop when edge of parent node leading to the current goal_tree_node has been found
                break;

            }
        }


        //Set current node to parent of current node (going one step backwards along the tree)
        goal_tree_node = m_goal_tree.nodes[goal_tree_node.parent_id];

        //cout<<"Solution path search (goal tree): "<<goal_tree_node.node_id<<endl;
    }

    //cout<<endl;

    //Publish endeffector solution path
    m_ee_solution_path_pub.publish(ee_solution_path_marker); //Show line strip representing solution path

    //Publish base solution path
    m_base_solution_path_pub.publish(base_solution_path_marker); //Show line strip representing solution path

}


//Compute final solution path trajectoryS
void BiRRTstarPlanner::computeFinalSolutionPathTrajectories()
{

    //Get node belonging to the start and goal tree
    // -> required because it is not known to which tree (start and goal tree) corresponds to tree_A and tree_B (due to swapping operation)
    Node start_tree_node;
    Node goal_tree_node;
    bool tree_idx = m_connected_tree_name == "START" ? true : false;
    if(tree_idx == true)
    {
        start_tree_node = m_node_tree_B;
        goal_tree_node = m_node_tree_A;

    }
    else
    {
        start_tree_node = m_node_tree_A;
        goal_tree_node = m_node_tree_B;
    }


    //Solution path edges of goal and star tre
    vector<Edge> solution_path_edges_start_tree;
    vector<Edge> solution_path_edges_goal_tree;;


    //Go backwards from connected start tree node along the tree until start node has been reached (start node  = m_start_tree.nodes[0].id = 0)
    while(start_tree_node.node_id != 0)
    {
        //Get Number of outgoing edges from parent node
        int num_out_edges = m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges.size();

        //Search for edge leading to current node
        for (int ed = 0 ; ed < num_out_edges ; ed++)
        {
            if(m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed].child_node_id == start_tree_node.node_id)
            {
                //Add edge to solution path
                solution_path_edges_start_tree.insert(solution_path_edges_start_tree.begin(), m_start_tree.nodes[start_tree_node.parent_id].outgoing_edges[ed]);

                //Leave loop when edge of parent node leading to the current start_tree_node has been found
                break;
            }
        }

        //Set current node to parent of current node (going one step backwards along the tree)
        start_tree_node = m_start_tree.nodes[start_tree_node.parent_id];

         //cout<<"Solution path search: "<<start_tree_node.node_id<<endl;
    }




    //Go backwards from connected goal tree node along the tree until goal node has been reached (start node  = m_goal_tree.nodes[0].id = 0)
    while(goal_tree_node.node_id != 0)
    {
        //Get Number of outgoing edges from parent node
        int num_out_edges = m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges.size();

        //Search for edge leading to current node
        for (int ed = 0 ; ed < num_out_edges ; ed++)
        {
            if(m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed].child_node_id == goal_tree_node.node_id)
            {

                //Add edge to solution path
                solution_path_edges_goal_tree.push_back(m_goal_tree.nodes[goal_tree_node.parent_id].outgoing_edges[ed]);

                //Leave loop when edge of parent node leading to the current goal_tree_node has been found
                break;
            }
        }


        //Set current node to parent of current node (going one step backwards along the tree)
        goal_tree_node = m_goal_tree.nodes[goal_tree_node.parent_id];

        //cout<<"Solution path search: "<<goal_tree_node.node_id<<endl;
    }



    //Build solution trajectories
    //Collect start tree edges constribution to solution path
    for (int spe = 0 ; spe < solution_path_edges_start_tree.size() ; spe++)
    {
        for(int element = 0 ; element < solution_path_edges_start_tree[spe].joint_trajectory.size()-1 ; element++)
        {
            //Note: Edge joint and ee trajectories have same number of elements
            m_result_joint_trajectory.push_back(solution_path_edges_start_tree[spe].joint_trajectory[element]);
            m_result_ee_trajectory.push_back(solution_path_edges_start_tree[spe].ee_trajectory[element]);

        }
    }
    //Collect goal tree edges constribution to solution path
    for (int spe = 0 ; spe < solution_path_edges_goal_tree.size() ; spe++)
    {
        //We need to invert the edges of the goal tree to get a forward motion from the start to the goal node
        for(int element = solution_path_edges_goal_tree[spe].joint_trajectory.size()-1 ; element > 0  ; element--)
        {
            //Note: Edge joint and ee trajectories have same number of elements
            m_result_joint_trajectory.push_back(solution_path_edges_goal_tree[spe].joint_trajectory[element]);
            m_result_ee_trajectory.push_back(solution_path_edges_goal_tree[spe].ee_trajectory[element]);
        }

        //Enter last point of last goal tree edge (i.e. the final point)
        if(spe == solution_path_edges_goal_tree.size()-1)
        {
            m_result_joint_trajectory.push_back(solution_path_edges_goal_tree[spe].joint_trajectory[0]);
            m_result_ee_trajectory.push_back(solution_path_edges_goal_tree[spe].ee_trajectory[0]);
        }
    }


    // --------------------------------- START: Smoothing --------------------------
    //Perform a smoothing operation given the jerky joint trajectory solution as input
    //solution_path_smoothing(m_result_joint_trajectory);
    // --------------------------------- END: Smoothing --------------------------

    //Write Planning results to file (joint and ee trajectory)
    m_KDLRobotModel->writeTrajectoryToFile(m_result_joint_trajectory,m_file_path_joint_trajectory);
    m_KDLRobotModel->writeTrajectoryToFile(m_result_ee_trajectory,m_file_path_ee_trajectory);

}



//Write Planner Statistics to File
void BiRRTstarPlanner::writePlannerStatistics(char *statistics_file, char *cost_evolution_file)
{
    //+++++++++ Write planner statistics to file ++++++++

    //Remove the planner statistics file
    if( remove( statistics_file ) != 0 )
    {
        cout<< "Error deleting file (planner_statistics)" <<endl;
    }

    //File to store planner statistics
    ofstream planner_stat_file;
    //Open file
    planner_stat_file.open(statistics_file);

    // file couldn't be opened
    if( !planner_stat_file)
    {
        cerr << "Error: planner_statistics file could not be opened" << endl;
        exit(1);
    }

    //------------------ Write data
    planner_stat_file << "planner_name" <<" "<< m_planner_type << endl \
                 << "planning_group" <<" "<< m_planning_group << endl\
                 << "planning_result" <<" "<< m_planner_success << endl;


    //Planner run with maximum planning iterations or time
    if(m_max_planner_iter != 0)
    {
        planner_stat_file << "max_iter" <<" "<< m_max_planner_iter << endl\
                 << "executed_iter" <<" "<< m_executed_planner_iter << endl;
    }
    else
    {
        planner_stat_file << "max_time" <<" "<< m_max_planner_time << endl\
                 << "executed_time" <<" "<< m_executed_planner_time << endl;
    }


    planner_stat_file << "first_solution_iter" <<" "<< m_first_solution_iter << endl\
                 << "first_solution_time" <<" "<< m_time_first_solution << endl\
                 << "last_solution_iter" <<" "<< m_last_solution_iter << endl\
                 << "last_solution_time" <<" "<< m_time_last_solution << endl;


    //Planner run with maximum planning iterations or time
    if(m_max_planner_iter != 0)
    {
        planner_stat_file << "total_plannig_time" <<" "<< m_time_planning_end << endl;
    }
    else
    {
        planner_stat_file << "total_plannig_iter" <<" "<< m_executed_planner_iter << endl;
    }


    planner_stat_file << "cost_theoretical_solution_total" <<" "<< m_cost_theoretical_solution_path[0] << endl\
                 << "cost_theoretical_solution_revolute" <<" "<< m_cost_theoretical_solution_path[1] << endl\
                 << "cost_theoretical_solution_prismatics" <<" "<< m_cost_theoretical_solution_path[2] << endl\
                 << "cost_final_solution_total" <<" "<< m_cost_best_solution_path << endl\
                 << "cost_final_solution_revolute" <<" "<< m_cost_best_solution_path_revolute << endl\
                 << "cost_final_solution_prismatic" <<" "<< m_cost_best_solution_path_prismatic << endl\
                 << "num_nodes_start_tree" <<" "<< m_start_tree.num_nodes << endl\
                 << "num_nodes_goal_tree" <<" "<< m_goal_tree.num_nodes << endl\
                 << "num_nodes_total" <<" "<< (m_start_tree.num_nodes + m_goal_tree.num_nodes) << endl\
                 << "num_edges_start_tree" <<" "<< m_start_tree.num_edges << endl\
                 << "num_edges_goal_tree" <<" "<< m_goal_tree.num_edges << endl\
                 << "num_edges_total" <<" "<< (m_start_tree.num_edges + m_goal_tree.num_edges) << endl\
                 << "num_rewire_start_tree" <<" "<< m_start_tree.num_rewire_operations << endl\
                 << "num_rewire_goal_tree" <<" "<< m_goal_tree.num_rewire_operations << endl\
                 << "num_rewire_total" <<" "<< (m_start_tree.num_rewire_operations + m_goal_tree.num_rewire_operations) << endl;


    //Close the file
    planner_stat_file.close();


    //++++++++ Write planner cost evolution to file +++++++++

    //Remove the planner cost evolution file
    if( remove( cost_evolution_file ) != 0 )
    {
        cout<< "Error deleting file (planner_cost_evolution)" <<endl;
    }

    //File to store planner statistics
    ofstream planner_cost_file;
    //Open file
    planner_cost_file.open(cost_evolution_file);

    // file couldn't be opened
    if( !planner_cost_file)
    {
        cerr << "Error: planner_cost_evolution file could not be opened" << endl;
        exit(1);
    }


    //Write header
    planner_cost_file << "planner_name" <<" "<< "iteration" <<" "<< "planning_time_elapsed" << " " << "cost_solution_total" <<" "<< "cost_solution_revolute" <<" "<< "cost_solution_prismatic" << " " << "theoretical_cost_total" << " " << "theoretical_cost_revolute" << " " << "theoretical_cost_prismatic"<<endl;

    for (int i = 0; i < m_solution_cost_trajectory.size() ; i++)
    {
        planner_cost_file << m_planner_type << " ";
        for (int j = 0; j < m_solution_cost_trajectory[0].size() ; j++)
        {
            //Write evolution of solution cost (total, revolute, prismatic)
            planner_cost_file << m_solution_cost_trajectory[i][j]<< " ";
        }
        //Write theoretical solution cost (total, revolute, prismatic)
        planner_cost_file << m_cost_theoretical_solution_path[0] << " " << m_cost_theoretical_solution_path[1] << " " << m_cost_theoretical_solution_path[2]<<endl;
    }

    //Close the file
    planner_cost_file.close();

}


//Write Start and Goal Config
void BiRRTstarPlanner::writeStartGoalConfig(char *start_goal_config_file, vector<double> start_config, vector<double> goal_config)
{
    //+++++++++ Write start and goal config to file ++++++++

    //Remove the planner statistics file
    if( remove( start_goal_config_file ) != 0 )
    {
        cout<< "Error deleting file (start_goal_config_file)" <<endl;
    }

    //File to store planner statistics
    ofstream terminal_configs_file;
    //Open file
    terminal_configs_file.open(start_goal_config_file);

    // file couldn't be opened
    if( !terminal_configs_file)
    {
        cerr << "Error: start_goal_config_file file could not be opened" << endl;
        exit(1);
    }

    //Write Start Config
    for (int i = 0; i < start_config.size() ; i++)
            terminal_configs_file<<start_config[i]<<" ";
    terminal_configs_file<<endl;
    //Write Goal Config
    for (int i = 0; i < goal_config.size() ; i++)
            terminal_configs_file<<goal_config[i]<<" ";
    terminal_configs_file<<endl;

    //Close the file
    terminal_configs_file.close();

}

//Read Start and Goal Config
bool BiRRTstarPlanner::readStartGoalConfig(char *start_goal_config_file, vector<double> &start_config, vector<double> &goal_config)
{
    //Flag indicating whether configurations have been read to file successfully
    bool read_ok = true;

    // --------------- Get the number terminal configuration coordinates --------------------------
    string line;
    int num_vals_per_line = 0;

    //Input stream
    ifstream terminal_config_file(start_goal_config_file);

    //Check if file can be opened
    if (terminal_config_file.is_open())
    {
        //Get first line
         std::getline (terminal_config_file,line);

         //Get the number of values per line
         istringstream buf(line);
         istream_iterator<string> beg(buf), end;
         vector<string> substrings(beg, end); // done!

         //Get number of values per line
         num_vals_per_line = substrings.size();

    }
    else
    {
        ROS_ERROR("Unable to open file (readStartGoalConfig)");
        read_ok = false;
    }

    //Close file
    terminal_config_file.close();


    //--------------- Set size for Start and Goal Configuration Vector ------------------
    start_config.resize(num_vals_per_line);
    goal_config.resize(num_vals_per_line);


    //--------------- Fill Start and Goal Configuration Vector --------------------------
    char * pEnd;
    int j = 0;
    char char_line[1000];

    //Open file
    terminal_config_file.open(start_goal_config_file);

    //Check if file is open
    if (terminal_config_file.is_open())
    {
         //--- read start config line from file
         std::getline (terminal_config_file,line);

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;

         for (int i = 0 ; i < num_vals_per_line ; ++i)
         {
           start_config[i] = strtod(tmp,&pEnd);
           //cout<<strtod(tmp,&pEnd)<<" ";
           tmp = pEnd;
         }


         //--- read goal config line from file
         std::getline (terminal_config_file,line);

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp2 = char_line;

         for (int i = 0 ; i < num_vals_per_line ; ++i)
         {
           goal_config[i] = strtod(tmp2,&pEnd);
           //cout<<strtod(tmp,&pEnd)<<" ";
           tmp2 = pEnd;
         }
        //Close file
        terminal_config_file.close();
    }
    else
    {
        ROS_ERROR("Unable to open file (readStartGoalConfig)");
        read_ok = false;
    }

    return read_ok;

}


//Attach Object to the End-effector
void BiRRTstarPlanner::attachObject(moveit_msgs::AttachedCollisionObject attached_object)
{
    //Store attached object (not used yet, but is probably required for attached object operations while planning)
    m_attached_object = attached_object;

    //Attach the given object "attached_object" to the end-effector in the start pose m_start_tree.nodes[0].ee_pose
    m_planning_world->attachObjecttoEndeffector(attached_object,m_ee_start_pose);

}

//DeAttach Object to the End-effector
void BiRRTstarPlanner::detachObject(moveit_msgs::AttachedCollisionObject attached_object)
{
    //Dettach the given object "attached_object" from the end-effector in the goal pose m_goal_tree.nodes[0].ee_pose
    m_planning_world->detachObjectfromEndeffector(attached_object,m_ee_goal_pose);
}


////Method for Solution Path Smoothing (from Paper "Fast Smoothing of Manipulator Trajectories using Optimal Bounded-Acceleration Shortcuts")
//void BiRRTstarPlanner::basic_solution_path_smoothing(vector<vector<double> > raw_joint_trajectory)
//{

//    //velocity and acceleration bounds, respectively
//    ParabolicRamp::Vector vmax(m_num_joints),amax(m_num_joints);
//    int joint_idx = 0;
//    for (int k = 0 ; k < m_manipulator_chain.getNrOfSegments(); k++)
//     {
//         if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "None")
//         {
//             //Bounds for prismatic joints
//             if(m_manipulator_chain.getSegment(k).getJoint().getTypeName() != "RotAxis")
//             {
//                vmax[joint_idx] = 1.0; // m/s
//                amax[joint_idx] = 0.5; // m/s^²
//                //cout<<vmax[joint_idx]<<" "<<amax[joint_idx]<<endl;
//             }
//             else //Bounds for revolute joints
//             {
//                vmax[joint_idx] = 0.175; // rad/s
//                amax[joint_idx] = 0.1; // rad/s^²
//                //cout<<vmax[joint_idx]<<" "<<amax[joint_idx]<<endl;
//             }

//             //Next joint
//             joint_idx++;
//         }
//    }

//    //Provides the min, max and center value for all joint contained in a given kinematic chain
//    vector<double> q_min(m_num_joints);
//    vector<double> q_max(m_num_joints);
//    vector<double> q_opt_pos(m_num_joints);
//    m_KDLRobotModel->getJointRangeData(m_manipulator_chain , q_min, q_max, q_opt_pos);


//    //a configuration/segment feasibility checker, subclassed from FeasibilityCheckerBase
//    state_feasibility_checker::FeasibilityChecker feas(m_planning_group);

//    //some number of shortcutting iterations
//    int numIters=20; //before 1000
//    //checks a piecewise linear path whose distance from the parabolic path is no more than the tolerance tol
//    ParabolicRamp::Real tol=1e-4;

//    //Checker
//    ParabolicRamp::RampFeasibilityChecker checker(&feas,tol);
//    //TODO: compute milestones, velocity and acceleration bounds

//    ParabolicRamp::DynamicPath traj;
//    traj.Init(vmax,amax);
//    traj.SetJointLimits(q_min,q_max);
//    //The sequence of milestones, i.e. the configurations of the solution path
//    vector<ParabolicRamp::Vector> path = raw_joint_trajectory;
//    //now the trajectory starts and stops at every milestone
//    traj.SetMilestones(path);

//    ParabolicRamp::DynamicPath traj_intermediate;
//    traj_intermediate.Init(vmax,amax);
//    traj_intermediate.SetJointLimits(q_min,q_max);

//    printf("Initial path duration: %g\n",traj.GetTotalTime());
//    printf("Initial number of path waypoints: %d\n",raw_joint_trajectory.size());
//    int res=traj.Shortcut(numIters,checker, traj_intermediate);
//    printf("After shortcutting: %d shortcuts taken, duration %g\n",res,traj.GetTotalTime());
//    vector<ParabolicRamp::Vector> pos;
//    vector<ParabolicRamp::Vector> vel;
//    traj_intermediate.GetMilestones(pos,vel);
//    printf("After shortcutting: Trajectory composed of %d waypoints\n",pos.size());

//    for(int wp = 0 ; wp < pos.size() ; wp++)
//        cout<<pos[wp][0]<<" "<<pos[wp][0]<<endl;


//}


// -------------------------------- TF Transforms when planning is performed on real robot (i.e. planning_frame = /map) ------------------------------------

//Transform sample from base_link frame to map frame
void BiRRTstarPlanner::transform_sample_to_map_frame(KDL::JntArray& sample_conf)
{
    //Transform base sample_conf to /map frame only when localization is active (acml package)
    //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
    if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
    {
        //cout<<"Name of Planning frame: "<<m_planning_frame<<endl;

        if(m_transform_map_to_base_available)
        {
            //Transform base_link to sample
            tf::StampedTransform transform_base_to_sample;
            transform_base_to_sample.setOrigin(tf::Vector3(sample_conf(0),sample_conf(1), 0.0));
            transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(sample_conf(2)));


            //Transform map frame to sample
            tf::StampedTransform transform_map_to_sample;
            transform_map_to_sample.mult(m_transform_map_to_base,transform_base_to_sample);

            //Sample in map frame (as vector)
            vector<double> map_to_sample_conf(3);
            tf::Vector3 map_to_sample_trans = transform_map_to_sample.getOrigin();
            tf::Quaternion map_to_sample_rot = transform_map_to_sample.getRotation();
            map_to_sample_conf[0] = map_to_sample_trans.x();
            map_to_sample_conf[1] = map_to_sample_trans.y();
            double z_dir = transform_map_to_sample.getRotation().getAxis().z();
            map_to_sample_conf[2] = z_dir > 0.0 ? map_to_sample_rot.getAngle() : -map_to_sample_rot.getAngle();

            //Express base config w.r.t map frame
            sample_conf(0) = map_to_sample_conf[0];
            sample_conf(1) = map_to_sample_conf[1];
            sample_conf(2) = map_to_sample_conf[2];
        }
    }

}

//Transform sample from base_link frame to map frame
void BiRRTstarPlanner::transform_sample_to_map_frame(vector<double>& sample_conf)
{
    //Transform base config to /map frame only when localization is active (acml package)
    //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
    if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
    {
        //cout<<"Name of Planning frame: "<<m_planning_frame<<endl;

        if(m_transform_map_to_base_available)
        {
            //Transform base_link to sample
            tf::StampedTransform transform_base_to_sample;
            transform_base_to_sample.setOrigin(tf::Vector3(sample_conf[0],sample_conf[1], 0.0));
            transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(sample_conf[2]));


            //Transform map frame to sample
            tf::StampedTransform transform_map_to_sample;
            transform_map_to_sample.mult(m_transform_map_to_base,transform_base_to_sample);

            //Sample in map frame (as vector)
            vector<double> map_to_sample_conf(3);
            tf::Vector3 map_to_sample_trans = transform_map_to_sample.getOrigin();
            tf::Quaternion map_to_sample_rot = transform_map_to_sample.getRotation();
            map_to_sample_conf[0] = map_to_sample_trans.x();
            map_to_sample_conf[1] = map_to_sample_trans.y();
            double z_dir = transform_map_to_sample.getRotation().getAxis().z();
            map_to_sample_conf[2] = z_dir > 0.0 ? map_to_sample_rot.getAngle() : -map_to_sample_rot.getAngle();

            //Express base config w.r.t map frame
            sample_conf[0] = map_to_sample_conf[0];
            sample_conf[1] = map_to_sample_conf[1];
            sample_conf[2] = map_to_sample_conf[2];
        }
    }

}


//Transform sample from map frame to base_link frame
void BiRRTstarPlanner::transform_sample_to_base_link_frame(KDL::JntArray& sample_conf)
{
    //Transform base config to /map frame only when localization is active (acml package)
    //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
    if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2 means robot base is involved in planning
    {
        //cout<<"Name of Planning frame: "<<m_planning_frame<<endl;

        //Transform map frame to sample
        tf::StampedTransform transform_map_to_sample;
        transform_map_to_sample.setOrigin(tf::Vector3(sample_conf(0),sample_conf(1), 0.0));
        transform_map_to_sample.setRotation(tf::createQuaternionFromYaw(sample_conf(2)));


        if(m_transform_map_to_base_available)
        {
            //Transform base_link frame to sample
            tf::StampedTransform transform_base_link_to_sample;
            transform_base_link_to_sample.mult(m_transform_map_to_base.inverse(),transform_map_to_sample);

            //Sample in base_link frame (as vector)
            vector<double> base_link_to_sample_conf(3);
            tf::Vector3 base_link_to_sample_trans = transform_base_link_to_sample.getOrigin();
            tf::Quaternion base_link_to_sample_rot = transform_base_link_to_sample.getRotation();
            //Set translation and rotation
            base_link_to_sample_conf[0] = base_link_to_sample_trans.x();
            base_link_to_sample_conf[1] = base_link_to_sample_trans.y();
            double z_dir = transform_base_link_to_sample.getRotation().getAxis().z();
            base_link_to_sample_conf[2] = z_dir > 0.0 ? base_link_to_sample_rot.getAngle() : -base_link_to_sample_rot.getAngle();

            //Convert goal_pose for base from vector to Eigen::Affine3d
            //Eigen::Affine3d sample_pose_base;
            //Eigen::Affine3d trans_base(Eigen::Translation3d(Eigen::Vector3d(base_link_to_sample_conf[0],base_link_to_sample_conf[1],0)));
            //Eigen::Affine3d rot_base = Eigen::Affine3d(Eigen::AngleAxisd(base_link_to_sample_conf[2], Eigen::Vector3d(0, 0, 1)));
            //sample_pose_base = trans_base * rot_base;

            //Express base config w.r.t map frame
            sample_conf(0) = base_link_to_sample_conf[0];
            sample_conf(1) = base_link_to_sample_conf[1];
            sample_conf(2) = base_link_to_sample_conf[2];

        }
    }
}

//Transform sample from map frame to base_link frame
void BiRRTstarPlanner::transform_sample_to_base_link_frame(vector<double>& sample_conf)
{
    //Transform base config to /map frame only when localization is active (acml package)
    //if(m_planning_frame == "/map" && (m_planning_group == "omnirob_base" || m_planning_group == "omnirob_lbr_sdh"))
    if(m_planning_frame == "/map" && m_num_joints_prismatic == 2 && m_num_joints_revolute != 0) //-> m_num_joints_prismatic == 2  and && m_num_joints_revolute > 1 means robot base is involved in planning
    {
        //cout<<"Name of Planning frame: "<<m_planning_frame<<endl;

        if(m_transform_map_to_base_available)
        {
            //Transform map frame to sample
            tf::StampedTransform transform_map_to_sample;
            transform_map_to_sample.setOrigin(tf::Vector3(sample_conf[0],sample_conf[1], 0.0));
            transform_map_to_sample.setRotation(tf::createQuaternionFromYaw(sample_conf[2]));


            //Transform base_link frame to sample
            tf::StampedTransform transform_base_link_to_sample;
            transform_base_link_to_sample.mult(m_transform_map_to_base.inverse(),transform_map_to_sample);

            //Sample in base_link frame (as vector)
            vector<double> base_link_to_sample_conf(3);
            tf::Vector3 base_link_to_sample_trans = transform_base_link_to_sample.getOrigin();
            tf::Quaternion base_link_to_sample_rot = transform_base_link_to_sample.getRotation();
            base_link_to_sample_conf[0] = base_link_to_sample_trans.x();
            base_link_to_sample_conf[1] = base_link_to_sample_trans.y();
            double z_dir = transform_base_link_to_sample.getRotation().getAxis().z();
            base_link_to_sample_conf[2] = z_dir > 0.0 ? base_link_to_sample_rot.getAngle() : -base_link_to_sample_rot.getAngle();

            //Convert goal_pose for base from vector to Eigen::Affine3d
            //Eigen::Affine3d sample_pose_base;
            //Eigen::Affine3d trans_base(Eigen::Translation3d(Eigen::Vector3d(base_link_to_sample_conf[0],base_link_to_sample_conf[1],0)));
            //Eigen::Affine3d rot_base = Eigen::Affine3d(Eigen::AngleAxisd(base_link_to_sample_conf[2], Eigen::Vector3d(0, 0, 1)));
            //sample_pose_base = trans_base * rot_base;

            //Express base config w.r.t map frame
            sample_conf[0] = base_link_to_sample_conf[0];
            sample_conf[1] = base_link_to_sample_conf[1];
            sample_conf[2] = base_link_to_sample_conf[2];

        }
    }
}


// -------------------------------- Getters ------------------------------------

//Get the planned joint trajectory
vector<vector<double> > BiRRTstarPlanner::getJointTrajectory()
{
    return m_result_joint_trajectory;

}
//Get the planned endeffector trajectory
vector<vector<double> > BiRRTstarPlanner::getEndeffectorTrajectory()
{
    return m_result_ee_trajectory;
}

//Get the number of joints the planning group is composed of
int BiRRTstarPlanner::getNumJointsPlanningGroup()
{
    return m_num_joints;
}

int BiRRTstarPlanner::getNumPrismaticJointsPlanningGroup()
{
    return m_num_joints_prismatic;
}

int BiRRTstarPlanner::getNumRevoluteJointsPlanningGroup()
{
    return m_num_joints_revolute;
}


// -------------------------------- Functions for consistency checks ------------------------------------
void BiRRTstarPlanner::no_two_parents_check(Rrt_star_tree *tree)
{

    //
    for (int i = 0 ; i < tree->nodes.size() ; i++)
    {
        //int parent_id = tree->nodes[i].parent_id;
        //cout<<"Incoming edges for node: "<<tree->nodes[i].node_id<<endl;
        int num_incoming_edges = 0;

        for (int j = 0 ; j < tree->nodes.size() ; j++)
        {
            for(int e = 0 ; e < tree->nodes[j].outgoing_edges.size() ; e++)
            {
                //if(tree->nodes[j].node_id == 1)
                 //   cout<<"Start Node has an outgoing edge"<<endl;

                if(tree->nodes[j].outgoing_edges[e].child_node_id == tree->nodes[i].node_id)
                {
                    //if(tree->nodes[j].cost_reach.total > tree->nodes[i].cost_reach.total)
                    //   cout<<"Outgoing edges from node: "<<tree->nodes[j].cost_reach.total<<" to nodes: "<< tree->nodes[i].cost_reach.total<<endl;
                    //cout<<"Outgoing edges from node: "<<tree->nodes[j].node_id<<" to nodes: "<< tree->nodes[i].node_id<<endl;

                    num_incoming_edges++;
                    if(num_incoming_edges > 1)
                    {
                        cout<<"No two parents check for tree: "<<tree->name<<endl;

                        cout<<"ERROR Node: "<<tree->nodes[i].node_id<<" has more than one incoming edge /parent"<<endl;


                        int stop;
                        cin>>stop;
                    }
//                    if(tree->nodes[i].node_id == 0)
//                    {
//                        cout<<"ERROR Start Node has an incoming edge"<<endl;
//                        int stop;
//                        cin>>stop;
//                    }
                }
            }

        }

//        if(num_incoming_edges == 0 && tree->nodes[i].node_id != 1 && tree->nodes[i].node_id != 0)
//        {
//            cout<<"ERROR Node: "<<tree->nodes[i].node_id<<" has NO incoming edge"<<endl;

//            int stop;
//            cin>>stop;
//        }
    }
}


void BiRRTstarPlanner::cost_consistency_check(Rrt_star_tree *tree)
{
    for (int i = 0 ; i < tree->nodes.size() ; i++)
    {
        for(int e = 0 ; e < tree->nodes[i].outgoing_edges.size() ; e++)
        {
            if(tree->nodes[tree->nodes[i].outgoing_edges[e].child_node_id].cost_reach.total < tree->nodes[i].cost_reach.total)
            {
                cout<<"Cost inconsistency in tree: "<<tree->name<<endl;
                cout<<"Cost of child node: "<<tree->nodes[tree->nodes[i].outgoing_edges[e].child_node_id].node_id<<" smaller than cost of parent node: "<<tree->nodes[tree->nodes[i].outgoing_edges[e].child_node_id].parent_id<<" "<<tree->nodes[i].node_id<<endl;
                cout<<"Parent of child node: "<<tree->nodes[tree->nodes[i].outgoing_edges[e].child_node_id].parent_id<<endl;
                cout<<"Cost of child: "<<tree->nodes[tree->nodes[i].outgoing_edges[e].child_node_id].cost_reach.total<<" smaller than cost of parent: "<<tree->nodes[i].cost_reach.total<<endl;

                int test;
                cin>>test;
            }
        }
    }
}





} //end of namespace

