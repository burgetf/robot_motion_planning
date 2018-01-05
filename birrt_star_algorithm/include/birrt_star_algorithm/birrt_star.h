/*
 * birrt_star.h
 *
 *  Created on: July 06, 2015
 *      Author: Felix Burget
 */


// --- Includes -- 
#include <ros/ros.h>
#include <planner_data_structures/data_structs.h>
#include <kuka_motion_control/control_laws.h>
#include <validity_checker/feasibility_checker.h>
#include <planning_heuristics/distance_heuristics.h>
#include <planning_world_builder/planning_world_builder.h>

//Needed for planning with the real robot
//#include <robot_interface_definition/robot_interface.h>

//#include <rrt_star_algorithm/ProlateHyperspheroid.h>
#include <omp.h>


#ifndef BIRRT_STAR_H
#define BIRRT_STAR_H

// --Namespaces --
using namespace std;


namespace birrt_star_motion_planning{


class BiRRTstarPlanner //: public robot_interface_definition::RobotInterface
{
	public:
    BiRRTstarPlanner(string planning_group);
    ~BiRRTstarPlanner();

    //Set the planning scene
    void setPlanningSceneInfo(vector<double> size_x, vector<double> size_y, string scene_name, bool show_env_borders = true);
    void setPlanningSceneInfo(planning_world::PlanningWorldBuilder world_builder);

    //Implementation of the virtual function "move" defined in Abstract Class robot_interface_definition
    //bool plan(const Eigen::Affine3d& goal);

    //Move function for base, base+endeffector and endeffector only
    //bool move_base(const Eigen::Affine3d& goal);
    //bool move_base_endeffector(const Eigen::Affine3d& goal);
    //bool move_endeffector(const Eigen::Affine3d& goal);

    //Initialize RRT* Planner (reading start and goal config from file)
    bool init_planner(char *start_goal_config_file, int search_space);

    //Initialize RRT* Planner (given start and goal config)
    bool init_planner(vector<double> start_conf, vector<double> goal_conf, int search_space);

    //Initialize RRT* Planner (given start config and final endeffector pose)
    bool init_planner(vector<double> start_conf, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space);

    //Initialize RRT* Planner (given start and final endeffector pose)
    bool init_planner(vector<double> ee_start_pose, vector<int> constraint_vec_start_pose, vector<double> ee_goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, int search_space);

    //Initialize RRT* Planner for plannig with the real robot
    bool init_planner_map_goal_pose(const Eigen::Affine3d& goal, const vector<int> constraint_vec_goal_pose, const vector<pair<double,double> > target_coordinate_dev, const string planner_type, bool &planning_needed);
    bool init_planner_map_goal_config(const vector<double> goal, const string planner_type, bool &planning_needed);


    //Read / Write Start and Goal Config
    void writeStartGoalConfig(char *start_goal_config_file, vector<double> start_config, vector<double> goal_config);
    bool readStartGoalConfig(char *start_goal_config_file, vector<double> &start_config, vector<double> &goal_config);

    //Generate configurations from ee poses
    vector<double> generate_config_from_ee_pose(vector<double> ee_pose, vector<int> constraint_vec_ee_pose, vector<pair<double,double> > coordinate_dev, bool show_motion);
    vector<double> generate_config_from_ee_pose_with_reference_config(vector<double> ee_pose, vector<int> constraint_vec_ee_pose, vector<pair<double,double> > coordinate_dev, vector<double> mean_init_config, bool show_motion);
    vector<vector<double> > generate_start_goal_config(vector<double> start_pose, vector<int> constraint_vec_start_pose, vector<double> goal_pose, vector<int> constraint_vec_goal_pose, vector<pair<double,double> > coordinate_dev, bool show_motion );

    //Compute IK for given endeffector pose
    vector<double> findIKSolution(vector<double> goal_ee_pose, vector<int> constraint_vec, vector<pair<double,double> > coordinate_dev, bool show_motion);
    //Compute IK for given endeffector goal pose (given a specific initial config)
    vector<double> findIKSolution(vector<double> goal_ee_pose, vector<int> constraint_vec, vector<pair<double,double> > coordinate_dev, vector<double> mean_init_config, bool show_motion);

    //Attach/Detach an given Object to the End-effector
    void attachObject(moveit_msgs::AttachedCollisionObject attached_object);
    void detachObject(moveit_msgs::AttachedCollisionObject attached_object);

    //Function to set task frame constraints
    //Note: if no task_frame (w.r.t base_link) is given the task_frame will be identical to the end-effector frame at the start config
    void setTaskFrameConstraints(vector<int> cv, vector<pair<double, double> > coordinate_dev, bool task_pos_global, bool task_orient_global, boost::optional<KDL::Frame> task_frame = boost::none);
    //Function to set parameterized task frame
    //void setParameterizedTaskFrame(vector<int> cv, vector<pair<double,double> > coordinate_dev, bool task_pos_global, bool task_orient_global);
    //Function to set fixed task frame
    //void setFixedTaskFrame(vector<int> cv, vector<pair<double,double> > coordinate_dev, KDL::Frame task_frame);

    //Function to set edge cost weights (to punish certain joint motions)
    void setEdgeCostWeights(vector<double> ecw);

    //Function to set the configuration projection error threshold (used in constraint satisfaction function)
    //void setConfigProjectionErrorThreshold(double threshold);

    //Activate/Deactivate Tree Optimization Features
    void activateTreeOptimization();
    void deactivateTreeOptimization();

    //Activate/Deactivate Informed Sampling Heuristic Features
    void activateInformedSampling();
    void deactivateInformedSampling();

    //Run RRT* Planner (given max. planning time or iterations)
    bool run_planner(int search_space, bool flag_iter_or_time, double max_iter_time, bool show_tree_vis, double iter_sleep, int planner_run_number = 0);

    //Reset RRT* Planner Data
    void reset_planner_and_config(); //reset everything, i.e. planner data and configuration such as edge cost weights, environment size, constraints
    void reset_planner_only(); //reset planner only (planner configuration and environment data is kept)
    void reset_planner_to_initial_state(); //reset planner to initial trees (containing only start and goal config) -> planner configuration and environment data is kept

    // ----- Getters -----

    //Get the planning frame from the SRDF description
    string getPlanningFrameFromSRDF(string robot_desciption_param);

    //Get the planned joint trajectory
    vector<vector<double> > getJointTrajectory();
    //Get the planned endeffector trajectory
    vector<vector<double> > getEndeffectorTrajectory();
    //Get the number of joints the planning group is composed of
    int getNumJointsPlanningGroup();
    int getNumPrismaticJointsPlanningGroup();
    int getNumRevoluteJointsPlanningGroup();




    private:


    //-- Class Objects --

    //Node handle
    ros::NodeHandle m_nh;

    // Path to package (to be read from param server)
    string m_planner_package_path;

    //Path to start goal config files
    string m_terminal_configs_path;

    //Planning World and Environment Size
    boost::shared_ptr<planning_world::PlanningWorldBuilder> m_planning_world;
    vector<double> m_env_size_x; //m_env_size_x[0] = size in negative x dir / m_env_size_x[1] = size in positive x dir
    vector<double> m_env_size_y; //m_env_size_y[0] = size in negative y dir / m_env_size_y[1] = size in positive y dir

    //Planning Scene Monitor (used for Collision Checking)
    //planning_scene_monitor::PlanningSceneMonitorPtr m_planning_scene_monitor;

    //Generate KDL Tree from Kuka LBR urdf
    // -> used to generate samples for the kinematic chain of the robot
    boost::shared_ptr<kuka_motion_controller::KDLRobotModel> m_KDLRobotModel;

    //Create RobotController Object for Motion Control of the robot
    // -> used to connect samples of the RRT* planner
    boost::shared_ptr<kuka_motion_controller::RobotController> m_RobotMotionController;

    //FeasibilityChecker to check configurations/nodes and edges for collisions
    boost::shared_ptr<state_feasibility_checker::FeasibilityChecker> m_FeasibilityChecker;

    //Heuristics for measuring distances between nodes
    heuristics_motion_planning::DistanceHeuristics m_Heuristic;

    //Group for which planning is performed (set in constructor + must be a group defined in the robot srdf)
    string m_planning_group;

    //Frame in which planning is performed
    // -> "/map" frame if planning is performed in real environment
    // -> "/base_link" frame if planning is performed in simulated environment
    string m_planning_frame;

    //Kinematic Chain of robot
    KDL::Chain m_manipulator_chain;

    //Joint Names of KDL::Chain
    vector<string> m_joint_names;

    //Name of Base Link Frame of Kinematic Chain
    string m_base_link_name;

    //Number of joints
    int m_num_joints;
    int m_num_joints_revolute;
    int m_num_joints_prismatic;


    //RRT* trees
    Rrt_star_tree m_start_tree;
    Rrt_star_tree m_goal_tree;

    //Start and Goal Endeffector Pose
    vector<double> m_ee_start_pose;
    vector<double> m_ee_goal_pose;

    //Start and Goal Configuration
    vector<double> m_config_start_pose;
    vector<double> m_config_goal_pose;

    //Maximal Planner iterations/time and actually executed planner iterations/time
    int m_max_planner_iter;
    double m_max_planner_time;
    int m_executed_planner_iter;
    double m_executed_planner_time;

    //Iteration when first and last solution is found
    int m_first_solution_iter;
    int m_last_solution_iter;

    //Tree Optimization Flag
    // -> If "activated" near nodes are considered in the nearest neighbour search
    // -> Nodes are rewired when a new node is added
    // -> If "deactivated" the planner reduces to the standard RRT-CONNECT
    bool m_tree_optimization_active;

    //Informed Sampling Heuristic Flag
    // -> If "activated" , samples are drawn from an informed subset once a solution is found
    // -> If "deactivated" , samples are drawn uniformly from the C-Space
    bool m_informed_sampling_active;

    //Bidirectional planner type
    string m_planner_type;

    //Distance threshold for workspace samples(Defines "nearness" of ndoes in "find_near_vertices"-function)
    double m_near_threshold_control;

    //Distance threshold for C-Space samples(Defines "nearness" of ndoes in "find_near_vertices"-function)
    double m_near_threshold_interpolation;

    //Weights for individual edge cost components (to punish motions of some variables stronger/less than the one of others)
    vector<double> m_edge_cost_weights;

    //Number of points for C-Space interpolation
    int m_num_traj_segments_interp;

    //Step width from nearest neighbour in the direction of random sample
    double m_unconstraint_extend_step_factor;

    //Flag indicating whether single or multiple steps are performed towards a random config
    bool m_single_extend_step;

    //Maximum planning time and path optimality treshold from param server
    double  m_max_planning_time;
    double m_path_optimality_treshold;

    //Cost solution path available
    bool m_solution_path_available;


    //Theoretical Minimum Cost for Solution Path (linear interpolation betweeen start and goal pose/config)
    vector<double> m_cost_theoretical_solution_path;

    //Cost of selected best solution path (total cost + cost for prismatic and rotational joints)
    double m_cost_best_solution_path;
    double m_cost_best_solution_path_revolute;
    double m_cost_best_solution_path_prismatic;

    //Tree and nodes that have established a connection
    string m_connected_tree_name;
    Node m_node_tree_B; //Node of the tree "m_connected_tree_name" that has connected "to m_node_tee_A"
    Node m_node_tree_A; //Node of the other tree that has been reached by tree "m_connected_tree_name"

    //Timer for measuring performance of planner
    timeval m_timer;
    double m_time_planning_start;
    double m_time_planning_end;
    double m_time_first_solution;
    double m_time_last_solution;


    //Flag indicating whether transform between map and base_link is available
    bool m_transform_map_to_base_available;
    //Map to base_link transform
    tf::StampedTransform m_transform_map_to_base;

    //Random number generator to generate a number between 0 and 1 for sampling the unit n-dimensional ball
    random_numbers::RandomNumberGenerator  m_random_number_generator;



    // ++ Class Functions ++


    //++ Cartesian Space Search (with local  Controller) ++

    //Compute pose of endeffector in configuration
    vector<double> computeEEPose(vector<double> start_conf);
    vector<double> computeEEPose(KDL::JntArray start_conf);

    //Sample Endeffector pose (used for Control-based tree expansion)
    vector<double> sampleEEpose();

    //Sample Endeffector pose from Informed Subset / Ellipse
    vector<double> sampleEEposefromEllipse();

    //Find the nearest neighbour (node storing the ee pose closest to the sampled pose)
    Node find_nearest_neighbour_control(Rrt_star_tree *tree, vector<double> x_rand);

    //Connect Nodes (running Variable DLS Controller)
    kuka_motion_controller::Status connectNodesControl(Rrt_star_tree *tree, Node near_node, Node end_node, Node &x_new, Edge &e_new);

    //Compute the cost of an edge connecting two nodes
    double compute_edge_cost_control(vector<vector<double> > ee_traj);

    //Find the set of vertices in the vicinity of x_new
    vector<int> find_near_vertices_control(Rrt_star_tree *tree, Node x_new);

    //Choose Parent for x_new minimizing the cost of reaching x_new (given the set of vertices surrounding x_new as potential parents)
    bool choose_node_parent_control(Rrt_star_tree *tree, vector<int> near_vertices, Node nn_node, Edge &e_new, Node &x_new);

    //Rewire Nodes of the Tree, i.e. checking if some if the nodes can be reached by a lower cost path
    void rewireTreeControl(Rrt_star_tree *tree, vector<int> near_vertices, Node x_new, bool show_tree_vis);

    //Try to connect the two RRT* Trees given the new node added to "tree_A" and its nearest neighbour in "tree_B"
    void connectGraphsControl(Rrt_star_tree *tree, Node x_connect, Node x_new, bool show_tree_vis);




    //++ C-Space Search (with local interpolation) ++

    //Initialize constant parameters for Joint Config sampling from Ellipse
    void jointConfigEllipseInitialization();

    //Sample Configuration (as JntArray) from Ellipse containing configurations that may improve current solution
    KDL::JntArray sampleJointConfigfromEllipse_JntArray();

    //Sample Configuration (as std::vector)
    vector<double> sampleJointConfig_Vector();
    //Sample Configuration (as JntArray)
    KDL::JntArray sampleJointConfig_JntArray();

    //Sample Configuration around mean config vector(as std::vector)
    vector<double> sampleJointConfig_Vector(vector<double> mean_config, double std_dev);
    //Sample Configuration around mean config vector(as JntArray)
    KDL::JntArray sampleJointConfig_JntArray(vector<double> mean_config, double std_dev);

    //Transform sample from base_link frame to map frame
    void transform_sample_to_map_frame(KDL::JntArray& sample_conf);
    void transform_sample_to_map_frame(vector<double> &sample_conf);
    //Transform sample from map frame to base_link frame
    void transform_sample_to_base_link_frame(KDL::JntArray &sample_conf);
    void transform_sample_to_base_link_frame(vector<double> &sample_conf);


    //Tree expansions step
    bool expandTree(Rrt_star_tree *tree, Node nn_node, Node x_rand, Node &x_new, Edge &e_new, bool show_tree_vis);

    //Connect Nodes (by interpolating configurations)
    bool connectNodesInterpolation(Rrt_star_tree *tree, Node near_node, Node end_node, int num_points, Node &x_new, Edge &e_new);

    //Interpolate configurations of near_node and end_node
    bool interpolateConfigurations(Node near_node, Node end_node, int num_points ,vector<vector<double> > &joint_traj, vector<vector<double> > &ee_traj);

    //Compute the cost of an edge connecting two nodes
    vector<double> compute_edge_cost_interpolation(vector<vector<double> > joint_traj);


    //Find the nearest neighbour (node storing the ee pose closest to the sampled config)
    Node find_nearest_neighbour_interpolation(Rrt_star_tree *tree, vector<double> x_rand);

    //Find the set of vertices in the vicinity of x_new
    vector<int> find_near_vertices_interpolation(Rrt_star_tree *tree, Node x_new);

    //Choose Parent for x_new minimizing the cost of reaching x_new (given the set of vertices surrounding x_new as potential parents)
    bool choose_node_parent_interpolation(Rrt_star_tree *tree, vector<int> near_vertices, Node nn_node, Edge &e_new, Node &x_new, bool show_tree_vis);

    //Rewire Nodes of the Tree, i.e. checking if some if the nodes can be reached by a lower cost path
    void rewireTreeInterpolation(Rrt_star_tree *tree, vector<int> near_vertices, Node x_new, bool show_tree_vis);

    //Update cost of nodes following to a rewired near_vertices
    void recursiveNodeCostUpdate(Rrt_star_tree *tree, Node tree_vertex, vector<double> cost_reduction, bool show_tree_vis);

    //Insert Node and Edge into the RRT* Tree
    void insertNode(Rrt_star_tree *tree, Edge e_new, Node x_new, bool show_tree_vis);

    //Try to connect the two RRT* Trees given the new node added to "tree_A" and its nearest neighbour in "tree_B"
    void connectGraphsInterpolation(Rrt_star_tree *tree, Node x_connect, Node x_new, bool show_tree_vis);

    //Check whether a solution has been found (not neccessarily the optimal one)
    //bool solution_path_available();


    //Compute and show the current solution path (computes node sequence for line strip for visualization)
    // -> node_tree_A and node_tree_B are the same nodes exrpessed w.r.t tree_A and tree_B respectively
    // -> "tree_name" tells us which tree has established the connection (either start or goal tree)
    void showCurrentSolutionPath(string connected_tree_name, Node node_tree_B, Node node_tree_A);



    //++ Joint config sampling from Ellipse ++

    //Center of the 7-dimensional unit ball
    Eigen::VectorXd  m_ball_center_revolute;
    Eigen::VectorXd  m_ball_center_prismatic;

    //Rotation Matrix "C"
    Eigen::MatrixXd  m_rotation_C_revolute;
    Eigen::MatrixXd  m_rotation_C_prismatic;




    //++ Constraint motion planning ++

    //Flag indicating whether constraint motion planning is active
    bool m_constraint_active;

    //Flags indicating whether the constraint is global (i.e the task frame always corresponds to the start ee frame)
    //or local (i.e the task frame always corresponds to the current ee frame)
    bool m_task_pos_global;
    bool m_task_orient_global;

    //Constraint Selection Vector
    // -> specifying which axes of the task frame permit valid displacement
    vector<int> m_constraint_vector;

    //Permitted deviation for constrained coordinates
    // -> specifies an lower and upper deviation boundary
    vector<pair<double,double> > m_coordinate_dev;

    //Position and orientation of Task Frame
    // -> Position defined by nearest neighbour (i.e. locally)
    // -> Orientation defined globally by start ee pose
    KDL::Frame m_task_frame;

    //Transformation between end-effector and task frame after grasping
    KDL::Frame m_grasp_transform;

    //Step width from nearest neighbour in the direction of random sample
    double m_constraint_extend_step_factor;

    //Perform a step from nearest neighbour towards random sample
    bool stepTowardsRandSample(Node nn_node, Node &x_rand, double extend_step_factor);

    //Permitted sample projection error
    //double m_projection_error_threshold;

    //Minimum distance between near sample and projected sample
    // -> to avoid random samples being projected back onto near sample
    double m_min_projection_distance;

    //Maximum permitted task error for intermediate ee poses between two given configurations
    //double m_max_task_error_interpolation;

    //Maximum iterations for projection operation
    int m_max_projection_iter;

    //Maximum number of near nodes considered in "choose_node_parent_iterpolation" and "rewireTreeInterpolation"
    // -> "choose_node_parent_iterpolation" : Only first "m_max_near_nodes" nodes are considered
    // -> "rewireTreeInterpolation" : Only last "m_max_near_nodes" nodes are considered
    // Note: Near Nodes are stored in ascending order of cost-to-reach
    int m_max_near_nodes;


    //++ Planning with Attached Objects ++

    //Attached object
    moveit_msgs::AttachedCollisionObject m_attached_object;


    // ++ RRT* Tree visualization ++

    //Scale for node and edge markers
    double m_node_marker_scale;
    double m_edge_marker_scale;
    double m_terminal_nodes_marker_scale;
    double m_solution_path_ee_marker_scale;
    double m_solution_path_base_marker_scale;


    // MarkerArray for start and goal node (SPHERES)
    visualization_msgs::MarkerArray m_terminal_nodes_marker_array_msg; //array storing terminal nodes

    // Marker to add/remove edges (LINE_LIST)
    visualization_msgs::MarkerArray m_start_tree_add_edge_marker_array_msg; //Vector of Line Strips / Edges
    visualization_msgs::MarkerArray m_goal_tree_add_edge_marker_array_msg; //Vector of Line Strips / Edges


    // Marker for nodes (SPHERE_LIST)
    visualization_msgs::Marker m_start_tree_add_nodes_marker; //add nodes to tree visualization
    visualization_msgs::Marker m_goal_tree_add_nodes_marker; //add nodes to tree visualization


    //Tree edges and node Publisher
    ros::Publisher m_start_tree_edge_pub;
    ros::Publisher m_start_tree_node_pub;
    ros::Publisher m_goal_tree_edge_pub;
    ros::Publisher m_goal_tree_node_pub;

    //Publisher for terminal nodes (start and goal node)
    ros::Publisher m_tree_terminal_nodes_pub;

    //Solution path publisher (for ee trajectory)
    ros::Publisher m_ee_solution_path_pub;
    //Solution path publisher (for base trajectory)
    ros::Publisher m_base_solution_path_pub;

    //Publisher for prismatic C-Space ellipse (informed subset for base)
    ros::Publisher m_base_ellipse_pub;


    //Functions for nodes and edges
    void add_tree_edge_vis(string tree_name,Edge new_edge);
    void remove_tree_edge_vis(string tree_name,Edge old_edge);
    void add_tree_node_vis(string tree_name,Node new_node);
    void add_tree_node_vis(string tree_name, Node new_node, vector<double> color_rgb);

    //Functions for informed sampling ellipses
    void drawBaseEllipse();

    // ++ Store results of RRT* planner ()++

    //Result of Motion Planning
    int m_planner_success;

    //Arrays storing the result
    vector< vector<double> > m_result_joint_trajectory;
    vector< vector<double> > m_result_ee_trajectory;

    //Evolution of the best solution cost
    vector< vector<double> > m_solution_cost_trajectory;

    //Path to files storing the trajectories generated by RRT*
    char* m_file_path_joint_trajectory;
    char* m_file_path_ee_trajectory;

    //Compute the final solution path (fills array for joint and endeffector trajectory and writes them to file)
    void computeFinalSolutionPathTrajectories();


    //Path to files storing the Planner Statistics
    char* m_file_path_planner_statistics;

    //Path to files storing the Solution Path Cost Evolution
    char* m_file_path_cost_evolution;

    //Write Planner Statistics to File
    void writePlannerStatistics(char *statistics_file, char *cost_evolution_file);

    //Method for Solution Path Smoothing
    //void basic_solution_path_smoothing(vector< vector<double> > raw_joint_trajectory);


    //Subscriber to get current joint state from real lbr arm
    ros::Subscriber m_lbr_joint_state_sub;
    //LBR Joint State Subscriber Callback
    void callback_lbr_joint_states(const sensor_msgs::JointState::ConstPtr& msg);
    //Flag indicating that lbr joint state is available
    bool m_lbr_joint_state_received;
    //Current Lbr joint state
    vector<double> m_lbr_joint_state;

    //Namespace prefix for robot
    string m_ns_prefix_robot;

    // ++ Information Publisher ++

    //Publisher for planning progress (0 to 100%)
    ros::Publisher m_pub_planning_progress;


    // ++ Functions and Variables for consistency checks ++

    //Variable to test for loops in the tree
    vector<int> testing_;
    //Tree consistency checks
    void no_two_parents_check(Rrt_star_tree *tree);
    void cost_consistency_check(Rrt_star_tree *tree);
};




}//end of namespace

#endif // BiRRT_STAR_H

