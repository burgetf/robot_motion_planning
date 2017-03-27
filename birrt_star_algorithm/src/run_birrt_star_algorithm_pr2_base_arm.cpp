/*
 *run_birrt_star_algorithm_pr2_base_arm.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <birrt_star_algorithm/birrt_star.h>
#include <planning_world_builder/planning_world_builder.h>

using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "birrt_star_algorithm_pr2_base_arm_node");

    //Node Handle
    ros::NodeHandle nh;

    //TODO
    // - Read planning group from terminal input
    string planning_group = "pr2_base_arm";

    // -------------------- Planning World Setup ----------------------------
    //Load Planning World
    planning_world::PlanningWorldBuilder world_builder("robot_description", planning_group);
    //Enter Environment Borders
    vector<double> env_size_x(2);
    env_size_x[0] = -10.0;
    env_size_x[1] = 10.0;
    vector<double> env_size_y(2);
    env_size_y[0] = -10.0;
    env_size_y[1] = 10.0;
    double env_size_z = 0.6;
    world_builder.insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);

    //Enter wall
    vector<double> wall_pos(3);
    wall_pos[0] = 0.0;
    wall_pos[1] = 0.0;
    wall_pos[2] = 0.0;
    vector<double> wall_dim(3);
    wall_dim[0] = 0.2;
    wall_dim[1] = 6.0;
    wall_dim[2] = 0.6;
    //world_builder.insertWall(wall_pos,wall_dim);

    //Enter Maze
    int num_walls = 4;
    double wall_thickness = 0.2;
    double wall_length_ratio = 0.5;
    //world_builder.insertMaze(num_walls,wall_thickness,wall_length_ratio);

    //Insert Narrow Passage
    wall_length_ratio = 0.47;
    //world_builder.insertNarrowPassage(wall_thickness, wall_length_ratio);

    //Insert "Three Passages" scenarion (providing suboptimal paths)
    wall_length_ratio = 0.3;
    world_builder.insertThreeGatesPassage(wall_thickness, wall_length_ratio);

    //Insert TwoRooms Office
    wall_length_ratio = 0.5;
    double width_narrow_passage = 1.1; // in m (omnirob has width of 0.67m)
    //world_builder.insertTwoRoomsOffice(wall_thickness,width_narrow_passage);

    //Insert Tunnel Scenario
    double tunnel_height = 1.4;
    double tunnel_width = 1.1; //Note : omnirob has width of 0.67m
    //world_builder.insertTunnelPassage(tunnel_width, tunnel_height, 0.2);

    //Insert Narrow Corridor Scenario
    double corridor_width = 2.0;
    //world_builder.insertNarrowCorridor(corridor_width, 0.2);




    // -------------------- Attached Objects Setup ----------------------------

    //vector<double> obj_pos(3);
    //obj_pos[0] = 8.0;   //x pos
    //obj_pos[1] = -3.5;  //y pos
    //obj_pos[2] = 0.9;   //z pos
    //vector<double> obs_dim(3);
    //obs_dim[0] = 0.1;   //x-width dim
    //obs_dim[1] = 0.1;   //y-length dim
    //obs_dim[2] = 0.1;   //z-height dim
    //moveit_msgs::AttachedCollisionObject trolley = world_builder.insertManipulableObject("box", obj_pos, obs_dim);


    //Dimension of cart
    vector<double> obs_dim(3);
    obs_dim[0] = 0.4;   //x-width dim
    obs_dim[1] = 0.5;   //y-length dim
    obs_dim[2] = 0.7;   //z-height dim

    //Start Position of cart
    vector<double> obj_pos_start(3);
    obj_pos_start[0] = 8.0;   //x pos
    obj_pos_start[1] = -3.5;  //y pos
    obj_pos_start[2] = 0.0;   //z pos
    //moveit_msgs::AttachedCollisionObject cart = world_builder.insertManipulableCart("cart", obj_pos_start, obs_dim);

    //Goal Position of cart
    vector<double> obj_pos_goal(3);
    obj_pos_goal[0] = -8.0;   //x pos
    obj_pos_goal[1] = -3.0;  //y pos
    obj_pos_goal[2] = 0.0;   //z pos
    //moveit_msgs::AttachedCollisionObject cart_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, obs_dim);


    // -------------------- Planner Setup ----------------------------

    //birrt_star_motion_planning::BiRRTstarPlanner planner("kuka_complete_arm");
    birrt_star_motion_planning::BiRRTstarPlanner planner(planning_group);

    //Set planning scene
    planner.setPlanningSceneInfo(world_builder);

    //Set default values
    int SEARCH_SPACE = 1;
    int MAX_ITERATIONS_TIME = 200;
    bool MAX_ITERATIONS_OR_TIME = 1;
    bool RVIZ_SHOW_TREE = 1;
    double ITERATION_SLEEP_TIME = 0.0;


    //Set search space for planner (0 = cartesian , 1 = c-space)
    if(argc > 1) {
      stringstream s(argv[1]);
      s >> SEARCH_SPACE;
    }

    //Set maximum iterations for planner
    if(argc > 2) {
      stringstream s(argv[2]);
      s >> MAX_ITERATIONS_OR_TIME;
    }

    //Set maximum iterations for planner
    if(argc > 3) {
      stringstream s(argv[3]);
      s >> MAX_ITERATIONS_TIME;
    }

    //Activate/Deactivate tree visualization in RVIZ
    if(argc > 4) {
      stringstream s(argv[4]);
      s >> RVIZ_SHOW_TREE;
    }

//    //Set sleep time between planner iterations (used to better see tree cosntruction process in RVIZ)
//    if(argc > 4) {
//      stringstream s(argv[4]);
//      s >> ITERATION_SLEEP_TIME;
//    }


//    //Set Start configuration and endeffector goal pose
//    vector<double> start_conf(7);
//    start_conf[0] = 0.74;
//    start_conf[1] =-0.61;
//    start_conf[2] =-0.74;
//    start_conf[3] =-0.80;
//    start_conf[4] =-1.45;
//    start_conf[5] =0.77;
//    start_conf[6] =0.3;


//    vector<double> ee_goal_pose(7);
//    ee_goal_pose[0] = -0.108012;
//    ee_goal_pose[1] = -0.59835;  //0.59835;
//    ee_goal_pose[2] = 0.215184;
//    ee_goal_pose[3] = 1000.0;
//    ee_goal_pose[4] = 1000.0;
//    ee_goal_pose[5] = 1000.0;
//    ee_goal_pose[6] = 1000.0;


//    //Set EE Start endeffector pose and constraint variables
//    vector<double> start_ee_pose(6);
//    start_ee_pose[0] = -0.6; //X
//    start_ee_pose[1] = 0.4;  //Y
//    start_ee_pose[2] = 0.3;  //Z
//    start_ee_pose[3] = 0.0;    //RotX
//    start_ee_pose[4] = -1.57;    //RotY
//    start_ee_pose[5] = 0.0;    //RotZ
//    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
//    constraint_vec_start_pose[0] = 1; //X
//    constraint_vec_start_pose[1] = 1; //Y
//    constraint_vec_start_pose[2] = 1; //Z
//    constraint_vec_start_pose[3] = 1; //RotX
//    constraint_vec_start_pose[4] = 1; //RotY
//    constraint_vec_start_pose[5] = 1; //RotZ


//    //Set EE Goal endeffector pose and constraint variables
//    vector<double> ee_goal_pose(6);
//    ee_goal_pose[0] = -0.1; //X
//    ee_goal_pose[1] = 0.4;  //Y
//    ee_goal_pose[2] = 0.3;  //Z
//    ee_goal_pose[3] = 0.0;    //RotX
//    ee_goal_pose[4] = -1.57;    //RotY
//    ee_goal_pose[5] = 0.0;    //RotZ
//    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
//    constraint_vec_goal_pose[0] = 1; //X
//    constraint_vec_goal_pose[1] = 1; //Y
//    constraint_vec_goal_pose[2] = 1; //Z
//    constraint_vec_goal_pose[3] = 1; //RotX
//    constraint_vec_goal_pose[4] = 1; //RotY
//    constraint_vec_goal_pose[5] = 1; //RotZ



    //Global Frame (top view) -> Shows end-effector in base_link frame
    //       ^ Y
    //       |
    //       | f
    //       |Z
    //   f   X-----> X
    //
    //         f

    //Set EE Start endeffector pose and constraint variables
    vector<double> start_ee_pose(6);
    //start_ee_pose[0] = 8.0;//3.6; //X
    //start_ee_pose[1] = -3.0;  //Y
    //start_ee_pose[2] = 0.9;  //Z
    start_ee_pose[0] = obj_pos_start[0];//3.6;  //X
    start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
    start_ee_pose[2] = 0.9;//obj_pos[2];        //Z
    start_ee_pose[3] = 1.57;    //RotX
    start_ee_pose[4] = 0.0;    //RotY
    start_ee_pose[5] = 1.57;    //RotZ
    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
    constraint_vec_start_pose[0] = 1; //X
    constraint_vec_start_pose[1] = 1; //Y
    constraint_vec_start_pose[2] = 1; //Z
    constraint_vec_start_pose[3] = 1; //RotX
    constraint_vec_start_pose[4] = 1; //RotY
    constraint_vec_start_pose[5] = 1; //RotZ


    //Set EE Goal endeffector pose and constraint variables
    vector<double> ee_goal_pose(6);
    ee_goal_pose[0] = obj_pos_goal[0];       //X
    ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
    ee_goal_pose[2] = 0.9;                   //Z
    ee_goal_pose[3] = 1.57;    //RotX
    ee_goal_pose[4] = 0.0;    //RotY
    ee_goal_pose[5] = 1.57;    //RotZ
    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
    constraint_vec_goal_pose[0] = 1; //X
    constraint_vec_goal_pose[1] = 1; //Y
    constraint_vec_goal_pose[2] = 1; //Z
    constraint_vec_goal_pose[3] = 1; //RotX
    constraint_vec_goal_pose[4] = 1; //RotY
    constraint_vec_goal_pose[5] = 1; //RotZ

    //Permitted displacement for ee coordinates w.r.t desired target frame
    vector<pair<double,double> > target_coordinate_dev(6);
    target_coordinate_dev[0].first = -0.005;    //negative X deviation [m]
    target_coordinate_dev[0].second = 0.005;    //positive X deviation
    target_coordinate_dev[1].first = -0.005;    //negative Y deviation
    target_coordinate_dev[1].second = 0.005;    //positive Y deviation
    target_coordinate_dev[2].first = -0.005;    //negative Z deviation
    target_coordinate_dev[2].second = 0.005;    //positive Z deviation
    target_coordinate_dev[3].first = -0.05;     //negative Xrot deviation [rad]
    target_coordinate_dev[3].second = 0.05;     //positive Xrot deviation
    target_coordinate_dev[4].first = -0.05;     //negative Yrot deviation
    target_coordinate_dev[4].second = 0.05;     //positive Yrot deviation
    target_coordinate_dev[5].first = -0.05;     //negative Zrot deviation
    target_coordinate_dev[5].second = 0.05;     //positive Zrot deviation


    //Initialize planner (with start and ee goal pose)
    planner.init_planner(start_ee_pose, constraint_vec_start_pose, ee_goal_pose, constraint_vec_goal_pose, target_coordinate_dev, SEARCH_SPACE);


    //Remove the cart at the goal position
    //world_builder.deleteCollisionObject(cart_goal);

    //Attach the cart to the endeffector being in the start pose before planning
    //planner.attachObject(cart);


    //Set constraint parameters / permitted axes for displacement (x,y,z,roll,pitch,yaw) relative to start ee pose during planning
    //  1 -> constraint
    //  0 -> unconstraint
    vector<int> constraint_vector(6);
    constraint_vector[0] = 1.0; //X translation
    constraint_vector[1] = 0.0; //Y translation
    constraint_vector[2] = 0.0; //Z translation
    constraint_vector[3] = 0.0; //X rotation
    constraint_vector[4] = 1.0; //Y rotation
    constraint_vector[5] = 1.0; //Z rotation
    //Permitted displacement for ee coordinates w.r.t task frame
    vector<pair<double,double> > permitted_coordinate_dev(6);
    permitted_coordinate_dev[0].first = 0.0;    //negative X deviation [m]
    permitted_coordinate_dev[0].second = 0.0;   //positive X deviation
    permitted_coordinate_dev[1].first = 0.0;    //negative Y deviation
    permitted_coordinate_dev[1].second = 0.0;   //positive Y deviation
    permitted_coordinate_dev[2].first = 0.0;    //negative Z deviation
    permitted_coordinate_dev[2].second = 0.0;   //positive Z deviation
    permitted_coordinate_dev[3].first = 0.0;    //negative Xrot deviation [rad]
    permitted_coordinate_dev[3].second = 0.0;   //positive Xrot deviation
    permitted_coordinate_dev[4].first = -0.52;    //negative Yrot deviation
    permitted_coordinate_dev[4].second = 0.0;   //positive Yrot deviation
    permitted_coordinate_dev[5].first = 0.0;    //negative Zrot deviation
    permitted_coordinate_dev[5].second = 0.0;   //positive Zrot deviation
    //Activate the constraint
    // -> Syntax: planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, bool task_pos_global, bool task_orient_global);
    // bool task_pos_global -> indicates whether task frame position is expressed w.r.t near node ee pos or always w.r.t start frame ee pos
    // bool task_orient_global -> indicates whether task frame orientation is expressed w.r.t near node ee orientation or always w.r.t start frame ee orientation
    //planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, true, true);


    //Set edge cost variable weights (to apply motion preferences)
    vector<double> edge_cost_weights(13);
    edge_cost_weights[0] = 1.0; //base_x
    edge_cost_weights[1] = 1.0; //base_y
    edge_cost_weights[2] = 1.0; //base_theta
    edge_cost_weights[3] = 1.0; //manipulator joint 1
    edge_cost_weights[4] = 1.0; //manipulator joint 2
    edge_cost_weights[5] = 1.0; //manipulator joint 3
    edge_cost_weights[6] = 1.0; //manipulator joint 4
    edge_cost_weights[7] = 1.0; //manipulator joint 5
    edge_cost_weights[8] = 1.0; //manipulator joint 6
    edge_cost_weights[9] = 1.0; //manipulator joint 7
    edge_cost_weights[10] = 1.0; //manipulator joint 8
    edge_cost_weights[11] = 1.0; //manipulator joint 9
    edge_cost_weights[12] = 1.0; //manipulator joint 10
    planner.setEdgeCostWeights(edge_cost_weights);



    // -------------------- Motion Planning Execution ----------------------------
    if(SEARCH_SPACE == 0)
        cout<<"Control-based Planner running......!"<<endl;
    else if (SEARCH_SPACE == 1)
        cout<<"C-Space Planner running......!"<<endl;
    else
        ROS_ERROR("PLANNER NOT KNOWN!!!");

    //Planner run index
    int planner_run_number = 0;

    //Run planner
    //planner.run_planner(start_conf, ee_goal_pose, SEARCH_SPACE, MAX_ITERATIONS, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME);
    //planner.run_planner(SEARCH_SPACE, 0, MAX_ITERATIONS, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME, planner_run_number);
    planner.run_planner(SEARCH_SPACE, MAX_ITERATIONS_OR_TIME, MAX_ITERATIONS_TIME, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME, planner_run_number);


    //Detach the object after planning
    //planner.detachObject(cart);


    //End of planning phase
    cout<<"..... Planner finished"<<endl;

    ros::shutdown();

    return 0;
}


