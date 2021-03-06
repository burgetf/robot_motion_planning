/*
 *run_birrt_star_algorithm_omnirob_lbr.cpp
 *
 *  Created on: June 25, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <birrt_star_algorithm/birrt_star.h>
#include <planning_world_builder/planning_world_builder.h>


using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "birrt_star_algorithm_omnirob_base_node");

    //Node Handle
    ros::NodeHandle nh;

    //TODO
    // - Read planning group from terminal input
    string planning_group = "omnirob_base";

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
    //world_builder.insertThreeGatesPassage(wall_thickness, wall_length_ratio);

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
    //start_ee_pose[0] = 3.6;  //X
    //start_ee_pose[1] = 4.5;  //Y
    //start_ee_pose[2] = 0.9;  //Z
    start_ee_pose[0] = 0.0;  //X
    start_ee_pose[1] = 0.0;  //Y
    start_ee_pose[2] = 0.9;  //Z
    start_ee_pose[3] = 0.0;    //RotX
    start_ee_pose[4] = 0.0;    //RotY
    start_ee_pose[5] = 1.57;    //RotZ
    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
    constraint_vec_start_pose[0] = 1; //X
    constraint_vec_start_pose[1] = 1; //Y
    constraint_vec_start_pose[2] = 0; //Z
    constraint_vec_start_pose[3] = 0; //RotX
    constraint_vec_start_pose[4] = 0; //RotY
    constraint_vec_start_pose[5] = 1; //RotZ


    //Set EE Goal endeffector pose and constraint variables
    vector<double> ee_goal_pose(6);
    ee_goal_pose[0] = 0.0;       //X
    ee_goal_pose[1] = 4.5; //Y
    ee_goal_pose[2] = 0.9;                   //Z
    ee_goal_pose[3] = 0.0;    //RotX
    ee_goal_pose[4] = 0.0;    //RotY
    ee_goal_pose[5] = 0.0;    //RotZ
    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
    constraint_vec_goal_pose[0] = 1; //X
    constraint_vec_goal_pose[1] = 1; //Y
    constraint_vec_goal_pose[2] = 0; //Z
    constraint_vec_goal_pose[3] = 0; //RotX
    constraint_vec_goal_pose[4] = 0; //RotY
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
    // -> Syntax: planner.setTaskFrameConstraints(constraint_vector,permitted_coordinate_dev,task_pos_global,task_orient_global);
    // bool task_pos_global -> indicates whether task frame position is expressed w.r.t near node ee pos or always w.r.t start frame ee pos
    // bool task_orient_global -> indicates whether task frame orientation is expressed w.r.t near node ee orientation or always w.r.t start frame ee orientation
    //planner.setTaskFrameConstraints(constraint_vector,permitted_coordinate_dev,true,true);

    //Set edge cost variable weights (to apply motion preferences)
    vector<double> edge_cost_weights(3);
    edge_cost_weights[0] = 1.0; //base_x
    edge_cost_weights[1] = 1.0; //base_y
    edge_cost_weights[2] = 3.0; //base_theta
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
    planner.run_planner(SEARCH_SPACE, MAX_ITERATIONS_OR_TIME, MAX_ITERATIONS_TIME, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME, planner_run_number);


    //End of planning phase
    cout<<"..... Planner finished"<<endl;

    ros::shutdown();

    return 0;
}


