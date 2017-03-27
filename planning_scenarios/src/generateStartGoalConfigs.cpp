/*
 *generateStartGoalConfig.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <birrt_star_algorithm/birrt_star.h>
#include <planning_world_builder/planning_world_builder.h>


using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "generate_start_goal_config_node");

    //Node Handle
    ros::NodeHandle nh;
    
    //Read package path from parameter server
    //string terminal_configs_path;
    //nh.param("terminal_configs_path", terminal_configs_path, std::string("/home/burgetf/catkin_ws/src/robot_motion_planning/planning_scenarios/Start_Goal_Configurations"));

    //Get package path of "planning_scenarios"
    string terminal_configs_path;
    terminal_configs_path = ros::package::getPath("planning_scenarios");
    
    //Set planning group
    //string planning_group = "omnirob_lbr_sdh";
    
    
    // -------------------- Get Scenario Setup ----------------------------
    
    //Set default values
    string SCENARIO_NAME = "three_gates";
    string PLANNING_GROUP = "omnirob_lbr_sdh";
    bool SHOW_MOTION = 1;
    

    //Set scenario
    if(argc > 1) {
      stringstream s(argv[1]);
      s >> SCENARIO_NAME;
    }
    if(argc > 2) {
      stringstream s(argv[2]);
      s >> PLANNING_GROUP;
    }
    //Activate/Deactivate tree visualization in RVIZ
    if(argc > 3) {
      stringstream s(argv[3]);
      s >> SHOW_MOTION;
    }
    
    
    

    // -------------------- Planning World Setup ----------------------------
    //Load Planning World
    planning_world::PlanningWorldBuilder world_builder("robot_description",PLANNING_GROUP);
    //Enter Environment Borders
    vector<double> env_size_x(2);
    env_size_x[0] = -10.0;
    env_size_x[1] = 10.0;
    vector<double> env_size_y(2);
    env_size_y[0] = -10.0;
    env_size_y[1] = 10.0;
    double env_size_z = 2.0;
    world_builder.insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);

	
	//Geometric Parameters
	double wall_thickness = 0.0;
	double wall_length_ratio = 0.0;
	
	
	 //EE Start and Goal endeffector pose and constraint variables
    vector<double> start_ee_pose(6);
    vector<double> ee_goal_pose(6); 
    //Set constraint vector for start and goal pose coordinates
    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
    constraint_vec_start_pose[0] = 1; //X
    constraint_vec_start_pose[1] = 1; //Y
    constraint_vec_start_pose[2] = 1; //Z
    constraint_vec_start_pose[3] = 1; //RotX
    constraint_vec_start_pose[4] = 1; //RotY
    constraint_vec_start_pose[5] = 1; //RotZ
    constraint_vec_goal_pose[0] = 1; //X
    constraint_vec_goal_pose[1] = 1; //Y
    constraint_vec_goal_pose[2] = 1; //Z
    constraint_vec_goal_pose[3] = 1; //RotX
    constraint_vec_goal_pose[4] = 1; //RotY
    constraint_vec_goal_pose[5] = 1; //RotZ
    //Permitted displacement for ee coordinates w.r.t desired target frame
    vector<pair<double,double> > target_coordinate_dev(6);
    target_coordinate_dev[0].first = -0.0005;    //negative X deviation [m]
    target_coordinate_dev[0].second = 0.0005;    //positive X deviation
    target_coordinate_dev[1].first = -0.0005;    //negative Y deviation
    target_coordinate_dev[1].second = 0.0005;    //positive Y deviation
    target_coordinate_dev[2].first = -0.0005;    //negative Z deviation
    target_coordinate_dev[2].second = 0.0005;    //positive Z deviation
    target_coordinate_dev[3].first = -0.0005;     //negative Xrot deviation [rad]
    target_coordinate_dev[3].second = 0.0005;     //positive Xrot deviation
    target_coordinate_dev[4].first = -0.0005;     //negative Yrot deviation
    target_coordinate_dev[4].second = 0.0005;     //positive Yrot deviation
    target_coordinate_dev[5].first = -0.0005;     //negative Zrot deviation
    target_coordinate_dev[5].second = 0.0005;     //positive Zrot deviation

    
    //Dimension of attached_object
    vector<double> cart_dim(3);
    cart_dim[0] = 0.4;   //x-width dim
    cart_dim[1] = 0.5;   //y-length dim
    cart_dim[2] = 0.7;   //z-height dim

    //Dimension of attached_object
    vector<double> glass_dim(3);
    glass_dim[0] = 0.08;   //x-width dim
    glass_dim[1] = 0.08;   //y-length dim
    glass_dim[2] = 0.12;   //z-height dim

    //Start Position of attached_object
    vector<double> obj_pos_start(3);
    moveit_msgs::AttachedCollisionObject attached_object;

    //Goal Position of attached_object
    vector<double> obj_pos_goal(3);
    moveit_msgs::AttachedCollisionObject attached_object_goal;


    //Global Frame (top view)
    //       ^ Y
    //       |
    //       | f
    //       |Z
    //   f   X-----> X
    //
    //         f
    
    
		
	if(SCENARIO_NAME == "empty")
    {
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Set Start Pose
            start_ee_pose[0] = 8.0;     //X
            start_ee_pose[1] = -3.0;    //Y
            start_ee_pose[2] = 0.9;     //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = -8.0;    //X
            ee_goal_pose[1] = 3.5;     //Y
            ee_goal_pose[2] = 0.9;     //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "door")
    {
        //Enter Environment Borders
        vector<double> env_size_x(2);
        env_size_x[0] = -5.0;
        env_size_x[1] = 5.0;
        vector<double> env_size_y(2);
        env_size_y[0] = -5.0;
        env_size_y[1] = 5.0;
        double env_size_z = 2.0;
        world_builder.insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);

        //Position of door hinge
        double door_hinge_pos_x = 0.0;
        double door_hinge_pos_y = 0.0;

        //Position of handle relative to hinge
        double door_hinge_to_handle_dist_x = 0.88;
        double door_hinge_to_handle_dist_y = 0.08;
        double handle_pos_z = 1.05; //Height of handle w.r.t ground plane

        //Length of vector connecting hinge to handle
        double hinge_to_handle_vec_length = sqrt(door_hinge_to_handle_dist_x*door_hinge_to_handle_dist_x + door_hinge_to_handle_dist_y*door_hinge_to_handle_dist_y);

        //Hinge to handle angle in [rad]
        double hinge_to_handle_angle = atan2(door_hinge_to_handle_dist_y,door_hinge_to_handle_dist_x);


        //Desired Rotation angle of door around hinge
        double door_opening_angle_deg = -90.0; //in degree
        double door_opening_angle_rad = door_opening_angle_deg *(M_PI / 180.0); //in degree


        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Set Start Pose
            start_ee_pose[0] = door_hinge_pos_x + door_hinge_to_handle_dist_x;    //X
            start_ee_pose[1] = door_hinge_pos_y + door_hinge_to_handle_dist_y;    //Y
            start_ee_pose[2] = handle_pos_z;   //Z
            start_ee_pose[3] = -1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 0.0;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = hinge_to_handle_vec_length * cos(hinge_to_handle_angle + door_opening_angle_rad);    //X
            ee_goal_pose[1] = hinge_to_handle_vec_length * sin(hinge_to_handle_angle + door_opening_angle_rad);    //Y
            ee_goal_pose[2] = handle_pos_z;     //Z
            ee_goal_pose[3] = -1.57;                      //RotX
            ee_goal_pose[4] = -door_opening_angle_rad;     //RotY
            ee_goal_pose[5] = 0.0;                        //RotZ

            cout<<start_ee_pose[0]<<" "<<start_ee_pose[1]<<" "<<start_ee_pose[2]<<endl;
            cout<<ee_goal_pose[0]<<" "<<ee_goal_pose[1]<<" "<<ee_goal_pose[2]<<endl;
        }

    }
    else if (SCENARIO_NAME == "rack")
    {
        //Enter Environment Borders
        vector<double> env_size_x(2);
        env_size_x[0] = -10.0;
        env_size_x[1] = 10.0;
        vector<double> env_size_y(2);
        env_size_y[0] = -10.0;
        env_size_y[1] = 10.0;
        double env_size_z = 2.0;
        world_builder.insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);


        //Insert Rack Scenario
        vector<double> rack_pos(3);
        rack_pos[0] = -4.0;
        rack_pos[1] = 0.0;
        rack_pos[2] = 0.0;
        vector<double> rack_dim(3);
        rack_dim[0] = 0.26;
        rack_dim[1] = 1.16;
        rack_dim[2] = 2.0;

        //Number of shelves and thickness of the shelves
        int num_shelves = 6;
        double shelf_thickness = 0.02;

        //Load Scene
        world_builder.insertRack(rack_pos,rack_dim,num_shelves,shelf_thickness);

        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Manipulable Object
            //Name, Position and Dimension of object
            string obj_name = "glass";
            vector<double> glass_dim(3);
            glass_dim[0] = 0.088;   //x-width dim
            glass_dim[1] = 0.088;   //y-length dim
            glass_dim[2] = 0.1233;  //z-height dim
            vector<double> glass_pos(3);
            glass_pos[0] = -4.0;   //x-width dim
            glass_pos[1] = 0.0;   //y-length dim
            glass_pos[2] = 1.12;   //z-height dim

            //Insert manipulable object into scene
            world_builder.insertManipulableGlass(obj_name, glass_pos, glass_dim);

            //Set Start Pose
            start_ee_pose[0] = 3.0;     //X
            start_ee_pose[1] = -3.0;    //Y
            start_ee_pose[2] = 1.12;     //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 0.0;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = glass_pos[0] + 0.18;    //X
            ee_goal_pose[1] = glass_pos[1];     //Y
            ee_goal_pose[2] = glass_pos[2];     //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = -1.57;     //RotY
            ee_goal_pose[5] = 0.0;    //RotZ
        }


    }
    else if (SCENARIO_NAME == "walls")
    {
        //Enter wall
		vector<double> wall_pos(3);
		wall_pos[0] = 0.0;
		wall_pos[1] = 0.0;
		wall_pos[2] = 0.0;
		vector<double> wall_dim(3);
		wall_dim[0] = 0.2;
		wall_dim[1] = 6.0;
		wall_dim[2] = 0.6;
		world_builder.insertWall(wall_pos,wall_dim);

        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Set Start Pose
            start_ee_pose[0] = 8.0;     //X
            start_ee_pose[1] = -3.0;    //Y
            start_ee_pose[2] = 0.9;     //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = -8.0;    //X
            ee_goal_pose[1] = 3.5;     //Y
            ee_goal_pose[2] = 0.9;     //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "block")
    {
        //Insert Block Scenario
        vector<double> block_pos(3);
        block_pos[0] = 0.0;
        block_pos[1] = 0.0;
        block_pos[2] = 0.0;
        vector<double> block_dim(3);
        block_dim[0] = 5.0;
        block_dim[1] = 5.0;
        block_dim[2] = 2.0;
        world_builder.insertBlock(block_pos, block_dim);

        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Set Start Pose
            start_ee_pose[0] = 8.0;     //X
            start_ee_pose[1] = -3.0;    //Y
            start_ee_pose[2] = 0.9;     //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = -8.0;    //X
            ee_goal_pose[1] = 3.5;     //Y
            ee_goal_pose[2] = 0.9;     //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }

        if(PLANNING_GROUP == "robotino_robot")
        {
            //Set Start Pose
            start_ee_pose[0] = 8.0;     //X
            start_ee_pose[1] = -3.0;    //Y
            start_ee_pose[2] = 0.6;     //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = -8.0;     //X
            ee_goal_pose[1] = 3.0;      //Y
            ee_goal_pose[2] = 0.6;      //Z
            ee_goal_pose[3] = 1.57;     //RotX
            ee_goal_pose[4] = 0.0;      //RotY
            ee_goal_pose[5] = 1.57;     //RotZ
        }
    }
    else if (SCENARIO_NAME == "glass")
    {
        //Insert Glass Delivery Scenario
       double tunnel_height = 1.4;
       double tunnel_width = 1.7; //Note : omnirob has width of 0.67m
       double ceiling_height = 1.2;
       world_builder.insertGlassDeliveryWorld(tunnel_width, tunnel_height, ceiling_height);

       if(PLANNING_GROUP == "omnirob_lbr_sdh")
       {
           //Start Position of attached_object
           obj_pos_start[0] = 9.0;   //x pos
           obj_pos_start[1] = -4.0;  //y pos
           obj_pos_start[2] = 0.9;   //z pos
           attached_object = world_builder.insertManipulableGlass("glass", obj_pos_start, glass_dim);

           //Goal Position of attached_object
           obj_pos_goal[0] = -8.0;   //x pos
           obj_pos_goal[1] = 3.0;  //y pos
           obj_pos_goal[2] = 0.9;   //z pos
           attached_object_goal = world_builder.insertManipulableGlass("glass_goal", obj_pos_goal, glass_dim);

           //Set Start Pose
           start_ee_pose[0] = obj_pos_start[0];  	   //X
           start_ee_pose[1] = obj_pos_start[1] + 0.35;  //Y
           start_ee_pose[2] = obj_pos_start[2];           //Z
           start_ee_pose[3] = 1.57;    //RotX
           start_ee_pose[4] = 0.0;     //RotY
           start_ee_pose[5] = 0.0;    //RotZ

           //Set Goal Pose
           ee_goal_pose[0] = obj_pos_goal[0];       //X
           ee_goal_pose[1] = obj_pos_goal[1] + 0.35; //Y
           ee_goal_pose[2] = obj_pos_goal[2];          //Z
           ee_goal_pose[3] = 1.57;    //RotX
           ee_goal_pose[4] = 0.0;     //RotY
           ee_goal_pose[5] = 0.0;    //RotZ
       }

       if(PLANNING_GROUP == "robotino_robot")
       {
           //Set Start Pose
           start_ee_pose[0] = 9.0;     //X
           start_ee_pose[1] = -4.0;    //Y
           start_ee_pose[2] = 0.6;     //Z
           start_ee_pose[3] = 1.57;    //RotX
           start_ee_pose[4] = 0.0;     //RotY
           start_ee_pose[5] = 1.57;    //RotZ

           //Set Goal Pose
           ee_goal_pose[0] = -8.0;     //X
           ee_goal_pose[1] = 3.0;      //Y
           ee_goal_pose[2] = 0.6;      //Z
           ee_goal_pose[3] = 1.57;     //RotX
           ee_goal_pose[4] = 0.0;      //RotY
           ee_goal_pose[5] = 1.57;     //RotZ
       }

    }
    else if (SCENARIO_NAME == "maze")
    {
        //Enter Maze
		int num_walls = 4;
		wall_thickness = 0.2;
		wall_length_ratio = 0.5;
		world_builder.insertMaze(num_walls,wall_thickness,wall_length_ratio);

        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 8.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of cart
            obj_pos_goal[0] = -8.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);
		 
            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "parking")
    {
        //Enter Parking Slot
        double narrow_passage_offset = 0.8; //in [m]
        double wall_thickness = 1.0; //in [m]
        world_builder.insertParkingSlot(narrow_passage_offset,wall_thickness);

        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 9.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of cart
            obj_pos_goal[0] = -9.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);


            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] - 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = -1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = -1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "narrow_passage")
    {
        //Insert Narrow Passage
        wall_thickness = 0.2;
    	wall_length_ratio = 0.47;
    	world_builder.insertNarrowPassage(wall_thickness, wall_length_ratio);
    	
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 8.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of attached_object
            obj_pos_goal[0] = -8.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);

            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "three_gates")
    {
        //Insert "Three Passages" scenarion (providing suboptimal paths)
        wall_thickness = 0.2;
    	wall_length_ratio = 0.3;
    	world_builder.insertThreeGatesPassage(wall_thickness, wall_length_ratio);
    	
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 8.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of attached_object
            obj_pos_goal[0] = -8.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);

            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "random_maze")
    {
       //Not implemented yet
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {

        }
    }
    else if (SCENARIO_NAME == "tunnel")
    {
        //Insert Tunnel Scenario
    	double tunnel_height = 1.4;
    	double tunnel_width = 1.1; //Note : omnirob has width of 0.67m
    	world_builder.insertTunnelPassage(tunnel_width, tunnel_height, 0.2);
    	
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 8.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of attached_object
            obj_pos_goal[0] = -8.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);

            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "two_rooms")
    {
        //Insert TwoRooms Office
        wall_thickness = 0.2;
        double width_narrow_passage = 2.1; // in m (omnirob has width of 0.67m)
        world_builder.insertTwoRoomsOffice(wall_thickness,width_narrow_passage);
		
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 8.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of attached_object
            obj_pos_goal[0] = -8.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);

            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else if (SCENARIO_NAME == "corridor")
    {
        //Insert Narrow Corridor Scenario
		double corridor_width = 2.0;
        world_builder.insertNarrowCorridor(corridor_width, 0.2);
		
        if(PLANNING_GROUP == "omnirob_lbr_sdh")
        {
            //Start Position of attached_object
            obj_pos_start[0] = 8.0;   //x pos
            obj_pos_start[1] = -3.5;  //y pos
            obj_pos_start[2] = 0.0;   //z pos
            attached_object = world_builder.insertManipulableCart("cart", obj_pos_start, cart_dim);

            //Goal Position of attached_object
            obj_pos_goal[0] = -8.0;   //x pos
            obj_pos_goal[1] = 3.0;  //y pos
            obj_pos_goal[2] = 0.0;   //z pos
            attached_object_goal = world_builder.insertManipulableCart("cart_goal", obj_pos_goal, cart_dim);

            //Set Start Pose
            start_ee_pose[0] = obj_pos_start[0];  		//X
            start_ee_pose[1] = obj_pos_start[1] + 0.5;  //Y
            start_ee_pose[2] = 0.9;				        //Z
            start_ee_pose[3] = 1.57;    //RotX
            start_ee_pose[4] = 0.0;     //RotY
            start_ee_pose[5] = 1.57;    //RotZ

            //Set Goal Pose
            ee_goal_pose[0] = obj_pos_goal[0];       //X
            ee_goal_pose[1] = obj_pos_goal[1] + 0.5; //Y
            ee_goal_pose[2] = 0.9;                   //Z
            ee_goal_pose[3] = 1.57;    //RotX
            ee_goal_pose[4] = 0.0;     //RotY
            ee_goal_pose[5] = 1.57;    //RotZ
        }
    }
    else
    {
        ROS_ERROR("Unknown Scenario!!!");
        //Shutdown node
        ros::shutdown();
    }
	
	
	//Set path to the file that will store the start and goal config for each scenario
    char* file_path_start_goal_config;
    string folder_path = terminal_configs_path + "/Start_Goal_Configurations/"+ PLANNING_GROUP + "_" + SCENARIO_NAME + "_start_goal_config.txt";
    file_path_start_goal_config = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_start_goal_config);
    file_path_start_goal_config[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_start_goal_config<<endl;
    
    

    // -------------------- Planner and Controller Setup ----------------------------

    //Bidirectional Planner
    birrt_star_motion_planning::BiRRTstarPlanner bi_planner(PLANNING_GROUP);

    //Set planning scene
    bi_planner.setPlanningSceneInfo(world_builder);
    

    //Robot Controller
    //Create Robot Controller
    kuka_motion_controller::RobotController robot_controller("robot_description", PLANNING_GROUP);

    //Set Motion Strategy (0 = all joints weighted equal)
    robot_controller.set_motion_strategy(0);

    
    
    // -------------------- Generate Star and Goal Config ----------------------------
    
    //Convert XYZ euler orientation of start and goal pose to quaternion
    vector<double> quat_start_pose = robot_controller.convertEulertoQuat(start_ee_pose[3],start_ee_pose[4],start_ee_pose[5]);
    vector<double> quat_goal_pose = robot_controller.convertEulertoQuat(ee_goal_pose[3],ee_goal_pose[4],ee_goal_pose[5]);

    //Set up start pose with orientation expressed by quaternion
    vector<double> start_ee_pose_quat_orient (7);
    start_ee_pose_quat_orient[0] = start_ee_pose[0]; //x
    start_ee_pose_quat_orient[1] = start_ee_pose[1]; //y
    start_ee_pose_quat_orient[2] = start_ee_pose[2]; //z
    start_ee_pose_quat_orient[3] = quat_start_pose[0];  //quat_x
    start_ee_pose_quat_orient[4] = quat_start_pose[1];  //quat_y
    start_ee_pose_quat_orient[5] = quat_start_pose[2];  //quat_z
    start_ee_pose_quat_orient[6] = quat_start_pose[3];  //quat_w
    //Set up goal pose with orientation expressed by quaternion
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
    vector<double> ik_sol_ee_goal_pose = bi_planner.findIKSolution(goal_ee_pose_quat_orient, constraint_vec_goal_pose, target_coordinate_dev, SHOW_MOTION);

    //Find configuration for endeffector start pose
    vector<double> ik_sol_ee_start_pose = bi_planner.findIKSolution(start_ee_pose_quat_orient, constraint_vec_start_pose, target_coordinate_dev, ik_sol_ee_goal_pose, SHOW_MOTION);
   

    //Write the start and goal configuration to file
    bi_planner.writeStartGoalConfig(file_path_start_goal_config,ik_sol_ee_start_pose, ik_sol_ee_goal_pose);

    //Shutdown node
    ros::shutdown();

    return 0;
}



