#include <planning_world_builder/planning_world_builder.h>


namespace planning_world
{
  
//Constructor
PlanningWorldBuilder::PlanningWorldBuilder(string robot_desciption_param, string planning_group, string ns_prefix)
{

    //Set up Planning Scene tpoics based on namespace
    string planning_scene_ns = ns_prefix + "planning_scene";
    string endeffector_trajectory_ns = ns_prefix + "endeffector_trajectory";


    //Publisher for collision objects
    m_ps_obj_pub = m_nh.advertise<moveit_msgs::PlanningScene>(planning_scene_ns, 1);


	//Create Robot model
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_desciption_param, planning_scene_ns, endeffector_trajectory_ns, planning_group));
	

	//Get Name of Base Link (required to set reference frame for obstacle visualization)
    m_base_link_name = ns_prefix + m_KDLRobotModel->getBaseLinkName();

    //Init number of walls
    m_num_walls = 0;

    //Environment size
    m_env_dim_x.resize(2);
    m_env_dim_x[0] = 0.0; //env dim in negative x dir
    m_env_dim_x[1] = 0.0; //env dim in positive x dir
    m_env_dim_y.resize(2);
    m_env_dim_y[0] = 0.0; //env dim in negative y dir
    m_env_dim_y[1] = 0.0; //env dim in positive y dir

    //Set default Scene Name
    m_scene_name = "none";

}


//Constructor
PlanningWorldBuilder::PlanningWorldBuilder(boost::shared_ptr<kuka_motion_controller::KDLRobotModel> kdl_robot_model, string ns_prefix)
{

    //Set up Planning Scene tpoics based on namespace
    string planning_scene_ns = ns_prefix + "planning_scene";

    //Publisher for collision objects
    m_ps_obj_pub = m_nh.advertise<moveit_msgs::PlanningScene>(planning_scene_ns, 1);

    //Get Robot model
    m_KDLRobotModel = kdl_robot_model;

    //Get Name of Base Link (required to set reference frame for obstacle visualization)
    m_base_link_name = ns_prefix + m_KDLRobotModel->getBaseLinkName();

    //Init number of walls
    m_num_walls = 0;

    //Environment size
    m_env_dim_x.resize(2);
    m_env_dim_x[0] = 0.0; //env dim in negative x dir
    m_env_dim_x[1] = 0.0; //env dim in positive x dir
    m_env_dim_y.resize(2);
    m_env_dim_y[0] = 0.0; //env dim in negative y dir
    m_env_dim_y[1] = 0.0; //env dim in positive y dir

    //Set default Scene Name
    m_scene_name = "none";

}



//Destructor
PlanningWorldBuilder::~PlanningWorldBuilder()
{
    //Nothing to do yet
}


//Insert Borders confining the environment
void PlanningWorldBuilder::insertEnvironmentBorders(vector<double> size_x, vector<double> size_y, double wall_height, string scene_name)
{
    //Set wall thickness
    double wall_thickness = 0.2;

    //Store environment dimensions  
    m_env_dim_x[0] = size_x[0];
    m_env_dim_x[1] = size_x[1];
    m_env_dim_y[0] = size_y[0];
    m_env_dim_y[1] = size_y[1];

    //Set Scene Name
    m_scene_name = scene_name;

    moveit_msgs::CollisionObject walls;
    walls.header.frame_id = m_base_link_name;
    walls.header.stamp = ros::Time::now();
    walls.id = "environment_borders";

    geometry_msgs::Pose wall_1;
    wall_1.position.x = m_env_dim_x[1] + wall_thickness/2.0;
    wall_1.position.y = m_env_dim_y[1] + m_env_dim_y[0];
    wall_1.position.z = wall_height/2.0;
    wall_1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_1;
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[0] = wall_thickness; //X dim
    primitive_1.dimensions[1] = (m_env_dim_y[1] - m_env_dim_y[0]) + wall_thickness*2.0; //Y dim
    primitive_1.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose wall_2;
    wall_2.position.x = m_env_dim_x[0] - (wall_thickness/2.0);
    wall_2.position.y = m_env_dim_y[1] + m_env_dim_y[0];
    wall_2.position.z = wall_height/2.0;
    wall_2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_2;
    primitive_2.type = primitive_2.BOX;
    primitive_2.dimensions.resize(3);
    primitive_2.dimensions[0] = wall_thickness; //X dim
    primitive_2.dimensions[1] = (m_env_dim_y[1]-m_env_dim_y[0]) + wall_thickness*2.0; //Y dim
    primitive_2.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose wall_3;
    wall_3.position.x = m_env_dim_x[1] + m_env_dim_x[0];
    wall_3.position.y = m_env_dim_y[0] - wall_thickness/2.0;
    wall_3.position.z = wall_height/2.0;
    wall_3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_3;
    primitive_3.type = primitive_3.BOX;
    primitive_3.dimensions.resize(3);
    primitive_3.dimensions[0] = m_env_dim_x[1] - m_env_dim_x[0]; //X dim
    primitive_3.dimensions[1] = wall_thickness; //Y dim
    primitive_3.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose wall_4;
    wall_4.position.x = m_env_dim_x[1] + m_env_dim_x[0];
    wall_4.position.y = m_env_dim_y[1] + wall_thickness/2.0;
    wall_4.position.z = wall_height/2.0;
    wall_4.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_4;
    primitive_4.type = primitive_4.BOX;
    primitive_4.dimensions.resize(3);
    primitive_4.dimensions[0] = m_env_dim_x[1] - m_env_dim_x[0]; //X dim
    primitive_4.dimensions[1] = wall_thickness; //Y dim
    primitive_4.dimensions[2] = wall_height; //Z dim

    //Push back primitives and their poses
    walls.primitives.push_back(primitive_1);
    walls.primitives.push_back(primitive_2);
    walls.primitives.push_back(primitive_3);
    walls.primitives.push_back(primitive_4);
    walls.primitive_poses.push_back(wall_1);
    walls.primitive_poses.push_back(wall_2);
    walls.primitive_poses.push_back(wall_3);
    walls.primitive_poses.push_back(wall_4);
	
	 //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;


    //Enter environment boundaries as obstacles
    planning_scene_msg.world.collision_objects.push_back(walls);
	
	//Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
	
}


//Insert a wall
void PlanningWorldBuilder::insertWall(vector<double> position, vector<double> dimension)
{
    //Set Scene Name
    m_scene_name = "walls_scene";

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2] + dimension[2]/2.0;
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimension[0]; //X dim
    primitive.dimensions[1] = dimension[1]; //5.0//Y dim
    primitive.dimensions[2] = dimension[2]; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive);
    coll_object.primitive_poses.push_back(pose);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}


//Insert a table top scene into the world
void PlanningWorldBuilder::insertTableTop(vector<double> table_pos, vector<double> table_dim)
{

}

//Insert a rack scene into the world
void PlanningWorldBuilder::insertRack(vector<double> rack_pos, vector<double> rack_dim, int num_shelves, double shelves_thickness)
{

    //Set Scene Name
    m_scene_name = "rack_scene";

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    // ----------------------------- Data --------------------------
    //Rack position
    double rack_x_pos = rack_pos[0];
    double rack_y_pos = rack_pos[1];
    double rack_z_pos = rack_pos[2];

    //Rack position
    double rack_x_dim = rack_dim[0];
    double rack_y_dim = rack_dim[1];
    double rack_z_dim = rack_dim[2];

    //Lower vertical spacing for first shelf
    double shelf_lower_spacing= 0.05; //10cm from floor

    //Determine rack depth (used for rack corner pole size)
    double rack_depth = rack_x_dim <= rack_y_dim ? rack_x_dim : rack_y_dim;
    //Determine rack width
    double rack_width = rack_x_dim <= rack_y_dim ? rack_y_dim : rack_x_dim;

    //Size of corner poles (they are square)
    double corner_pole_dim = rack_depth / 5.0;

    //Shelf spacing
    double shelf_spacing = (rack_z_dim - shelf_lower_spacing) / num_shelves;


    // ----------------------------- poses and primitives --------------------------

    // ++++ Corner Poles ++++

    //Corner Pole 1
    geometry_msgs::Pose pose_corner_pole_1;
    pose_corner_pole_1.position.x = rack_x_pos + rack_x_dim / 2.0 - corner_pole_dim / 2.0;
    pose_corner_pole_1.position.y = rack_y_pos + rack_y_dim / 2.0 - corner_pole_dim / 2.0;
    pose_corner_pole_1.position.z = rack_z_dim / 2.0;
    pose_corner_pole_1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_corner_pole_1;
    primitive_corner_pole_1.type = primitive_corner_pole_1.BOX;
    primitive_corner_pole_1.dimensions.resize(3);
    primitive_corner_pole_1.dimensions[0] = corner_pole_dim; //X dim
    primitive_corner_pole_1.dimensions[1] = corner_pole_dim; //Y dim
    primitive_corner_pole_1.dimensions[2] = rack_z_dim; //Z dim

    //Corner Pole 2
    geometry_msgs::Pose pose_corner_pole_2;
    pose_corner_pole_2.position.x = rack_x_pos - rack_x_dim / 2.0 + corner_pole_dim / 2.0;
    pose_corner_pole_2.position.y = rack_y_pos + rack_y_dim / 2.0 - corner_pole_dim / 2.0;
    pose_corner_pole_2.position.z = rack_z_dim / 2.0;
    pose_corner_pole_2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_corner_pole_2;
    primitive_corner_pole_2.type = primitive_corner_pole_2.BOX;
    primitive_corner_pole_2.dimensions.resize(3);
    primitive_corner_pole_2.dimensions[0] = corner_pole_dim; //X dim
    primitive_corner_pole_2.dimensions[1] = corner_pole_dim; //Y dim
    primitive_corner_pole_2.dimensions[2] = rack_z_dim; //Z dim

    //Corner Pole 3
    geometry_msgs::Pose pose_corner_pole_3;
    pose_corner_pole_3.position.x = rack_x_pos + rack_x_dim / 2.0 - corner_pole_dim / 2.0;
    pose_corner_pole_3.position.y = rack_y_pos - rack_y_dim / 2.0 + corner_pole_dim / 2.0;
    pose_corner_pole_3.position.z = rack_z_dim / 2.0;
    pose_corner_pole_3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_corner_pole_3;
    primitive_corner_pole_3.type = primitive_corner_pole_3.BOX;
    primitive_corner_pole_3.dimensions.resize(3);
    primitive_corner_pole_3.dimensions[0] = corner_pole_dim; //X dim
    primitive_corner_pole_3.dimensions[1] = corner_pole_dim; //Y dim
    primitive_corner_pole_3.dimensions[2] = rack_z_dim; //Z dim

    //Corner Pole 4
    geometry_msgs::Pose pose_corner_pole_4;
    pose_corner_pole_4.position.x = rack_x_pos - rack_x_dim / 2.0 + corner_pole_dim / 2.0;
    pose_corner_pole_4.position.y = rack_y_pos - rack_y_dim / 2.0 + corner_pole_dim / 2.0;
    pose_corner_pole_4.position.z = rack_z_dim / 2.0;
    pose_corner_pole_4.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_corner_pole_4;
    primitive_corner_pole_4.type = primitive_corner_pole_4.BOX;
    primitive_corner_pole_4.dimensions.resize(3);
    primitive_corner_pole_4.dimensions[0] = corner_pole_dim; //X dim
    primitive_corner_pole_4.dimensions[1] = corner_pole_dim; //Y dim
    primitive_corner_pole_4.dimensions[2] = rack_z_dim; //Z dim

    // ++++ Top Shelf ++++

    geometry_msgs::Pose pose_top_shelf;
    pose_top_shelf.position.x = rack_x_pos;
    pose_top_shelf.position.y = rack_y_pos;
    pose_top_shelf.position.z = rack_z_dim + shelves_thickness / 2.0;
    pose_top_shelf.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_top_shelf;
    primitive_top_shelf.type = primitive_top_shelf.BOX;
    primitive_top_shelf.dimensions.resize(3);
    primitive_top_shelf.dimensions[0] = rack_x_dim; //X dim
    primitive_top_shelf.dimensions[1] = rack_y_dim; //Y dim
    primitive_top_shelf.dimensions[2] = shelves_thickness; //Z dim


    //Add primitive and pose
    coll_object.primitives.push_back(primitive_corner_pole_1);
    coll_object.primitive_poses.push_back(pose_corner_pole_1);
    coll_object.primitives.push_back(primitive_corner_pole_2);
    coll_object.primitive_poses.push_back(pose_corner_pole_2);
    coll_object.primitives.push_back(primitive_corner_pole_3);
    coll_object.primitive_poses.push_back(pose_corner_pole_3);
    coll_object.primitives.push_back(primitive_corner_pole_4);
    coll_object.primitive_poses.push_back(pose_corner_pole_4);
    coll_object.primitives.push_back(primitive_top_shelf);
    coll_object.primitive_poses.push_back(pose_top_shelf);


    // ++++ Intermediate Shelves ++++

    for (int i = 0; i < num_shelves ; i++)
    {
        geometry_msgs::Pose pose_shelf;
        pose_shelf.position.x = rack_x_pos;
        pose_shelf.position.y = rack_y_pos;
        pose_shelf.position.z = shelf_lower_spacing + i*shelf_spacing + shelves_thickness / 2.0;
        pose_shelf.orientation.w = 1.0;
        shape_msgs::SolidPrimitive primitive_shelf;
        primitive_shelf.type = primitive_shelf.BOX;
        primitive_shelf.dimensions.resize(3);
        primitive_shelf.dimensions[0] = rack_x_dim <= rack_y_dim ? rack_x_dim : (rack_x_dim - 2* corner_pole_dim); //X dim
        primitive_shelf.dimensions[1] = rack_x_dim <= rack_y_dim ? (rack_y_dim - 2* corner_pole_dim) : rack_y_dim; //Y dim
        primitive_shelf.dimensions[2] = shelves_thickness; //Z dim

        //Add primitive and pose
        coll_object.primitives.push_back(primitive_shelf);
        coll_object.primitive_poses.push_back(pose_shelf);
    }


    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}


//Insert a block in the center of the world
void PlanningWorldBuilder::insertBlock(vector<double> position, vector<double> dimension)
{
    //Set Scene Name
    m_scene_name = "block_scene";

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    //Gap width between blocks
    double gap_width_x = 0.3* dimension[0];
    double gap_width_y = 0.3* dimension[1];

    //Block size
    double block_size_x = dimension[0]/2.0 - gap_width_x/2.0;
    double block_size_y = dimension[1]/2.0 - gap_width_y/2.0;


    //Block 1 (top left)
    geometry_msgs::Pose pose_block_1;
    pose_block_1.position.x = position[0] - gap_width_x/2.0 - block_size_x/2.0;
    pose_block_1.position.y = position[1] + gap_width_y/2.0 + block_size_y/2.0;
    pose_block_1.position.z = position[2] + dimension[2]/2.0;
    pose_block_1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_block_1;
    primitive_block_1.type = primitive_block_1.BOX;
    primitive_block_1.dimensions.resize(3);
    primitive_block_1.dimensions[0] = block_size_x; //X dim
    primitive_block_1.dimensions[1] = block_size_y; //Y dim
    primitive_block_1.dimensions[2] = dimension[2]; //Z dim

    //Block 2 (top right)
    geometry_msgs::Pose pose_block_2;
    pose_block_2.position.x = position[0] + gap_width_x/2.0 + block_size_x/2.0;
    pose_block_2.position.y = position[1] + gap_width_y/2.0 + block_size_y/2.0;;
    pose_block_2.position.z = position[2] + dimension[2]/2.0;
    pose_block_2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_block_2;
    primitive_block_2.type = primitive_block_2.BOX;
    primitive_block_2.dimensions.resize(3);
    primitive_block_2.dimensions[0] = block_size_x; //X dim
    primitive_block_2.dimensions[1] = block_size_y; //Y dim
    primitive_block_2.dimensions[2] = dimension[2]; //Z dim

    //Block 3 (bottom left)
    geometry_msgs::Pose pose_block_3;
    pose_block_3.position.x = position[0] - gap_width_x/2.0 - block_size_x/2.0;
    pose_block_3.position.y = position[1] - gap_width_y/2.0 - block_size_y/2.0;;
    pose_block_3.position.z = position[2] + dimension[2]/2.0;
    pose_block_3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_block_3;
    primitive_block_3.type = primitive_block_3.BOX;
    primitive_block_3.dimensions.resize(3);
    primitive_block_3.dimensions[0] = block_size_x; //X dim
    primitive_block_3.dimensions[1] = block_size_y; //Y dim
    primitive_block_3.dimensions[2] = dimension[2]; //Z dim

    //Block 4 (bottom right)
    geometry_msgs::Pose pose_block_4;
    pose_block_4.position.x = position[0] + gap_width_x/2.0 + block_size_x/2.0;
    pose_block_4.position.y = position[1] - gap_width_y/2.0 - block_size_y/2.0;;
    pose_block_4.position.z = position[2] + dimension[2]/2.0;
    pose_block_4.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_block_4;
    primitive_block_4.type = primitive_block_4.BOX;
    primitive_block_4.dimensions.resize(3);
    primitive_block_4.dimensions[0] = block_size_x; //X dim
    primitive_block_4.dimensions[1] = block_size_y; //Y dim
    primitive_block_4.dimensions[2] = dimension[2]; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive_block_1);
    coll_object.primitive_poses.push_back(pose_block_1);
    coll_object.primitives.push_back(primitive_block_2);
    coll_object.primitive_poses.push_back(pose_block_2);
    coll_object.primitives.push_back(primitive_block_3);
    coll_object.primitive_poses.push_back(pose_block_3);
    coll_object.primitives.push_back(primitive_block_4);
    coll_object.primitive_poses.push_back(pose_block_4);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
}


//Insert an easy maze
void PlanningWorldBuilder::insertMaze(int num_walls, double wall_thickness, double wall_length_ratio)
{
    //Set Scene Name
    m_scene_name = "maze_scene";

    //Wall width and length
    double wall_width = wall_thickness;
    double wall_length = (m_env_dim_y[1]-m_env_dim_y[0]) * wall_length_ratio;
    double wall_height = 0.6;

    //Width of free space passages (in x-direction)
    double width_free_space_elements = ((m_env_dim_x[1]-m_env_dim_x[0]) - (num_walls * wall_width)) / (num_walls+1);

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    for (int wall_num = 1 ; wall_num <= num_walls ; wall_num++)
    {
        geometry_msgs::Pose pose;
        shape_msgs::SolidPrimitive primitive;

        if(wall_num % 2 != 0)
        {
            //Pose
            pose.position.x = m_env_dim_x[0] + wall_num * width_free_space_elements + wall_num * wall_width - wall_width/2.0;
            pose.position.y = m_env_dim_y[1] - wall_length/2.0;
            pose.position.z = wall_height/2.0;
            pose.orientation.w = 1.0;
            //Primitive
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = wall_width; //X dim
            primitive.dimensions[1] = wall_length; //Y dim
            primitive.dimensions[2] = wall_height; //Z dim
        }
        else
        {
            //Pose
            pose.position.x = m_env_dim_x[0] + wall_num * width_free_space_elements + wall_num * wall_width - wall_width/2.0;
            pose.position.y = m_env_dim_y[0] + wall_length/2.0;
            pose.position.z = wall_height/2.0;
            pose.orientation.w = 1.0;
            //Primitive
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = wall_width; //X dim
            primitive.dimensions[1] = wall_length; //Y dim
            primitive.dimensions[2] = wall_height; //Z dim
        }

        //Add primitive and pose
        coll_object.primitives.push_back(primitive);
        coll_object.primitive_poses.push_back(pose);

    }

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();


}



//Insert narrow passage
void PlanningWorldBuilder::insertNarrowPassage(double wall_thickness, double wall_length_ratio)
{
    //Set Scene Name
    m_scene_name = "narrow_passage_scene";

    //Wall width and length
    double wall_width = wall_thickness;
    double wall_length;
    if(0.5 < wall_length_ratio)
    {
        wall_length = 0.5 * (m_env_dim_y[1]-m_env_dim_y[0]);
        ROS_INFO("wall_length_ratio reset to 0.5!");
    }
    else
        wall_length = wall_length_ratio * (m_env_dim_y[1]-m_env_dim_y[0]);
    double wall_height = 0.6;

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = m_env_dim_y[1] - wall_length/2.0;
    pose.position.z = wall_height/2.0;
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = wall_thickness; //X dim
    primitive.dimensions[1] = wall_length; //Y dim
    primitive.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose pose2;
    pose2.position.x = 0.0;
    pose2.position.y = m_env_dim_y[0] + wall_length/2.0;
    pose2.position.z = wall_height/2.0;
    pose2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = wall_thickness; //X dim
    primitive2.dimensions[1] = wall_length; //Y dim
    primitive2.dimensions[2] = wall_height; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive);
    coll_object.primitives.push_back(primitive2);
    coll_object.primitive_poses.push_back(pose);
    coll_object.primitive_poses.push_back(pose2);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
}


//Insert three passages scenario
void PlanningWorldBuilder::insertThreeGatesPassage(double wall_thickness, double wall_length_ratio)
{
    //Set Scene Name
    m_scene_name = "three_gates_scene";

    //Wall width and length
    double wall_width = wall_thickness;
    double wall_length;
    if(0.5 < wall_length_ratio)
    {
        wall_length = 0.5 * (m_env_dim_y[1]-m_env_dim_y[0]);
        ROS_INFO("wall_length_ratio reset to 0.5!");
    }
    else
        wall_length = wall_length_ratio * (m_env_dim_y[1]-m_env_dim_y[0]);
    double wall_height = 0.6;
    //Free Space between each wall (in y-direction)
    double free_space_width = ((m_env_dim_y[1]-m_env_dim_y[0]) - 2*wall_length) / 3.0;

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = m_env_dim_y[1] - free_space_width - wall_length/2.0;
    pose.position.z = wall_height/2.0;
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = wall_thickness; //X dim
    primitive.dimensions[1] = wall_length; //Y dim
    primitive.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose pose2;
    pose2.position.x = 0.0;
    pose2.position.y = m_env_dim_y[0] + free_space_width + wall_length/2.0;
    pose2.position.z = wall_height/2.0;
    pose2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = wall_thickness; //X dim
    primitive2.dimensions[1] = wall_length; //Y dim
    primitive2.dimensions[2] = wall_height; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive);
    coll_object.primitives.push_back(primitive2);
    coll_object.primitive_poses.push_back(pose);
    coll_object.primitive_poses.push_back(pose2);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
}

//Insert a random maze
void PlanningWorldBuilder::insertRandomMaze(int num_walls)
{

    //Set Scene Name
    m_scene_name = "random_maze_scene";

}


//Insert Tunnel Scenario
void PlanningWorldBuilder::insertTunnelPassage(double tunnel_width, double tunnel_height, double wall_length_ratio)
{
    //Set Scene Name
    m_scene_name = "tunnel_scene";

    //Width of narrow passage
    double width_tunnel_passage = tunnel_width; // in m (omnirob has width of 0.67m)
    //Height of narrow passage
    double height_tunnel_passage = tunnel_height; // in m

    //Wall length
    double wall_length = wall_length_ratio * (m_env_dim_x[1]-m_env_dim_x[0]);
    double wall_height = height_tunnel_passage + 0.4; //in [m]

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    //Floor Beam
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = height_tunnel_passage + ((wall_height - height_tunnel_passage)/2.0);
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = wall_length; //X dim
    primitive.dimensions[1] = width_tunnel_passage; //Y dim
    primitive.dimensions[2] = wall_height - height_tunnel_passage; //Z dim

    //Top Wall
    geometry_msgs::Pose pose2;
    pose2.position.x = 0.0;
    double block_length = m_env_dim_y[1] - width_tunnel_passage/2.0;
    pose2.position.y = width_tunnel_passage/2.0 + block_length/2.0;
    pose2.position.z = wall_height/2.0;
    pose2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = wall_length; //X dim
    primitive2.dimensions[1] = block_length; //Y dim
    primitive2.dimensions[2] = wall_height; //Z dim

    //Bottom Wall
    geometry_msgs::Pose pose3;
    pose3.position.x = 0.0;
    pose3.position.y = -width_tunnel_passage/2.0 - block_length/2.0;
    pose3.position.z = wall_height/2.0;
    pose3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = wall_length; //X dim
    primitive3.dimensions[1] = block_length; //Y dim
    primitive3.dimensions[2] = wall_height; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive);
    coll_object.primitives.push_back(primitive2);
    coll_object.primitives.push_back(primitive3);
    coll_object.primitive_poses.push_back(pose);
    coll_object.primitive_poses.push_back(pose2);
    coll_object.primitive_poses.push_back(pose3);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();


}


//Insert TwoRooms Office World
void PlanningWorldBuilder::insertTwoRoomsOffice(double wall_thickness, double width_narrow_passage)
{
    //Set Scene Name
    m_scene_name = "two_rooms_scene";

    //Wall height
    double wall_height = 0.6;

    //Width of narrow passage
    //double width_narrow_passage = 1.1; // in m (omnirob has width of 0.67m)

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    //Wall height
    double block_size_x = (m_env_dim_x[1]-m_env_dim_x[0])/2.0 ;
    double block_size_y = m_env_dim_y[1] - 0.1;

    //Block Object
    geometry_msgs::Pose pose;
    pose.position.x = m_env_dim_x[1] - block_size_x/2.0;
    pose.position.y = m_env_dim_y[1] - block_size_y/2.0;
    pose.position.z = wall_height/2.0;
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = m_env_dim_x[1]; //X dim
    primitive.dimensions[1] = m_env_dim_y[1]; //Y dim
    primitive.dimensions[2] = wall_height; //Z dim

    //Narrow Passage
    geometry_msgs::Pose pose2;
    pose2.position.x = wall_thickness/2.0;
    double block_length = (m_env_dim_y[1] - width_narrow_passage)/2.0;
    pose2.position.y = -block_length/2.0;
    pose2.position.z = wall_height/2.0;
    pose2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = wall_thickness; //X dim
    primitive2.dimensions[1] = block_length; //Y dim
    primitive2.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose pose3;
    pose3.position.x = wall_thickness/2.0;
    pose3.position.y = block_length/2.0 + m_env_dim_y[0];
    pose3.position.z = wall_height/2.0;
    pose3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = wall_thickness; //X dim
    primitive3.dimensions[1] = block_length; //Y dim
    primitive3.dimensions[2] = wall_height; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive);
    coll_object.primitives.push_back(primitive2);
    coll_object.primitives.push_back(primitive3);
    coll_object.primitive_poses.push_back(pose);
    coll_object.primitive_poses.push_back(pose2);
    coll_object.primitive_poses.push_back(pose3);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}


//Insert Narrow Corridor Scenario
void PlanningWorldBuilder::insertNarrowCorridor(double corridor_width, double wall_length_ratio)
{
    //Set Scene Name
    m_scene_name = "corridor_scene";

    //Wall length
    double wall_length = wall_length_ratio * (m_env_dim_x[1]-m_env_dim_x[0]);
    double wall_height = 0.6;

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    //Length of a block
    double off_center_y = 0.6; //Offset from global frame in y-direction
    double total_block_length = (m_env_dim_y[1] - 2*corridor_width) - 2*off_center_y;
    double single_block_length = total_block_length/2.0;

    //Block 1
    geometry_msgs::Pose pose_block_1;
    pose_block_1.position.x = 0.0;
    pose_block_1.position.y = off_center_y + single_block_length/2.0;
    pose_block_1.position.z = wall_height/2.0;
    pose_block_1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_block_1;
    primitive_block_1.type = primitive_block_1.BOX;
    primitive_block_1.dimensions.resize(3);
    primitive_block_1.dimensions[0] = wall_length; //X dim
    primitive_block_1.dimensions[1] = single_block_length; //Y dim
    primitive_block_1.dimensions[2] = wall_height; //Z dim

    //Block 2
    geometry_msgs::Pose pose_block_2;
    pose_block_2.position.x = 0.0;
    pose_block_2.position.y = -off_center_y - single_block_length/2.0;
    pose_block_2.position.z = wall_height/2.0;
    pose_block_2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_block_2;
    primitive_block_2.type = primitive_block_2.BOX;
    primitive_block_2.dimensions.resize(3);
    primitive_block_2.dimensions[0] = wall_length; //X dim
    primitive_block_2.dimensions[1] = single_block_length; //Y dim
    primitive_block_2.dimensions[2] = wall_height; //Z dim


//    //Block 1
//    geometry_msgs::Pose pose_block_1;
//    pose_block_1.position.x = 0.0;
//    pose_block_1.position.y = 0.0;
//    pose_block_1.position.z = wall_height/2.0;
//    pose_block_1.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive_block_1;
//    primitive_block_1.type = primitive_block_1.BOX;
//    primitive_block_1.dimensions.resize(3);
//    primitive_block_1.dimensions[0] = wall_length; //X dim
//    primitive_block_1.dimensions[1] = m_env_dim_y - 2*corridor_width; //Y dim
//    primitive_block_1.dimensions[2] = wall_height; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive_block_1);
    coll_object.primitive_poses.push_back(pose_block_1);
    coll_object.primitives.push_back(primitive_block_2);
    coll_object.primitive_poses.push_back(pose_block_2);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
}


//Insert Glass Delivery Scenario
void PlanningWorldBuilder::insertGlassDeliveryWorld(double tunnel_width,double tunnel_height, double ceiling_height)
{

    //Set Scene Name
    m_scene_name = "glass_scene";

    //Width of narrow passage
    double width_tunnel_passage = tunnel_width; // in m (omnirob has width of 0.67m)
    //Height of narrow passage
    double height_tunnel_passage = tunnel_height; // in m
    //Height of narrow passage
    double height_ceiling_passage = ceiling_height; // in m

    //Wall thickness
    double wall_thickness = 0.5;
    double wall_height = height_tunnel_passage + 0.4; //in [m]

    //Walls offset in x direction
    double obstacle_offset_x_dir = 2.5;

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;


    double block_length = ((m_env_dim_y[1]-m_env_dim_y[0]) - (3*width_tunnel_passage))/4.0;

    // ---------------------- Tunnel Passages ------------------------
    //Floor Beams
    geometry_msgs::Pose beam1;
    beam1.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    beam1.position.y = 0.0;
    beam1.position.z = height_tunnel_passage + ((wall_height - height_tunnel_passage)/2.0);
    beam1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_beam1;
    primitive_beam1.type = primitive_beam1.BOX;
    primitive_beam1.dimensions.resize(3);
    primitive_beam1.dimensions[0] = wall_thickness; //X dim
    primitive_beam1.dimensions[1] = width_tunnel_passage; //Y dim
    primitive_beam1.dimensions[2] = wall_height - height_tunnel_passage; //Z dim


    geometry_msgs::Pose beam2;
    beam2.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    beam2.position.y = block_length + width_tunnel_passage;
    beam2.position.z = height_tunnel_passage + ((wall_height - height_tunnel_passage)/2.0);
    beam2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_beam2;
    primitive_beam2.type = primitive_beam2.BOX;
    primitive_beam2.dimensions.resize(3);
    primitive_beam2.dimensions[0] = wall_thickness; //X dim
    primitive_beam2.dimensions[1] = width_tunnel_passage; //Y dim
    primitive_beam2.dimensions[2] = wall_height - height_tunnel_passage; //Z dim


    geometry_msgs::Pose beam3;
    beam3.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    beam3.position.y = -block_length - width_tunnel_passage;
    beam3.position.z = height_tunnel_passage + ((wall_height - height_tunnel_passage)/2.0);
    beam3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_beam3;
    primitive_beam3.type = primitive_beam3.BOX;
    primitive_beam3.dimensions.resize(3);
    primitive_beam3.dimensions[0] = wall_thickness; //X dim
    primitive_beam3.dimensions[1] = width_tunnel_passage; //Y dim
    primitive_beam3.dimensions[2] = wall_height - height_tunnel_passage; //Z dim

    //Top Wall
    geometry_msgs::Pose wall_1;
    wall_1.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    wall_1.position.y = 1.5 * block_length + 1.5 * width_tunnel_passage;
    wall_1.position.z = wall_height/2.0;
    wall_1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_wall_1;
    primitive_wall_1.type = primitive_wall_1.BOX;
    primitive_wall_1.dimensions.resize(3);
    primitive_wall_1.dimensions[0] = wall_thickness; //X dim
    primitive_wall_1.dimensions[1] = block_length; //Y dim
    primitive_wall_1.dimensions[2] = wall_height; //Z dim

    //Center Top Wall
    geometry_msgs::Pose wall_2;
    wall_2.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    wall_2.position.y = 0.5 * block_length + 0.5 * width_tunnel_passage;
    wall_2.position.z = wall_height/2.0;
    wall_2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_wall_2;
    primitive_wall_2.type = primitive_wall_2.BOX;
    primitive_wall_2.dimensions.resize(3);
    primitive_wall_2.dimensions[0] = wall_thickness; //X dim
    primitive_wall_2.dimensions[1] = block_length; //Y dim
    primitive_wall_2.dimensions[2] = wall_height; //Z dim

    //Center Lower Wall
    geometry_msgs::Pose wall_3;
    wall_3.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    wall_3.position.y = -0.5 * block_length - 0.5 * width_tunnel_passage;
    wall_3.position.z = wall_height/2.0;
    wall_3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_wall_3;
    primitive_wall_3.type = primitive_wall_3.BOX;
    primitive_wall_3.dimensions.resize(3);
    primitive_wall_3.dimensions[0] = wall_thickness; //X dim
    primitive_wall_3.dimensions[1] = block_length; //Y dim
    primitive_wall_3.dimensions[2] = wall_height; //Z dim

    //Bottom Wall
    geometry_msgs::Pose wall_4;
    wall_4.position.x = -(m_env_dim_x[1]-m_env_dim_x[0])/5 + obstacle_offset_x_dir;
    wall_4.position.y = -1.5 * block_length - 1.5 * width_tunnel_passage;
    wall_4.position.z = wall_height/2.0;
    wall_4.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_wall_4;
    primitive_wall_4.type = primitive_wall_4.BOX;
    primitive_wall_4.dimensions.resize(3);
    primitive_wall_4.dimensions[0] = wall_thickness; //X dim
    primitive_wall_4.dimensions[1] = block_length; //Y dim
    primitive_wall_4.dimensions[2] = wall_height; //Z dim


    // ---------------------- Lower Ceiling ------------------------

    //Ceiling
    geometry_msgs::Pose ceiling;
    ceiling.position.x = 0.0 + obstacle_offset_x_dir;
    ceiling.position.y = 0.0;
    ceiling.position.z = height_ceiling_passage + ((wall_height - height_ceiling_passage)/2.0);
    ceiling.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_ceiling;
    primitive_ceiling.type = primitive_ceiling.BOX;
    primitive_ceiling.dimensions.resize(3);
    primitive_ceiling.dimensions[0] = wall_thickness; //X dim
    primitive_ceiling.dimensions[1] = (m_env_dim_y[1]-m_env_dim_y[0]); //Y dim
    primitive_ceiling.dimensions[2] = wall_height - height_ceiling_passage; //Z dim


    // ---------------------- Narrow Passage ------------------------

//    geometry_msgs::Pose pose5;
//    pose5.position.x = (m_env_dim_x[1]-m_env_dim_x[0])/5+ obstacle_offset_x_dir;
//    pose5.position.y = m_env_dim_y/2.0 - (m_env_dim_y/2.0 - width_tunnel_passage/2.0)/2.0;
//    pose5.position.z = wall_height/2.0;
//    pose5.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive5;
//    primitive5.type = primitive.BOX;
//    primitive5.dimensions.resize(3);
//    primitive5.dimensions[0] = wall_thickness; //X dim
//    primitive5.dimensions[1] = m_env_dim_y/2.0 - width_tunnel_passage/2.0; //Y dim
//    primitive5.dimensions[2] = wall_height; //Z dim

//    geometry_msgs::Pose pose6;
//    pose6.position.x = (m_env_dim_x[1]-m_env_dim_x[0])/5+ obstacle_offset_x_dir;
//    pose6.position.y = -m_env_dim_y/2.0 + (m_env_dim_y/2.0 - width_tunnel_passage/2.0)/2.0;
//    pose6.position.z = wall_height/2.0;
//    pose6.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive6;
//    primitive6.type = primitive6.BOX;
//    primitive6.dimensions.resize(3);
//    primitive6.dimensions[0] = wall_thickness; //X dim
//    primitive6.dimensions[1] = m_env_dim_y/2.0 - width_tunnel_passage/2.0; //Y dim
//    primitive6.dimensions[2] = wall_height; //Z dim



//    //Top Wall
//    geometry_msgs::Pose wall_5;
//    wall_5.position.x = (m_env_dim_x[1]-m_env_dim_x[0])/5+ obstacle_offset_x_dir;
//    wall_5.position.y = 1.5 * block_length + 1.5 * width_tunnel_passage;
//    wall_5.position.z = wall_height/2.0;
//    wall_5.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive_wall_5;
//    primitive_wall_5.type = primitive_wall_5.BOX;
//    primitive_wall_5.dimensions.resize(3);
//    primitive_wall_5.dimensions[0] = wall_thickness; //X dim
//    primitive_wall_5.dimensions[1] = block_length; //Y dim
//    primitive_wall_5.dimensions[2] = wall_height; //Z dim

//    //Center Top Wall
//    geometry_msgs::Pose wall_6;
//    wall_6.position.x = (m_env_dim_x[1]-m_env_dim_x[0])/5+ obstacle_offset_x_dir;
//    wall_6.position.y = 0.5 * block_length + 0.5 * width_tunnel_passage;
//    wall_6.position.z = wall_height/2.0;
//    wall_6.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive_wall_6;
//    primitive_wall_6.type = primitive_wall_6.BOX;
//    primitive_wall_6.dimensions.resize(3);
//    primitive_wall_6.dimensions[0] = wall_thickness; //X dim
//    primitive_wall_6.dimensions[1] = block_length; //Y dim
//    primitive_wall_6.dimensions[2] = wall_height; //Z dim

//    //Center Lower Wall
//    geometry_msgs::Pose wall_7;
//    wall_7.position.x = (m_env_dim_x[1]-m_env_dim_x[0])/5+ obstacle_offset_x_dir;
//    wall_7.position.y = -0.5 * block_length - 0.5 * width_tunnel_passage;
//    wall_7.position.z = wall_height/2.0;
//    wall_7.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive_wall_7;
//    primitive_wall_7.type = primitive_wall_7.BOX;
//    primitive_wall_7.dimensions.resize(3);
//    primitive_wall_7.dimensions[0] = wall_thickness; //X dim
//    primitive_wall_7.dimensions[1] = block_length; //Y dim
//    primitive_wall_7.dimensions[2] = wall_height; //Z dim

//    //Bottom Wall
//    geometry_msgs::Pose wall_8;
//    wall_8.position.x = (m_env_dim_x[1]-m_env_dim_x[0])/5+ obstacle_offset_x_dir;
//    wall_8.position.y = -1.5 * block_length - 1.5 * width_tunnel_passage;
//    wall_8.position.z = wall_height/2.0;
//    wall_8.orientation.w = 1.0;
//    shape_msgs::SolidPrimitive primitive_wall_8;
//    primitive_wall_8.type = primitive_wall_8.BOX;
//    primitive_wall_8.dimensions.resize(3);
//    primitive_wall_8.dimensions[0] = wall_thickness; //X dim
//    primitive_wall_8.dimensions[1] = block_length; //Y dim
//    primitive_wall_8.dimensions[2] = wall_height; //Z dim


    // ---------------------- Start pose Obstacles ------------------------

    double wall_length_x = 3.5;

    geometry_msgs::Pose pose1;
    pose1.position.x = m_env_dim_x[1] - wall_length_x/2.0;
    pose1.position.y = -1.4 - wall_thickness/2.0;
    pose1.position.z = wall_height/2.0;
    pose1.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_1;
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[0] = wall_length_x; //X dim
    primitive_1.dimensions[1] = wall_thickness; //Y dim
    primitive_1.dimensions[2] = wall_height; //Z dim

    double wall_length_y = 6.5;

    geometry_msgs::Pose pose2;
    pose2.position.x = m_env_dim_x[1] - 2.0 - wall_thickness/2.0;
    pose2.position.y = m_env_dim_y[0]  + wall_length_y/2.0;
    pose2.position.z = 0.5;
    pose2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_2;
    primitive_2.type = primitive_2.BOX;
    primitive_2.dimensions.resize(3);
    primitive_2.dimensions[0] = wall_thickness; //X dim
    primitive_2.dimensions[1] = wall_length_y; //Y dim
    primitive_2.dimensions[2] = 1.0; //Z dim

    geometry_msgs::Pose pose3;
    pose3.position.x = m_env_dim_x[1] - 1.0;
    pose3.position.y = m_env_dim_y[0]  + (wall_length_y-1.0)/2.0;
    pose3.position.z = 0.5;
    pose3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive_3;
    primitive_3.type = primitive_3.BOX;
    primitive_3.dimensions.resize(3);
    primitive_3.dimensions[0] = 2.0; //X dim
    primitive_3.dimensions[1] = wall_length_y-1.0; //Y dim
    primitive_3.dimensions[2] = 1.0; //Z dim


    //Add primitive and pose
    coll_object.primitives.push_back(primitive_beam1);
    coll_object.primitives.push_back(primitive_beam2);
    coll_object.primitives.push_back(primitive_beam3);
    coll_object.primitives.push_back(primitive_wall_1);
    coll_object.primitives.push_back(primitive_wall_2);
    coll_object.primitives.push_back(primitive_wall_3);
    coll_object.primitives.push_back(primitive_wall_4);
//    coll_object.primitives.push_back(primitive_wall_5);
//    coll_object.primitives.push_back(primitive_wall_6);
//    coll_object.primitives.push_back(primitive_wall_7);
//    coll_object.primitives.push_back(primitive_wall_8);
    coll_object.primitives.push_back(primitive_1);
    coll_object.primitives.push_back(primitive_2);
    coll_object.primitives.push_back(primitive_3);
    coll_object.primitives.push_back(primitive_ceiling);

    coll_object.primitive_poses.push_back(beam1);
    coll_object.primitive_poses.push_back(beam2);
    coll_object.primitive_poses.push_back(beam3);
    coll_object.primitive_poses.push_back(wall_1);
    coll_object.primitive_poses.push_back(wall_2);
    coll_object.primitive_poses.push_back(wall_3);
    coll_object.primitive_poses.push_back(wall_4);
//    coll_object.primitive_poses.push_back(wall_5);
//    coll_object.primitive_poses.push_back(wall_6);
//    coll_object.primitive_poses.push_back(wall_7);
//    coll_object.primitive_poses.push_back(wall_8);
    coll_object.primitive_poses.push_back(pose1);
    coll_object.primitive_poses.push_back(pose2);
    coll_object.primitive_poses.push_back(pose3);
    coll_object.primitive_poses.push_back(ceiling);





    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}


void PlanningWorldBuilder::insertParkingSlot(double narrow_passage_offset, double wall_thickness)
{
    //Set Scene Name
    m_scene_name = "parking_scene";

    //Wall width and length
    double wall_width = wall_thickness;
    double wall_height = 0.6;
    double wall_length = (m_env_dim_y[1] - m_env_dim_y[0])/4.0;

    //Free Space between each wall (in y-direction)
    //double free_space_width = (m_env_dim_y - 2*wall_length) / 3.0;

    //Setup
    moveit_msgs::CollisionObject coll_object;
    coll_object.header.frame_id = m_base_link_name;
    coll_object.header.stamp = ros::Time::now();
    stringstream ss;
    ss << m_num_walls;
    string str_num_walls = ss.str();
    coll_object.id = "environment_obstacle_" + str_num_walls;
    m_num_walls++;
    coll_object.operation = moveit_msgs::CollisionObject::ADD;

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = narrow_passage_offset + wall_length/2.0;
    pose.position.z = wall_height/2.0;
    pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = wall_thickness; //X dim
    primitive.dimensions[1] = wall_length; //Y dim
    primitive.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose pose2;
    pose2.position.x = 0.0;
    pose2.position.y = -narrow_passage_offset - wall_length/2.0;
    pose2.position.z = wall_height/2.0;
    pose2.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = wall_thickness; //X dim
    primitive2.dimensions[1] = wall_length; //Y dim
    primitive2.dimensions[2] = wall_height; //Z dim


    double wall_thickness_parking_slot = 0.5;
    double wall_length_parking_slot = 2.0;

    geometry_msgs::Pose pose3;
    pose3.position.x = m_env_dim_x[1] - (wall_length_parking_slot + wall_thickness_parking_slot)/2.0;
    pose3.position.y = -4.0 - wall_thickness_parking_slot/2.0;
    pose3.position.z = wall_height/2.0;
    pose3.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = wall_length_parking_slot + wall_thickness_parking_slot; //X dim
    primitive3.dimensions[1] = wall_thickness_parking_slot; //Y dim
    primitive3.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose pose4;
    pose4.position.x = m_env_dim_x[1] - wall_length_parking_slot - wall_thickness_parking_slot/2.0;
    pose4.position.y = -4.0 + wall_length_parking_slot/2.0;
    pose4.position.z = wall_height/2.0;
    pose4.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive4;
    primitive4.type = primitive4.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = wall_thickness_parking_slot; //X dim
    primitive4.dimensions[1] = wall_length_parking_slot; //Y dim
    primitive4.dimensions[2] = wall_height; //Z dim


    geometry_msgs::Pose pose5;
    pose5.position.x = m_env_dim_x[0] + (wall_length_parking_slot + wall_thickness_parking_slot)/2.0;
    pose5.position.y = 4.5 - wall_thickness_parking_slot/2.0;
    pose5.position.z = wall_height/2.0;
    pose5.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive5;
    primitive5.type = primitive5.BOX;
    primitive5.dimensions.resize(3);
    primitive5.dimensions[0] = wall_length_parking_slot + wall_thickness_parking_slot; //X dim
    primitive5.dimensions[1] = wall_thickness_parking_slot; //Y dim
    primitive5.dimensions[2] = wall_height; //Z dim

    geometry_msgs::Pose pose6;
    pose6.position.x = m_env_dim_x[0] + wall_length_parking_slot + wall_thickness_parking_slot/2.0;
    pose6.position.y = 2.0 + wall_length_parking_slot/2.0;
    pose6.position.z = wall_height/2.0;
    pose6.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive6;
    primitive6.type = primitive4.BOX;
    primitive6.dimensions.resize(3);
    primitive6.dimensions[0] = wall_thickness_parking_slot; //X dim
    primitive6.dimensions[1] = wall_length_parking_slot; //Y dim
    primitive6.dimensions[2] = wall_height; //Z dim

    //Add primitive and pose
    coll_object.primitives.push_back(primitive);
    coll_object.primitives.push_back(primitive2);
    coll_object.primitives.push_back(primitive3);
    coll_object.primitives.push_back(primitive4);
    coll_object.primitives.push_back(primitive5);
    coll_object.primitives.push_back(primitive6);

    coll_object.primitive_poses.push_back(pose);
    coll_object.primitive_poses.push_back(pose2);
    coll_object.primitive_poses.push_back(pose3);
    coll_object.primitive_poses.push_back(pose4);
    coll_object.primitive_poses.push_back(pose5);
    coll_object.primitive_poses.push_back(pose6);

    //Publish on planning_scene topic
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = m_scene_name;

    //Enter other obstacles into the environment
    planning_scene_msg.world.collision_objects.push_back(coll_object);

    //Publish information
    planning_scene_msg.is_diff = true;
    m_ps_obj_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
}

//Remove an obstacle from the scene
void PlanningWorldBuilder::deleteCollisionObject(moveit_msgs::AttachedCollisionObject input_object)
{
    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = input_object.object.id;
    remove_object.header.frame_id = m_base_link_name;
    remove_object.operation = remove_object.REMOVE;


    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    m_ps_obj_pub.publish(planning_scene);

    //ros::Duration(0.5).sleep(); //sleep for half a second

}


//Insert an Object that can be attached and detached from the robot end-effector
moveit_msgs::AttachedCollisionObject PlanningWorldBuilder::insertManipulableObject(string object_name, vector<double> position, vector<double> dimension)
{
    moveit_msgs::AttachedCollisionObject attached_object;
    //attached_object.link_name = "lbr_flange_link";
    attached_object.link_name = m_base_link_name;
    /* The header must contain a valid TF frame*/
    //attached_object.object.header.frame_id = "lbr_flange_link";
    attached_object.object.header.frame_id = m_base_link_name;
    /* The id of the object */
    attached_object.object.id = object_name;

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2]+dimension[2]/2;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;


    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimension[0];
    primitive.dimensions[1] = dimension[1];
    primitive.dimensions[2] = dimension[2];

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    //Note that attaching an object to the robot requires the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    //Add the object into the environment by adding it to the set of collision objects in the “world” part of the planning scene.
    //Note that we are using only the “object” field of the attached_object message here.
    ROS_INFO("Adding the object into the world at the location of the wrist.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    m_ps_obj_pub.publish(planning_scene);
    ros::Duration(0.5).sleep(); //sleep for half a second

    return attached_object;
}


//Insert an Cart that can be attached and detached from the robot end-effector
moveit_msgs::AttachedCollisionObject PlanningWorldBuilder::insertManipulableCart(string cart_name, vector<double> position, vector<double> dimension)
{
    moveit_msgs::AttachedCollisionObject attached_object;
    //attached_object.link_name = "lbr_flange_link";
    attached_object.link_name = m_base_link_name;
    /* The header must contain a valid TF frame*/
    //attached_object.object.header.frame_id = "lbr_flange_link";
    attached_object.object.header.frame_id = m_base_link_name;
    /* The id of the object */
    attached_object.object.id = cart_name;

    /* The cart pose in the world*/
    geometry_msgs::Pose cart_pose;
    cart_pose.position.x = position[0];
    cart_pose.position.y = position[1];
    cart_pose.position.z = position[2]+dimension[2]/2;
    cart_pose.orientation.x = 0.0;
    cart_pose.orientation.y = 0.0;
    cart_pose.orientation.z = 0.0;
    cart_pose.orientation.w = 1.0;

    //Which side of the cart is shorter?
    double short_cart_side = dimension[0] < dimension[1] ? dimension[0] : dimension[1];
    string short_cart_side_name = dimension[0] < dimension[1] ? "x" : "y";


    /* The cart wheels*/
    geometry_msgs::Pose wheel_0_pose;
    wheel_0_pose.position.x = short_cart_side_name == "x" ? cart_pose.position.x - dimension[0]/2 + dimension[0]/4 : cart_pose.position.x - dimension[0]/2 + dimension[0]/6;
    wheel_0_pose.position.y = short_cart_side_name == "x" ? cart_pose.position.y - dimension[1]/2 + dimension[1]/6 : cart_pose.position.y - dimension[1]/2 + dimension[1]/4;
    wheel_0_pose.position.z = cart_pose.position.z - dimension[2]/2 + short_cart_side/8.0;
    wheel_0_pose.orientation.x = 0.0;
    wheel_0_pose.orientation.y = 0.0;
    wheel_0_pose.orientation.z = 0.0;
    wheel_0_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive wheel_0_dim;
    wheel_0_dim.type = wheel_0_dim.SPHERE;
    wheel_0_dim.dimensions.resize(1);
    wheel_0_dim.dimensions[0] = short_cart_side/8.0; //Wheel radius

    geometry_msgs::Pose wheel_1_pose;
    wheel_1_pose.position.x = short_cart_side_name == "x" ? cart_pose.position.x + dimension[0]/2 - dimension[0]/4 : cart_pose.position.x + dimension[0]/2 - dimension[0]/6;
    wheel_1_pose.position.y = short_cart_side_name == "x" ? cart_pose.position.y - dimension[1]/2 + dimension[1]/6 : cart_pose.position.y - dimension[1]/2 + dimension[1]/4;
    wheel_1_pose.position.z = cart_pose.position.z - dimension[2]/2 + short_cart_side/8.0;
    wheel_1_pose.orientation.x = 0.0;
    wheel_1_pose.orientation.y = 0.0;
    wheel_1_pose.orientation.z = 0.0;
    wheel_1_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive wheel_1_dim;
    wheel_1_dim.type = wheel_1_dim.SPHERE;
    wheel_1_dim.dimensions.resize(1);
    wheel_1_dim.dimensions[0] = short_cart_side/8.0; //Wheel radius


    geometry_msgs::Pose wheel_2_pose;
    wheel_2_pose.position.x = short_cart_side_name == "x" ? cart_pose.position.x - dimension[0]/2 + dimension[0]/4 : cart_pose.position.x - dimension[0]/2 + dimension[0]/6;
    wheel_2_pose.position.y = short_cart_side_name == "x" ? cart_pose.position.y + dimension[1]/2 - dimension[1]/6 : cart_pose.position.y + dimension[1]/2 - dimension[1]/4;
    wheel_2_pose.position.z = cart_pose.position.z - dimension[2]/2 + short_cart_side/8.0;
    wheel_2_pose.orientation.x = 0.0;
    wheel_2_pose.orientation.y = 0.0;
    wheel_2_pose.orientation.z = 0.0;
    wheel_2_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive wheel_2_dim;
    wheel_2_dim.type = wheel_2_dim.SPHERE;
    wheel_2_dim.dimensions.resize(1);
    wheel_2_dim.dimensions[0] = short_cart_side/8.0; //Wheel radius


    geometry_msgs::Pose wheel_3_pose;
    wheel_3_pose.position.x = short_cart_side_name == "x" ? cart_pose.position.x + dimension[0]/2 - dimension[0]/4 : cart_pose.position.x + dimension[0]/2 - dimension[0]/6;
    wheel_3_pose.position.y = short_cart_side_name == "x" ? cart_pose.position.y + dimension[1]/2 - dimension[1]/6 : cart_pose.position.y + dimension[1]/2 - dimension[1]/4;
    wheel_3_pose.position.z = cart_pose.position.z - dimension[2]/2 + short_cart_side/8.0;
    wheel_3_pose.orientation.x = 0.0;
    wheel_3_pose.orientation.y = 0.0;
    wheel_3_pose.orientation.z = 0.0;
    wheel_3_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive wheel_3_dim;
    wheel_3_dim.type = wheel_3_dim.SPHERE;
    wheel_3_dim.dimensions.resize(1);
    wheel_3_dim.dimensions[0] = short_cart_side/8.0; //Wheel radius


    geometry_msgs::Pose cart_box_pose;
    cart_box_pose.position.x = cart_pose.position.x;
    cart_box_pose.position.y = cart_pose.position.y;
    cart_box_pose.position.z = cart_pose.position.z - dimension[2]/2 + 2*short_cart_side/8.0 + (dimension[2] - 2*(short_cart_side/8.0))/2;
    cart_box_pose.orientation.x = 0.0;
    cart_box_pose.orientation.y = 0.0;
    cart_box_pose.orientation.z = 0.0;
    cart_box_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive cart_box_dim;
    cart_box_dim.type = cart_box_dim.BOX;
    cart_box_dim.dimensions.resize(3);
    cart_box_dim.dimensions[0] = dimension[0];
    cart_box_dim.dimensions[1] = dimension[1];
    cart_box_dim.dimensions[2] = dimension[2] - 2*(short_cart_side/8.0);


    attached_object.object.primitives.push_back(wheel_0_dim);
    attached_object.object.primitive_poses.push_back(wheel_0_pose);

    attached_object.object.primitives.push_back(wheel_1_dim);
    attached_object.object.primitive_poses.push_back(wheel_1_pose);

    attached_object.object.primitives.push_back(wheel_2_dim);
    attached_object.object.primitive_poses.push_back(wheel_2_pose);

    attached_object.object.primitives.push_back(wheel_3_dim);
    attached_object.object.primitive_poses.push_back(wheel_3_pose);

    attached_object.object.primitives.push_back(cart_box_dim);
    attached_object.object.primitive_poses.push_back(cart_box_pose);

    //Note that attaching an object to the robot requires the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    //Add the object into the environment by adding it to the set of collision objects in the “world” part of the planning scene.
    //Note that we are using only the “object” field of the attached_object message here.
    ROS_INFO("Adding the cart into the world at the location of the wrist.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    m_ps_obj_pub.publish(planning_scene);
    ros::Duration(0.5).sleep(); //sleep for half a second

    return attached_object;
}


//Insert a Glass that can be attached and detached from the robot end-effector
moveit_msgs::AttachedCollisionObject PlanningWorldBuilder::insertManipulableGlass(string object_name, vector<double> position, vector<double> dimension)
{
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = m_base_link_name;
    attached_object.object.header.frame_id = m_base_link_name;
    attached_object.object.id = object_name;

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;


    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimension[0];
    primitive.dimensions[1] = dimension[1];
    primitive.dimensions[2] = dimension[2];

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    //Note that attaching an object to the robot requires the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    //Add the object into the environment by adding it to the set of collision objects in the “world” part of the planning scene.
    //Note that we are using only the “object” field of the attached_object message here.
    ROS_INFO("Adding the object into the world at the location of the wrist.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    m_ps_obj_pub.publish(planning_scene);
    ros::Duration(0.5).sleep(); //sleep for half a second

    return attached_object;

}

//Attach the Object to the robot end-effector
moveit_msgs::AttachedCollisionObject PlanningWorldBuilder::attachObjecttoEndeffector(moveit_msgs::AttachedCollisionObject attached_object, vector<double> curr_ee_pose)
{
    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = attached_object.object.id;
    remove_object.header.frame_id = m_base_link_name;
    remove_object.operation = remove_object.REMOVE;

    //Reset name of the link the object is attached to
    attached_object.link_name = "lbr_flange_link";
    attached_object.object.header.frame_id = "lbr_flange_link";


    //Compute Pose of Attached Object in the Endeffector Frame
    for(int obj = 0; obj < attached_object.object.primitive_poses.size() ; obj++)
    {
        // --------------------- Prepare Object Transform w.r.t Global Frame --------------------

        //+ reset the position which is now described w.r.t the new link
        double global_pos_x = attached_object.object.primitive_poses[obj].position.x;
        double global_pos_y = attached_object.object.primitive_poses[obj].position.y;
        double global_pos_z = attached_object.object.primitive_poses[obj].position.z;
        double global_orient_x = attached_object.object.primitive_poses[obj].orientation.x;
        double global_orient_y = attached_object.object.primitive_poses[obj].orientation.y;
        double global_orient_z = attached_object.object.primitive_poses[obj].orientation.z;
        double global_orient_w = attached_object.object.primitive_poses[obj].orientation.w;
        //Normalize Quaternion
        double quat_norm = sqrt(global_orient_x*global_orient_x + global_orient_y*global_orient_y + global_orient_z*global_orient_z + global_orient_w*global_orient_w);
        global_orient_x /= quat_norm;
        global_orient_y /= quat_norm;
        global_orient_z /= quat_norm;
        global_orient_w /= quat_norm;

        //Build KDL Frames for object pose
        KDL::Vector position_object(global_pos_x,global_pos_y,global_pos_z);
        KDL::Rotation rotation_object;
        rotation_object = rotation_object.Quaternion(global_orient_x,global_orient_y,global_orient_z,global_orient_w);
        //rotation_object = rotation_object.EulerZYX(rot_z,rot_y,rot_x);
        KDL::Frame object_pose_wrt_global_frame(rotation_object,position_object);


        // --------------------- Prepare End-Effector Transform w.r.t Global Frame --------------------

        //Normalize Quaternion
        quat_norm = sqrt(curr_ee_pose[3]*curr_ee_pose[3] + curr_ee_pose[4]*curr_ee_pose[4] + curr_ee_pose[5]*curr_ee_pose[5] + curr_ee_pose[6]*curr_ee_pose[6]);
        curr_ee_pose[3] /= quat_norm;
        curr_ee_pose[4] /= quat_norm;
        curr_ee_pose[5] /= quat_norm;
        curr_ee_pose[6] /= quat_norm;
        //Build KDL Frames for endeffector pose
        KDL::Vector position_ee(curr_ee_pose[0],curr_ee_pose[1],curr_ee_pose[2]);
        KDL::Rotation rotation_ee;
        rotation_ee = rotation_ee.Quaternion(curr_ee_pose[3],curr_ee_pose[4],curr_ee_pose[5],curr_ee_pose[6]);
        KDL::Frame ee_pose_wrt_global_frame(rotation_ee,position_ee);


        // --------------------- Print global Transforms --------------------
        //cout<<"Global Pos Obj: "<<global_pos_x<<" "<<global_pos_y<<" "<<global_pos_z<<" "<<global_orient_x<<" "<<global_orient_y<<" "<<global_orient_z<<" "<<global_orient_w<<endl;
        //cout<<"Global Pos EE: "<<ee_pose_wrt_global_frame.p.x()<<" "<<ee_pose_wrt_global_frame.p.y()<<" "<<ee_pose_wrt_global_frame.p.z()<<curr_ee_pose[3]<<" "<<curr_ee_pose[4]<<" "<<curr_ee_pose[5]<<" "<<curr_ee_pose[6]<<endl;


        // --------------------- Compute Object Transform w.r.t End-Effector Frame --------------------

        ///Express object pose w.r.t ee frame
        KDL::Frame ob_wrt_ee = ee_pose_wrt_global_frame.Inverse() * object_pose_wrt_global_frame;
        m_object_pose_wrt_ee_frame.push_back(ob_wrt_ee);
        //cout<<"Pos Obj w.r.t EE start pose: "<<pose_wrt_ee_frame.p.x()<<" "<<pose_wrt_ee_frame.p.y()<<" "<<pose_wrt_ee_frame.p.z()<<endl;


        // --------------------- Set New Object Transform w.r.t End-Effector Frame --------------------
        //Set new Object Pose, now expressed w.r.t ee frame
        attached_object.object.primitive_poses[obj].position.x = m_object_pose_wrt_ee_frame[obj].p.x();
        attached_object.object.primitive_poses[obj].position.y = m_object_pose_wrt_ee_frame[obj].p.y();
        attached_object.object.primitive_poses[obj].position.z = m_object_pose_wrt_ee_frame[obj].p.z();
        m_object_pose_wrt_ee_frame[obj].M.GetQuaternion(attached_object.object.primitive_poses[obj].orientation.x, attached_object.object.primitive_poses[obj].orientation.y\
                                          ,attached_object.object.primitive_poses[obj].orientation.z,attached_object.object.primitive_poses[obj].orientation.w);


    }
    //ros::Publisher att_object_in_map_pub_;
    //att_object_in_map_pub_  = m_nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
    //att_object_in_map_pub_.publish(attached_object);
    //sleep(2);


    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the object to the wrist and removing it from the world.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    m_ps_obj_pub.publish(planning_scene);

    ros::Duration(0.5).sleep(); //sleep for half a second

    return attached_object;

}

//Detach the Object to the robot end-effector
void PlanningWorldBuilder::detachObjectfromEndeffector(moveit_msgs::AttachedCollisionObject attached_object, vector<double> curr_ee_pose)
{
    /* First, define the DETACH object message*/
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = attached_object.object.id;
    detach_object.link_name = "lbr_flange_link";
    detach_object.object.operation = attached_object.object.REMOVE;

    //Reset name of the link the object is attached to, to the world link
    attached_object.link_name = m_base_link_name;
    attached_object.object.header.frame_id = m_base_link_name;


    //Convert current end-effector pose to KDL::Frame
    //Normalize Quaternion
    double quat_norm = sqrt(curr_ee_pose[3]*curr_ee_pose[3] + curr_ee_pose[4]*curr_ee_pose[4] + curr_ee_pose[5]*curr_ee_pose[5] + curr_ee_pose[6]*curr_ee_pose[6]);
    curr_ee_pose[3] /= quat_norm;
    curr_ee_pose[4] /= quat_norm;
    curr_ee_pose[5] /= quat_norm;
    curr_ee_pose[6] /= quat_norm;
    //Build KDL Frames for endeffector pose
    KDL::Vector position_ee(curr_ee_pose[0],curr_ee_pose[1],curr_ee_pose[2]);
    KDL::Rotation rotation_ee;
    rotation_ee = rotation_ee.Quaternion(curr_ee_pose[3],curr_ee_pose[4],curr_ee_pose[5],curr_ee_pose[6]);
    KDL::Frame ee_pose_wrt_global_frame(rotation_ee,position_ee);


    //Set Pose of Attached Object in the World Frame
    for(int obj = 0; obj < attached_object.object.primitive_poses.size() ; obj++)
    {
        ///Express object pose w.r.t global frame
        /// Note: "m_object_pose_wrt_ee_frame" has been defined in "attachObjecttoEndeffector" and remains constant while the end-effector is holding the object
        KDL::Frame object_pose_wrt_global_frame = ee_pose_wrt_global_frame * m_object_pose_wrt_ee_frame[obj];
        //Get Orientation
        double qx,qy,qz,qw;
        object_pose_wrt_global_frame.M.GetQuaternion(qx,qy,qz,qw);

        //Set new pose for object, now expressed w.r.t global frame
        attached_object.object.primitive_poses[obj].position.x = object_pose_wrt_global_frame.p.x();
        attached_object.object.primitive_poses[obj].position.y = object_pose_wrt_global_frame.p.y();
        attached_object.object.primitive_poses[obj].position.z = object_pose_wrt_global_frame.p.z();
        attached_object.object.primitive_poses[obj].orientation.x = qx;
        attached_object.object.primitive_poses[obj].orientation.y = qy;
        attached_object.object.primitive_poses[obj].orientation.z = qz;
        attached_object.object.primitive_poses[obj].orientation.w = qw;
    }


    /* Carry out the DETACH + ADD operation */
    ROS_INFO("Detaching the object from the robot end-effector and returning it to the world.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(attached_object.object);
    m_ps_obj_pub.publish(planning_scene);

    ros::Duration(0.5).sleep(); //sleep for half a second

//    /* First, define the REMOVE object message*/
//    moveit_msgs::CollisionObject remove_object;
//    remove_object.id = attached_object.object.id;
//    remove_object.header.frame_id = m_base_link_name;
//    remove_object.operation = remove_object.REMOVE;

//    ROS_INFO("Removing the object from the world.");
//    planning_scene.robot_state.attached_collision_objects.clear();
//    planning_scene.world.collision_objects.clear();
//    planning_scene.world.collision_objects.push_back(remove_object);
//    m_ps_obj_pub.publish(planning_scene);
}


//Get the size of the environment
void PlanningWorldBuilder::getEnvironmentDimensions(vector<double>& dim_x,vector<double>& dim_y)
{
    //Get environemnt size (x,y dimension)
    dim_x = m_env_dim_x;
    dim_y = m_env_dim_y;
}


//----------------------- Insert Obstacles (Multi-Robot Planning) ----------------------------

moveit_msgs::AttachedCollisionObject PlanningWorldBuilder::insertRobotBase(vector<double> base_2d_pos, vector<double> base_2d_orient, string robot_name)
{
    moveit_msgs::AttachedCollisionObject rob_base;
    rob_base.link_name = m_base_link_name;
    rob_base.object.header.frame_id = m_base_link_name;
    rob_base.object.id = robot_name + "_robot_base";

    geometry_msgs::Pose pose;
    pose.position.x = base_2d_pos[0];
    pose.position.y = base_2d_pos[1];
    pose.orientation.x = base_2d_orient[0];
    pose.orientation.y = base_2d_orient[1];
    pose.orientation.z = base_2d_orient[2];
    pose.orientation.w = base_2d_orient[3];
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);


    if(robot_name == "robotino")
    {
        pose.position.z = 0.3/2.0;

        primitive.dimensions[0] = 0.45; //X dim
        primitive.dimensions[1] = 0.45; //Y dim
        primitive.dimensions[2] = 0.3;//Z dim

    }
    else if(robot_name == "omnirob")
    {

        pose.position.z = 0.65/2.0;

        primitive.dimensions[0] = 0.6; //X dim
        primitive.dimensions[1] = 1.2; //Y dim
        primitive.dimensions[2] = 0.65;//Z dim


    }
    else
        ROS_ERROR("Robot name in insertRobotBase not known!!!");

    //Add primitive and pose
    rob_base.object.primitives.push_back(primitive);
    rob_base.object.primitive_poses.push_back(pose);

    //Specify operation as an ADD operation
    rob_base.object.operation = rob_base.object.ADD;


    //Add the object into the environment by adding it to the set of collision objects in the “world” part of the planning scene.
    //Note that we are using only the “object” field of the attached_object message here.
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(rob_base.object);
    planning_scene.is_diff = true;
    m_ps_obj_pub.publish(planning_scene);
    //ros::Duration(0.5).sleep(); //sleep for half a second


    return rob_base;

}


} //End namespace
