// --- Includes -- 
#include <ros/ros.h>
#include <kuka_motion_control/kdl_kuka_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>



#ifndef PLANNING_WORLD_BUILDER_H
#define PLANNING_WORLD_BUILDER_H

// --Namespaces --
using namespace std;


namespace planning_world{


class PlanningWorldBuilder
{
	public:
    PlanningWorldBuilder(string robot_desciption_param, string planning_group, string ns_prefix = "");
    ~PlanningWorldBuilder();
	
    //Insert Borders, i.e. walls confining the robot's workspace
    void insertEnvironmentBorders(vector<double> size_x, vector<double> size_y, double wall_height, string scene_name  = "empty_scene");

    //----------------------- Insert Obstacles ----------------------------

    //Insert a wall
    void insertWall(vector<double> position, vector<double> dimension);

    //Insert a table top scene into the world
    void insertTableTop(vector<double> table_pos, vector<double> table_dim);

    //Insert a rack scene into the world
    void insertRack(vector<double> rack_pos, vector<double> rack_dim, int num_shelves, double shelves_thickness);

    //Insert a block in the center of the world
    void insertBlock(vector<double> position, vector<double> dimension);

    //Insert a maze
    void insertMaze(int num_walls, double wall_thickness, double wall_length_ratio);

    //Insert a random maze (to be implemented!)
    void insertRandomMaze(int num_walls);

    //Insert narrow passage
    void insertNarrowPassage(double wall_thickness, double wall_length_ratio);

    //Insert "three passages" scenario
    void insertThreeGatesPassage(double wall_thickness, double wall_length_ratio);


    //----------------------- Insert Obstacles (Multi-Robot Planning) ----------------------------

    moveit_msgs::AttachedCollisionObject insertRobotBase(vector<double> base_2d_pos, vector<double> base_2d_orient, string robot_name);



    //------------------- Evaluation Scenarios ------------------------

    //Insert Tunnel Scenario
    void insertTunnelPassage(double tunnel_width, double tunnel_height, double wall_length_ratio);

    //Insert TwoRooms Office World
    void insertTwoRoomsOffice(double wall_thickness, double width_narrow_passage);

    //Insert Narrow Corridor Scenario
    void insertNarrowCorridor(double corridor_width, double wall_length_ratio);

    //Insert Glass Delivery Scenario
    void insertGlassDeliveryWorld(double tunnel_width, double tunnel_height, double ceiling_height);

    //Insert Cart Parking Scenario
    void insertParkingSlot(double narrow_passage_offset, double wall_thickness);


    //----------------------- Remove Obstacles ----------------------------

    //Remove an obstacle from the scene
    void deleteCollisionObject(moveit_msgs::AttachedCollisionObject input_object);

    //----------------------- Insert Manipulable Objects ----------------------------

    //Insert an Object that can be attached and detached from the robot end-effector
    moveit_msgs::AttachedCollisionObject insertManipulableObject(string object_name, vector<double> position, vector<double> dimension);

    //Insert a Cart that can be attached and detached from the robot end-effector
    moveit_msgs::AttachedCollisionObject insertManipulableCart(string cart_name, vector<double> position, vector<double> dimension);

    //Insert a Glass that can be attached and detached from the robot end-effector
    moveit_msgs::AttachedCollisionObject insertManipulableGlass(string object_name, vector<double> position, vector<double> dimension);

    //Attach the Object to the robot end-effector
    moveit_msgs::AttachedCollisionObject attachObjecttoEndeffector(moveit_msgs::AttachedCollisionObject attached_object, vector<double> curr_ee_pose);

    //Detach the Object to the robot end-effector
    void detachObjectfromEndeffector(moveit_msgs::AttachedCollisionObject attached_object, vector<double> curr_ee_pose);


    //----------------------- Get Functions ----------------------------

    //Get the size of the environment
    void getEnvironmentDimensions(vector<double> &dim_x, vector<double> &dim_y);

    //Get Name of the loaded Environment
    string getSceneName(){return m_scene_name;}


    //----------------------- Set Functions ----------------------------

    //Set Name of the loaded Environment
    void setSceneName(string scene_name){m_scene_name = scene_name;}




    private:
    //-- Class Objects --

    //Node handle
    ros::NodeHandle m_nh;

    //Planning Scene Publisher (for adding collision objects to the planning scene)
    ros::Publisher m_ps_obj_pub;
    //planning_scene_monitor::PlanningSceneMonitorPtr m_planning_scene_monitor;
	
	//Generate KDL Tree from Kuka LBR urdf
    // -> used to generate samples for the kinematic chain of the robot
    boost::shared_ptr<kuka_motion_controller::KDLRobotModel> m_KDLRobotModel;
	
	//Name of Base Link Frame of Kinematic Chain
    string m_base_link_name;

    //Name of the Scene
    string m_scene_name;

    //Environment size
    vector<double> m_env_dim_x; //m_env_dim_x[0] = size in negative x dir / m_env_dim_x[1] = size in positive x dir
    vector<double> m_env_dim_y; //m_env_dim_y[0] = size in negative y dir / m_env_dim_y[1] = size in positive y dir


    //Number of walls
    int m_num_walls;

    //Object Pose w.r.t end-effector frame (used for detaching object from ee frame and returning it into the world frame)
    vector<KDL::Frame> m_object_pose_wrt_ee_frame;

};




}//end of namespace

#endif // PLANNING_WORLD_BUILDER_H
