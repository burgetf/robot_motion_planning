#include <validity_checker/feasibility_checker.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>


namespace state_feasibility_checker{

FeasibilityChecker::FeasibilityChecker(string robot_desciption_param, string planning_group, string ns_prefix)
{
    //Check the planning frame (from virtual joint in srdf)
    m_planning_frame = getPlanningFrameFromSRDF(robot_desciption_param);

	//Planning Scene Monitor (required for collision checks)
    m_planning_scene_monitor =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_desciption_param);

    //Namespace prefix for robot
    m_ns_prefix_robot = ns_prefix;

    //Get topic prefix in constructor argument list -> to set topic names e.g. "robotino_group/planning_scene"
    string planning_scene_ns = ns_prefix + "planning_scene";
    string planning_scene_service_ns = ns_prefix + "get_planning_scene";
    string endeffector_trajectory_ns = ns_prefix + "endeffector_trajectory";

    //string joint_states_ns = ns_prefix + "joint_states";
    //string attached_collision_object_ns = ns_prefix + "attached_collision_object";
    //string collision_object_ns = ns_prefix + "collision_object";
    //string planning_scene_world_ns = ns_prefix + "planning_scene_world";

    //m_planning_scene_monitor->startSceneMonitor(planning_scene_ns);
    //m_planning_scene_monitor->startStateMonitor(joint_states_ns, attached_collision_object_ns);
    //m_planning_scene_monitor->startWorldGeometryMonitor(collision_object_ns,planning_scene_world_ns, false);


	//Set planning Group 
	m_planning_group = planning_group;

    //Name of Planning Scene Service;
    m_planning_scene_service = planning_scene_service_ns;
	
	 //Create Robot model
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_desciption_param, planning_scene_ns, endeffector_trajectory_ns, m_planning_group));
    
    //Get Kinematic Chain of manipulator
    m_manipulator_chain = m_KDLRobotModel->getCompleteArmChain();
    
    //Get the joint names
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Number of joints
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();
	
    //Step width along an tree edge for collision checking (used only in "isEdgeValid(Edge)" function)
    m_nh.param("collision_check_extend_step_factor", m_collision_check_extend_step_factor, 0.3);

    //Step width along an tree edge for collision checking (used only in "isEdgeValid(Edge)" function)
    //m_collision_check_extend_step_factor = 0.3;

    //Flag indicating whether transform between map and base_link is available
    m_transform_map_to_base_available = false;

}


FeasibilityChecker::FeasibilityChecker(boost::shared_ptr<kuka_motion_controller::KDLRobotModel> kdl_robot_model, string robot_desciption_param, string planning_group, string ns_prefix)
{

    //Check the planning frame (from virtual joint in srdf)
    m_planning_frame = getPlanningFrameFromSRDF(robot_desciption_param);

    //Planning Scene Monitor (required for collision checks)
    m_planning_scene_monitor =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_desciption_param);

    //Namespace prefix for robot
    m_ns_prefix_robot = ns_prefix;

    //Get topic prefix in constructor argument list -> to set topic names e.g. "robotino_group/planning_scene"
    string planning_scene_service_ns = ns_prefix + "get_planning_scene";

    //string planning_scene_ns = ns_prefix + "planning_scene";
    //string joint_states_ns = ns_prefix + "joint_states";
    //string attached_collision_object_ns = ns_prefix + "attached_collision_object";
    //string collision_object_ns = ns_prefix + "collision_object";
    //string planning_scene_world_ns = ns_prefix + "planning_scene_world";
    //string endeffector_trajectory_ns = ns_prefix + "endeffector_trajectory";

    //m_planning_scene_monitor->startSceneMonitor(planning_scene_ns);
    //m_planning_scene_monitor->startStateMonitor(joint_states_ns, attached_collision_object_ns);
    //m_planning_scene_monitor->startWorldGeometryMonitor(collision_object_ns,planning_scene_world_ns, false);


    //Set planning Group
    m_planning_group = planning_group;

    //Name of Planning Scene Service;
    m_planning_scene_service = planning_scene_service_ns;

    //Create Robot model
    m_KDLRobotModel = kdl_robot_model;

    //Get Kinematic Chain of manipulator
    m_manipulator_chain = m_KDLRobotModel->getCompleteArmChain();

    //Get the joint names
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Number of joints
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();

    //Step width along an tree edge for collision checking (used only in "isEdgeValid(Edge)" function)
    m_nh.param("collision_check_extend_step_factor", m_collision_check_extend_step_factor, 0.3);

    //Step width along an tree edge for collision checking (used only in "isEdgeValid(Edge)" function)
    //m_collision_check_extend_step_factor = 0.3;

    //Flag indicating whether transform between map and base_link is available
    m_transform_map_to_base_available = false;
}

FeasibilityChecker::~FeasibilityChecker()
{
	//Nothing to do yet
}


//Get the planning frame from the SRDF description
string FeasibilityChecker::getPlanningFrameFromSRDF(string robot_desciption_param)
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


//Set step width for collision checking along an tree edge (only used in "isEdgeValid(Edge tree_edge)")
void FeasibilityChecker::setCollisionCheckingStepWidth(double step_width)
{
    m_collision_check_extend_step_factor = step_width;
}


//Set Planning Scene Service Name
void FeasibilityChecker::setPlanningSceneServiceName(string service_name)
{
    m_planning_scene_service = service_name;
}


//Check whether an tree node config is valid (i.e. whether it is Collision-Free)
bool FeasibilityChecker::isConfigValid(vector<double> config, bool print_contacts)
{
    //Set name of planning scene service
    const std::string PLANNING_SCENE_SERVICE = m_planning_scene_service;

    //Get curent planning scene
    m_planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(m_planning_scene_monitor);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();


    //Collision checking Setup
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = m_planning_group;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

    //For contact information
    collision_request.contacts = print_contacts;
    collision_request.max_contacts = 1000;


    //Init isValid flag to be returned by the function
    bool isValid = true;


    //++++++++++ START: TESTING ++++++++++

    //Transform base config to /map frame only when localization is active (acml package)
    if(m_planning_frame == "/map" && m_num_joints_prismatic >= 2 && m_num_joints_revolute >= 1)
    {
    	//cout<<"Name of Planning frame: "<<m_planning_frame<<endl;

        if(m_transform_map_to_base_available)
        {
            //Transform base_link to sample
            tf::StampedTransform transform_base_to_sample;
            transform_base_to_sample.setOrigin(tf::Vector3(config[0],config[1], 0.0));
            transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(config[2]));


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
            config[0] = map_to_sample_conf[0];
            config[1] = map_to_sample_conf[1];
            config[2] = map_to_sample_conf[2];
        }
    }


    //++++++++++ END: TESTING ++++++++++

    //Set up Map storing the configuration of manipulator
    std::map<std::string, double> configuration;
    for (int i = 0; i < m_num_joints ; i++)
    {
        configuration[m_joint_names[i]] = config[i];
        //std::cout<<m_joint_names[i] <<": "<<configuration[m_joint_names[i]]<<std::endl;
    }

    if (m_planning_group == "kuka_complete_arm" || m_planning_group == "omnirob_lbr_sdh")
    {
        configuration["sdh2_finger_12_joint"] = -1.57;
        configuration["sdh2_finger_22_joint"] = -1.57;
        configuration["sdh2_thumb_2_joint"] = -1.57;
    }

    //Assign the robot state to the Kinematic Model
    //robot_state::RobotState state(scene->getRobotModel());
    robot_state::RobotState state = scene->getCurrentStateNonConst();
    state.setToDefaultValues();



    //Set Configuration of robot state
    state.setVariablePositions(configuration);

    //Apply robot state to planning scene
    scene->setCurrentState(state);


//    robot_state::RobotState state_test = scene->getCurrentStateNonConst();
//    double *curr_state  = state_test.getVariablePositions();
//    cout<<curr_state[0]<<endl;
//    cout<<curr_state[1]<<endl;
//    cout<<curr_state[2]<<endl;

    // -------------------------------- Collision checking -------------------------------------
    //Clear the collision checking result
    collision_result.clear();

    //Check for collisions
    scene->checkCollision(collision_request, collision_result, state, acm);

    //Print collision checking results
    //ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    //Write result of collision check to console
    if (collision_result.collision == 1)
    {
        isValid = false;
        //std::cout<< "Config is in self-collision or in collision with an obstacle"<< std::endl;

        for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); it != collision_result.contacts.end();  ++it)
        {
            ROS_INFO_STREAM("Contact between: " <<it->first.first.c_str()<<" and" <<it->first.second.c_str());
        }

    }


    //Check whether config is valid
    //if (isValid == true)
    //{
        //std::cout<< "Config is collision-free"<< std::endl;
    //}

    return isValid;
}



//Check whether an tree node config is valid (i.e. whether it is Collision-Free)
bool FeasibilityChecker::isConfigValid(KDL::JntArray config, bool print_contacts)
{

    //Set name of planning scene service
    const std::string PLANNING_SCENE_SERVICE = m_planning_scene_service;

    //Get curent planning scene
    bool success = m_planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(m_planning_scene_monitor);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();


    //Collision checking Setup
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = m_planning_group;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();


    //For contact information
    collision_request.contacts = print_contacts;
    collision_request.max_contacts = 1000;

    //collision_request.verbose = true;

    //Init isValid flag to be returned by the function
    bool isValid = true;


    //++++++++++ START: TESTING ++++++++++

    //Transform base config to /map frame only when localization is active (acml package)
    if(m_planning_frame == "/map" && m_num_joints_prismatic >= 2 && m_num_joints_revolute >= 1)
    {
        //cout<<"Name of Planning frame: "<<m_planning_frame<<endl;

        if(m_transform_map_to_base_available)
        {
            //Transform base_link to sample
            tf::StampedTransform transform_base_to_sample;
            transform_base_to_sample.setOrigin(tf::Vector3(config(0),config(1), 0.0));
            transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(config(2)));


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

            //cout<<"base_link in map frame:"<<endl;
            //cout<<m_transform_map_to_base.getOrigin().x()<<endl;
            //cout<<m_transform_map_to_base.getOrigin().y()<<endl;
            //cout<<m_transform_map_to_base.getRotation().getAngle()<<endl;

            //cout<<"Config in base_link frame:"<<endl;
            //cout<<config(0)<<endl;
            //cout<<config(1)<<endl;
            //cout<<config(2)<<endl;

            //Express base config w.r.t map frame
            config(0) = map_to_sample_conf[0];
            config(1) = map_to_sample_conf[1];
            config(2) = map_to_sample_conf[2];

            //cout<<"Config in map frame:"<<endl;
            //cout<<config(0)<<endl;
            //cout<<config(1)<<endl;
            //cout<<config(2)<<endl;

	    ROS_INFO_STREAM("Config in map frame");
            //cout << goal_pose_base.matrix() << endl << endl;
            ROS_INFO_STREAM(config(0)<<"  "<<config(1)<<"  "<<config(2));
        }
	else
	{
		ROS_ERROR("No map to base_link transform available");
	}
    }


    //++++++++++ END: TESTING ++++++++++


    //Set up Map storing the configuration of manipulator
    std::map<std::string, double> configuration;
    for (int i = 0; i < m_num_joints ; i++)
    {
        configuration[m_joint_names[i]] = config(i);
        //std::cout<<m_joint_names[i] <<": "<<configuration[m_joint_names[i]]<<std::endl;
    }

    if (m_planning_group == "kuka_complete_arm" || m_planning_group == "omnirob_lbr_sdh")
    {
        configuration["sdh2_finger_12_joint"] = -1.57;
        configuration["sdh2_finger_22_joint"] = -1.57;
        configuration["sdh2_thumb_2_joint"] = -1.57;
    }

    //Assign the robot state to the Kinematic Model
    //robot_state::RobotState state(scene->getRobotModel());
    robot_state::RobotState state = scene->getCurrentStateNonConst();

    //Set State to default values
    state.setToDefaultValues();

    //Set Configuration of robot state
    state.setVariablePositions(configuration);

    //Apply robot state to planning scene
    scene->setCurrentState(state);


    // -------------------------------- Collision checking (self-collision)-------------------------------------
    //Clear the collision checking result
    collision_result.clear();

//    //Check for self- collisions
//    scene->checkSelfCollision(collision_request, collision_result, state, acm);

//    //Write result of collision check to console
//    if (collision_result.collision == 1)
//    {
//        isValid = false;

//        std::cout<< "Config is in self-collision !!!"<< std::endl;

//        for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); it != collision_result.contacts.end();  ++it)
//        {
//          ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
//        }

//        return isValid;
//    }


    // -------------------------------- Collision checking (all-collisions)-------------------------------------

    //Check for collisions
    scene->checkCollision(collision_request, collision_result, state, acm);


    //Print collision checking results
    //ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    //Write result of collision check to console
    if (collision_result.collision == 1)
    {
        isValid = false;

        //std::cout<< "Config is in collision with an obstacle"<< std::endl;

        for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); it != collision_result.contacts.end();  ++it)
        {
          ROS_INFO_STREAM("Contact between: " <<it->first.first.c_str()<<" and" <<it->first.second.c_str());
        }

        return isValid;
    }


    //Check whether config is valid
    if (isValid == true)
    {
        //std::cout<< "Config is collision-free"<< std::endl;
    }

    return isValid;
}


//Check whether an tree edge is valid (i.e. whether all of it Configuration are Collision-Free)
bool FeasibilityChecker::isEdgeValid(Edge tree_edge, bool print_contacts)
{


//    //Set name of planning scene service
//    const std::string PLANNING_SCENE_SERVICE = m_planning_scene_service;

//    //Get curent planning scene
//    m_planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
//    planning_scene_monitor::LockedPlanningSceneRW ps(m_planning_scene_monitor);
//    ps->getCurrentStateNonConst().update();
//    //if you want to modify it
//    planning_scene::PlanningScenePtr scene = ps->diff();
//    scene->decoupleParent();


//    //Collision checking Setup
//    collision_detection::CollisionRequest collision_request;
//    collision_request.group_name = m_planning_group;
//    collision_detection::CollisionResult collision_result;
//    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

//    //For contact information
//    collision_request.contacts = print_contacts;
//    collision_request.max_contacts = 1000;


//    //Init isValid flag to be returned by the function
//    bool isValid = true;

//    //Define start and goal node corresponding to first and last config of edge
//    Node start_node;
//    Node end_node;
//    //Set start node to first config of egde
//    start_node.config = tree_edge.joint_trajectory[0];


//    //Get current pose of robot in the map frame
//    tf::TransformListener listener;
//    //Flag indicating whether transform between map and base_link is available
//    bool m_transform_map_to_base_available = false;
//    tf::StampedTransform m_transform_map_to_base;
//    //Get TF only if planning frame is /map (i.e. localization is active)
//    if(m_planning_frame == "/map"){
//        try {
//            listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
//            listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
//            m_m_transform_map_to_base_available = true;
//        } catch (tf::TransformException ex) {
//            //ROS_ERROR("%s",ex.what());
//            m_transform_map_to_base_available = false;
//        }
//    }


//    bool trajectory_end_reached = false;
//    while (trajectory_end_reached == false)
//    {
//        //Reset end node config to last edge config
//        end_node.config = tree_edge.joint_trajectory[tree_edge.joint_trajectory.size()-1];
//        //Modify end node config by stepping from the current start node towards the end node
//        trajectory_end_reached = stepAlongEdge(start_node,end_node, m_collision_check_extend_step_factor);


//        //++++++++++ START: TESTING ++++++++++

//        if(m_num_joints_prismatic == 2 && m_num_joints_revolute >= 1)
//        {
//            if(transform_map_to_base_available)
//            {
//                //Transform base_link to sample
//                tf::StampedTransform transform_base_to_sample;
//                transform_base_to_sample.setOrigin(tf::Vector3(end_node.config[0],end_node.config[1], 0.0));
//                transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(end_node.config[2]));


//                //Transform map frame to sample
//                tf::StampedTransform transform_map_to_sample;
//                transform_map_to_sample.mult(m_transform_map_to_base,transform_base_to_sample);

//                //Sample in map frame (as vector)
//                vector<double> map_to_sample_conf(3);
//                tf::Vector3 map_to_sample_trans = transform_map_to_sample.getOrigin();
//                tf::Quaternion map_to_sample_rot = transform_map_to_sample.getRotation();
//                map_to_sample_conf[0] = map_to_sample_trans.x();
//                map_to_sample_conf[1] = map_to_sample_trans.y();
//                double z_dir = transform_map_to_sample.getRotation().getAxis().z();
//                map_to_sample_conf[2] = z_dir > 0.0 ? map_to_sample_rot.getAngle() : -map_to_sample_rot.getAngle();

//                //Express base config w.r.t map frame
//                end_node.config[0] = map_to_sample_conf[0];
//                end_node.config[1] = map_to_sample_conf[1];
//                end_node.config[2] = map_to_sample_conf[2];
//            }
//        }

//        //++++++++++ END: TESTING ++++++++++

//        //Set up Map storing the configuration of the robot
//        std::map<std::string, double> configuration;
//        for (int i = 0; i < m_num_joints ; i++)
//        {
//            configuration[m_joint_names[i]] = end_node.config[i];
//            //std::cout<<m_joint_names[i] <<": "<<configuration[m_joint_names[i]]<<std::endl;
//        }

//        if (m_planning_group == "kuka_complete_arm" || m_planning_group == "omnirob_lbr_sdh")
//        {
//            configuration["sdh2_finger_12_joint"] = -1.57;
//            configuration["sdh2_finger_22_joint"] = -1.57;
//            configuration["sdh2_thumb_2_joint"] = -1.57;
//        }

//        //Assign the robot state to the Kinematic Model
//        //robot_state::RobotState state(scene->getRobotModel());
//        robot_state::RobotState state = scene->getCurrentStateNonConst();
//        state.setToDefaultValues();

//        //Set Configuration of robot state
//        state.setVariablePositions(configuration);

//        //Apply robot state to planning scene
//        scene->setCurrentState(state);


//        //--- Collision checking ---

//        //Clear the collision checking result
//        collision_result.clear();

//        //Check for collisions
//        scene->checkCollision(collision_request, collision_result, state, acm);


//        //Write result of collision check to console and return false if an config of the edge is in collision
//        if (collision_result.collision == 1)
//        {
//            //Set validity to false
//            isValid = false;

//            for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); it != collision_result.contacts.end();  ++it)
//            {
//              ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
//            }

//            //Return to function that called "isEdgeValid" if a colliding state has been detected
//            return isValid;
//        }
//        else
//        {
//            //std::cout<< "Current edge config collision-free"<< std::endl;
//        }

//        //Set start node to extended config
//        start_node.config = end_node.config;
//    }

//    return isValid;


    //Set name of planning scene service
    const std::string PLANNING_SCENE_SERVICE = m_planning_scene_service;

    //Get curent planning scene
    m_planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(m_planning_scene_monitor);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();


    //Collision checking Setup
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = m_planning_group;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

    //For contact information
    collision_request.contacts = print_contacts;
    collision_request.max_contacts = 1000;


    //Init isValid flag to be returned by the function
    bool isValid = true;


    //Iterate through configurations of the edge (edp = EDge Points)
    for (int edp = 0 ; edp < tree_edge.joint_trajectory.size() ; edp++)
    {

        if(m_planning_frame == "/map" && m_num_joints_prismatic >= 2 && m_num_joints_revolute >= 1)
        {
            if(m_transform_map_to_base_available)
            {
                cout<<"Transform map to base available!"<<endl;

                //Transform base_link to sample
                tf::StampedTransform transform_base_to_sample;
                transform_base_to_sample.setOrigin(tf::Vector3(tree_edge.joint_trajectory[edp][0],tree_edge.joint_trajectory[edp][1], 0.0));
                transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(tree_edge.joint_trajectory[edp][2]));


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
                tree_edge.joint_trajectory[edp][0] = map_to_sample_conf[0];
                tree_edge.joint_trajectory[edp][1] = map_to_sample_conf[1];
                tree_edge.joint_trajectory[edp][2] = map_to_sample_conf[2];
            }
        }

        //++++++++++ END: TESTING ++++++++++

        //Set up Map storing the configuration of manipulator
        std::map<std::string, double> configuration;
        for (int i = 0; i < m_num_joints ; i++)
        {
            configuration[m_joint_names[i]] = tree_edge.joint_trajectory[edp][i];
            //std::cout<<m_joint_names[i] <<": "<<configuration[m_joint_names[i]]<<std::endl;
        }


        if (m_planning_group == "kuka_complete_arm" || m_planning_group == "omnirob_lbr_sdh")
        {
            configuration["sdh2_finger_12_joint"] = -1.57;
            configuration["sdh2_finger_22_joint"] = -1.57;
            configuration["sdh2_thumb_2_joint"] = -1.57;
        }


        //Assign the robot state to the Kinematic Model
        //robot_state::RobotState state(scene->getRobotModel());
        robot_state::RobotState state = scene->getCurrentStateNonConst();
        state.setToDefaultValues();

        //Set Configuration of robot state
        state.setVariablePositions(configuration);

        //Apply robot state to planning scene
        scene->setCurrentState(state);


        // -------------------------------- Collision checking -------------------------------------
        //Clear the collision checking result
        collision_result.clear();

        //Check for collisions
        scene->checkCollision(collision_request, collision_result, state, acm);

        //Write result of collision check to console and return false if an config of the edge is in collision
        if (collision_result.collision == 1)
        {
            isValid = false;
            //std::cout<< "Edge config is in collision with an obstacle"<< std::endl;

            for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); it != collision_result.contacts.end();  ++it)
            {
              ROS_INFO_STREAM("Contact between: " <<it->first.first.c_str()<<" and" <<it->first.second.c_str());
            }


            //Return to function that called "isEdgeValid" if a colliding state has been detected
            break;
        }
        else
        {
            //std::cout<< "Edge config collision free <<std::endl;
        }

    }


    return isValid;

}


//Check whether an tree edge is valid (i.e. whether all of it Configuration are Collision-Free)
// -> additionally returns index of last valid node
bool FeasibilityChecker::isEdgeValid(Edge tree_edge, int &last_valid_node_idx, bool print_contacts)
{
    //Set name of planning scene service
    const std::string PLANNING_SCENE_SERVICE = m_planning_scene_service;

    //Get curent planning scene
    m_planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(m_planning_scene_monitor);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();


    //Collision checking Setup
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = m_planning_group;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

    //For contact information
    collision_request.contacts = print_contacts;
    collision_request.max_contacts = 1000;


    //Init isValid flag to be returned by the function
    bool isValid = true;

    //Init last valid node
    last_valid_node_idx = 0;


    //Iterate through configurations of the edge (edp = EDge Points)
    for (int edp = 0 ; edp < tree_edge.joint_trajectory.size() ; edp++)
    {
        //cout<<edp<<endl;

        if(m_planning_frame == "/map" && m_num_joints_prismatic >= 2 && m_num_joints_revolute >= 1)
        {
            if(m_transform_map_to_base_available)
            {
                //Transform base_link to sample
                tf::StampedTransform transform_base_to_sample;
                transform_base_to_sample.setOrigin(tf::Vector3(tree_edge.joint_trajectory[edp][0],tree_edge.joint_trajectory[edp][1], 0.0));
                transform_base_to_sample.setRotation(tf::createQuaternionFromYaw(tree_edge.joint_trajectory[edp][2]));


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
                tree_edge.joint_trajectory[edp][0] = map_to_sample_conf[0];
                tree_edge.joint_trajectory[edp][1] = map_to_sample_conf[1];
                tree_edge.joint_trajectory[edp][2] = map_to_sample_conf[2];
            }
        }

        //++++++++++ END: TESTING ++++++++++

        //Set up Map storing the configuration of manipulator
        std::map<std::string, double> configuration;
        for (int i = 0; i < m_num_joints ; i++)
        {
            configuration[m_joint_names[i]] = tree_edge.joint_trajectory[edp][i];
            //std::cout<<m_joint_names[i] <<": "<<configuration[m_joint_names[i]]<<std::endl;
        }

        if (m_planning_group == "kuka_complete_arm" || m_planning_group == "omnirob_lbr_sdh")
        {
            configuration["sdh2_finger_12_joint"] = -1.57;
            configuration["sdh2_finger_22_joint"] = -1.57;
            configuration["sdh2_thumb_2_joint"] = -1.57;
        }

        //Assign the robot state to the Kinematic Model
        //robot_state::RobotState state(scene->getRobotModel());
        robot_state::RobotState state = scene->getCurrentStateNonConst();
        state.setToDefaultValues();

        //Set Configuration of robot state
        state.setVariablePositions(configuration);

        //Apply robot state to planning scene
        scene->setCurrentState(state);



        // -------------------------------- Collision checking -------------------------------------
        //Clear the collision checking result
        collision_result.clear();

        //Check for collisions
        scene->checkCollision(collision_request, collision_result, state, acm);

        //Write result of collision check to console and return false if an config of the edge is in collision
        if (collision_result.collision == 1)
        {
            isValid = false;
            //std::cout<< "Edge config is in collision with an obstacle"<< std::endl;

            for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); it != collision_result.contacts.end();  ++it)
            {
              ROS_INFO_STREAM("Contact between: " <<it->first.first.c_str()<<" and" <<it->first.second.c_str());
            }


            //Return to function that called "isEdgeValid" if a colliding state has been detected
            break;
        }
        else
        {
            //Update index of last valid node along the edge
            last_valid_node_idx = edp;

            //std::cout<< "Last collision free config has index: "<<last_valid_node_idx<<std::endl;
        }
    }

    //Check whether edge is valid
    if (isValid == true)
    {
        //std::cout<< "Edge is collision-free"<< std::endl;
    }

    return isValid;
}


//Perform a step from a node towards another node
bool FeasibilityChecker::stepAlongEdge(Node start_node, Node &end_node, double extend_step_factor)
{
    //Flag to be returned
    bool rand_sample_reached = true;

    //Compute distance between configurations
    vector<double> eucl_dist(start_node.config.size());

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
             eucl_dist[joint_idx] = end_node.config[joint_idx] - start_node.config[joint_idx];

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
    vector<double> extend_config(start_node.config.size());

    //For computing distance between start_node config and extend config
    // (prismatic and revolute joint expansion treated seperately)
    sum_squares_rev_joints = 0.0;
    sum_squares_prism_joints = 0.0;


//    cout<<"abs"<<endl;

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

                     //Perform step from start_node config towards end_node (for prismatic joints)
                     double c_step = extend_step_factor * eucl_dist[joint_idx];
                     //cout<<c_step<<endl;
                     extend_config[joint_idx] = start_node.config[joint_idx] + c_step;

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

                     //Perform step from start_node config towards end_node (for revolute joints)
                     double c_step = extend_step_factor * eucl_dist[joint_idx];
                     //cout<<c_step<<endl;
                     extend_config[joint_idx] = start_node.config[joint_idx] + c_step;

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


    //Check whether revolute joints have reached the end node config (otherwise revolute joint values of current end node remain untouched)
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
                     end_node.config[joint_idx] = extend_config[joint_idx];
                 }

                 //Increment joint index counter
                 joint_idx++;
             }
        }

        //Set flag indicating whether end node config has been reached to false
        rand_sample_reached = false;
    }

    //Check whether prismatic joints have reached the end node config (otherwise prismatic joint values of current end node remain untouched)
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
                     end_node.config[joint_idx] = extend_config[joint_idx];
                 }

                 //Increment joint index counter
                 joint_idx++;
             }
        }

        //Set flag indicating whether end node config has been reached to false
        rand_sample_reached = false;
    }


//    //Modify config of end_node if initial config of end_node has not been reached
//    if(extend_vec_length_rot < vec_length_rot && extend_vec_length_prism < vec_length_prism)
//    {
//        //Set new config for end_node
//        end_node.config = extend_config;

//        //Set new ee pose for end_node
//        //KDL::JntArray ext_configuration = m_RobotMotionController->Vector_to_JntArray(extend_config);
        
//        //Convert Vector to JntArray
//        KDL::JntArray ext_configuration(extend_config.size());
//    	for(int i = 0 ; i < extend_config.size() ; i++)
//        	ext_configuration(i)= extend_config[i];
        
//        end_node.ee_pose = m_KDLRobotModel->compute_FK(m_manipulator_chain,ext_configuration);

//    }
//    else
//    {
//        //Configuration of end_node has been reached (no modification to end_node data performeds)
//        rand_sample_reached = true;
//    }



    return rand_sample_reached;

}


//Get the current transform map to base_link
bool FeasibilityChecker::update_map_to_robot_transform()
{
    //Get current pose of robot in the map frame
    tf::TransformListener listener;

    try {
        listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), m_transform_map_to_base);
        m_transform_map_to_base_available = true;
    } catch (tf::TransformException ex) {
        //ROS_ERROR("%s",ex.what());
        m_transform_map_to_base_available = false;
    }

    return m_transform_map_to_base_available;
}


//Reset class variables and data structures
bool FeasibilityChecker::reset_data()
{
    //Reset map to base transform availability
    m_transform_map_to_base_available = false;
}



} //end of namespace

