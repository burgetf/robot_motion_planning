#include <planning_heuristics/distance_heuristics.h>

namespace heuristics_motion_planning
{


//Constructor
DistanceHeuristics::DistanceHeuristics()
{
    //Nothing to do yet
}

//Destructor
DistanceHeuristics::~DistanceHeuristics()
{
    //Nothing to do yet
}


 //Compute Euclidean Distance between two poses
double DistanceHeuristics::euclidean_pose_distance(vector<double> pose_start, vector<double> pose_end)
{
    //Euclidean distance to be returned
    double eucl_dist = 0.0;

    //Scaling factor for orientation error
    // -> in order to make orientation error [in rad] comparable to cartesian error [in m]
    // -> or to give less weight to orientation error
    double orientation_error_weight = 4.0;

    //Check if both vectors have the same size
    if(pose_start.size() != pose_end.size())
        ROS_ERROR("Different number of elements in pose_start and pose_end vector!");


    //Distance Vector between two points
    vector<double> distance_vec(pose_start.size());

    //Sum of the error square
    double sum_error_sq = 0.0;

    //Consider position and orientation difference
    for (int i = 0 ; i < pose_start.size() ; i++)
    {
        //If an element of the end/start pose vector is constrained, i.e. there is a specific value for it (a value of 1000.0 means vector element is unconstrained)

        //Position error
        if (i < 3 && pose_end[i] != 1000.0 && pose_start[i] != 1000.0)
        {
            distance_vec[i] = pose_end[i] - pose_start[i];
            sum_error_sq += (distance_vec[i] * distance_vec[i]);
        }
        //Orientation error (consider only scalar part of quaternion = amount of rotation)
        if (i == (pose_start.size()-1) && pose_end[i] != 1000.0 && pose_start[i] != 1000.0)
        {
            distance_vec[i] = (pose_end[i] - pose_start[i]) / orientation_error_weight;
            sum_error_sq += (distance_vec[i] * distance_vec[i]);
        }

    }

    //Length of error vector
    eucl_dist = sqrt(sum_error_sq);

    //cout<<"Distance: "<<eucl_dist<<endl;

    //Return euclidean distance
	return eucl_dist;
}


//Compute Euclidean Distance between two point positions
double DistanceHeuristics::euclidean_cartesian_distance(vector<double> pose_start, vector<double> pose_end)
{
    //Euclidean distance to be returned
    double eucl_dist = 0.0;

    //Check if both vectors have the same size
    if(pose_start.size() != pose_end.size())
        ROS_ERROR("Different number of elements in pose_start and pose_end vector!");

    //Only cartesian distance between points considered
    double diff_x = pose_end[0] - pose_start[0];
    double diff_y = pose_end[1] - pose_start[1];
    double diff_z = pose_end[2] - pose_start[2];
    //Length of distance vector
    eucl_dist = sqrt((diff_x*diff_x) + (diff_y*diff_y) + (diff_z*diff_z));

    //Return euclidean distance
    return eucl_dist;
}



//Compute Euclidean Distance between two base configurations
double DistanceHeuristics::euclidean_base_distance(vector<double> conf_start, vector<double> conf_end)
{
    //Euclidean distance to be returned
    double eucl_dist = 0.0;

    //Check if both vectors have at least three elements (base x,y,theta)
    if(conf_start.size() < 3 || conf_end.size() < 3)
        ROS_ERROR("Base configurations must have at least three elements!");


    //Distance Vector between two configurations
    vector<double> distance_vec(3);

    //Sum of the error square
    double sum_error_sq = 0.0;

    //Consider base config distance (x,y,theta)
    for (int i = 0 ; i < 3 ; i++)
    {
        distance_vec[i] = conf_end[i] - conf_start[i];
        sum_error_sq += (distance_vec[i] * distance_vec[i]);
    }

    //Length of error vector
    eucl_dist = sqrt(sum_error_sq);

    //Return euclidean distance
    return eucl_dist;
}


//Compute Euclidean Distance between two configurations
double DistanceHeuristics::euclidean_joint_space_distance(vector<double> conf_start, vector<double> conf_end)
{
    //Euclidean distance to be returned
    double eucl_dist = 0.0;

    //Check if both vectors have the same size
    if(conf_start.size() != conf_end.size())
        ROS_ERROR("Different number of elements in conf_start and conf_end vector!");


    //Distance Vector between two configurations
    vector<double> distance_vec(conf_start.size());

    //Sum of the error square
    double sum_error_sq = 0.0;

    //Consider joint space difference
    for (int i = 0 ; i < conf_start.size() ; i++)
    {
        distance_vec[i] = conf_end[i] - conf_start[i];
        sum_error_sq += (distance_vec[i] * distance_vec[i]);
    }

    //Length of error vector
    eucl_dist = sqrt(sum_error_sq);

    //Return euclidean distance
    return eucl_dist;
}


//Compute Euclidean Distance between two configurations (+ distance for revolute and prismatic joints)
vector<double> DistanceHeuristics::euclidean_joint_space_distance(KDL::Chain kin_chain, vector<double> conf_start, vector<double> conf_end)
{
    //Euclidean distance to be returned
    vector<double> eucl_dist(3);
    eucl_dist[0] = 0.0; //total c-Space distance
    eucl_dist[1] = 0.0; //c-Space distance for revolute joints
    eucl_dist[2] = 0.0; //c-Space distance for prismatic joints


    //Check if both vectors have the same size
    if(conf_start.size() != conf_end.size())
        ROS_ERROR("Different number of elements in conf_start and conf_end vector!");


    //Distance Vector between two configurations
    vector<double> distance_vec(conf_start.size());

    //Sum of the error square
    double sum_error_sq = 0.0;
    double sum_error_sq_rev = 0.0;
    double sum_error_sq_prism = 0.0;

    //Sum of squares for all, revolute and prismatic joints
    int joint_idx = 0;
    for (int k = 0 ; k < kin_chain.getNrOfSegments(); k++)
      {
          if(kin_chain.getSegment(k).getJoint().getTypeName() != "None")
          {
              distance_vec[joint_idx] = conf_end[joint_idx] - conf_start[joint_idx];
              sum_error_sq += (distance_vec[joint_idx] * distance_vec[joint_idx]);

              if(kin_chain.getSegment(k).getJoint().getTypeName() == "RotAxis")
                  sum_error_sq_rev += (distance_vec[joint_idx] * distance_vec[joint_idx]);
              else
                  sum_error_sq_prism += (distance_vec[joint_idx] * distance_vec[joint_idx]);

              //Next joint
              joint_idx++;
          }
     }


//    //Consider joint space difference
//    for (int i = 0 ; i < conf_start.size() ; i++)
//    {
//        distance_vec[i] = conf_end[i] - conf_start[i];
//        sum_error_sq += (distance_vec[i] * distance_vec[i]);
//    }


    //Norm of distance vector
    eucl_dist[0] = sqrt(sum_error_sq);  //total distance
    eucl_dist[1] = sqrt(sum_error_sq_rev);  //distance for revolute joints
    eucl_dist[2] = sqrt(sum_error_sq_prism);  //distance for prismatic joints

    //Return euclidean distance
    return eucl_dist;
}




}



