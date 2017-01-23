/*
 * distance_heuristics.h
 *
 *  Created on: July 06, 2015
 *      Author: Felix Burget
 */


// --- Includes -- 
#include <ros/ros.h>
#include <kdl/chain.hpp>


#ifndef DISTANCE_HEURISTICS_H
#define DISTANCE_HEURISTICS_H

// --Namespaces --
using namespace std;


namespace heuristics_motion_planning{


class DistanceHeuristics
{
	public:
	DistanceHeuristics();
	~DistanceHeuristics();
	
    //Compute Euclidean Distance between two poses
    double euclidean_pose_distance(vector<double> pose_start, vector<double> pose_end);

    //Compute Euclidean Distance between two point positions
    double euclidean_cartesian_distance(vector<double> pose_start, vector<double> pose_end);

    //Compute Euclidean Distance between two configurations
    double euclidean_joint_space_distance(vector<double> conf_start, vector<double> conf_end);

    //Compute Euclidean Distance between two configurations (+ distance for revolute and prismatic joints)
    vector<double> euclidean_joint_space_distance(KDL::Chain kin_chain, vector<double> conf_start, vector<double> conf_end);

    //Compute Euclidean Distance between two base configurations
    double euclidean_base_distance(vector<double> conf_start, vector<double> conf_end);


		
	private:


};


}//end of namespace

#endif // DISTANCE_HEURISTICS_H


