// "Copyright [year] <Copyright Owner>"  [legal/copyright]
 /**
@file
@brief A library of moves for Baxter. This file just contains declarations. 
 */

#ifndef PS6_KVC2_MY_INTERESTING_MOVES_H
#define PS6_KVC2_MY_INTERESTING_MOVES_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

class InterestingMoves
{
public:
        ros::NodeHandle nh_;
        int g_count;
 /**
Declarations for each maneuver, the trajectories of which are in the .src file.
 */
        explicit InterestingMoves(ros::NodeHandle *nh);
        void hello_i_am_baymax();
        void fistbump();
        void salute();

private:
	 /**
Declaration for the function that constructs trajectories from position vectors.
 */
        void build_traj(Vectorq7x1 position);
};

#endif  // PS6_KVC2_MY_INTERESTING_MOVES_H
