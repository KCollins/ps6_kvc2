// "Copyright [2015] <Kristina Collins>"  [legal/copyright]

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

        explicit InterestingMoves(ros::NodeHandle *nh);
        void hello_i_am_baymax();
        void fistbump();
        void salute();

private:
        void build_traj(Vectorq7x1 position);
};

#endif  // PS6_KVC2_MY_INTERESTING_MOVES_H
