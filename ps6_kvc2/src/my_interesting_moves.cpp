// Copyright [2015] <Kristina Collins> [legal/copyright]
/// Kristina Collins, kvc2@case.edu
/// This node defines the motions in the library as a series of positions, and builds trajectories based on them.
/// Each position is described by a vector of length 7, in which each number represents a joint on Baxter's right arm from shoulder to wrist.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>
#include <cwru_action/trajAction.h>
#include <ps6_kvc2/my_interesting_moves.h>
#include <vector>

int g_count = 0;

// Class constructor:
InterestingMoves::InterestingMoves(ros::NodeHandle *nh)
{
    nh_ = *nh;
}

void InterestingMoves::hello_i_am_baymax()
{
    Vectorq7x1 q_pose;
        q_pose <<0, 0, 3.14, 0.9, 1, 0, 0;
    build_traj(q_pose);
        q_pose <<0, 0, 3.14, 2, -2, 0, 0;
    build_traj(q_pose);
        q_pose <<0, 0, 3.14, 0.9, -2, 0, 0;
    build_traj(q_pose);
    ROS_INFO("Hello! I am Baymax, your personal healthcare companion.");
}

void InterestingMoves::fistbump()
{
    Vectorq7x1 q_pose;
    q_pose << 0.5, 0, .5, -3.14, 0, 1, 1;
    build_traj(q_pose);
    q_pose << 1.5, 0, .5, -3.14, 0, 1, 1;
    build_traj(q_pose);
    q_pose << 1, 0, .5, -3.14, 0, 1, 1;
    build_traj(q_pose);
    q_pose << 1, 0, .5, -3.14, 0, 1, 1;
    build_traj(q_pose);
    q_pose << 1, 0, 0, 0, 0, 0, 1;
    build_traj(q_pose);
    q_pose << 1, -.8, 0, 0, 0, -3.14, 0;
    build_traj(q_pose);
    ROS_INFO("Balalalalala.");
}

void InterestingMoves::salute()
{
    Vectorq7x1 q_pose;
    q_pose <<0, 0, 3.14, 0.9, 1, 0, 0;
    build_traj(q_pose);
    q_pose <<-1, 0, 0, 0, 0, 0, 0;
    build_traj(q_pose);
    q_pose <<1, -1, 0, 0, 0, 0, 0;
    build_traj(q_pose);
    q_pose <<1, 0, 0, 0, 0, 0, 0;
    build_traj(q_pose);
    q_pose <<1, -1, 1, 0, 0, 0, 0;
    build_traj(q_pose);
    q_pose <<1, -1, 0, 0, 0, 0, 0;
    build_traj(q_pose);
    q_pose <<1, -1, 1, 1, 0, 1, 0;
    build_traj(q_pose);
    q_pose <<1, -1, 1, 1, 0, 0, 0;
    ROS_INFO("Smoke me a kipper, I'll be back for breakfast.");
}

void InterestingMoves::build_traj(Vectorq7x1 position)
{
    Vectorq7x1 q_pose;
    q_pose << position;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory;
    Baxter_traj_streamer baxter_traj_streamer(&nh_);


    ROS_INFO("warming up callbacks...");
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // Find the current pose:
    ROS_INFO("getting current right arm pose");
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();
    ROS_INFO_STREAM("r_arm state: " << q_vec_right_arm.transpose());
    q_in_vecxd = q_vec_right_arm;
    des_path.push_back(q_in_vecxd);
    q_in_vecxd = q_pose;
    des_path.push_back(q_in_vecxd);

    cout << "stuffing traj: " << endl;
    baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory);
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
        cwru_action::trajGoal goal;
        // copy traj to goal:
        goal.trajectory = des_trajectory;
        actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0));  // wait for up to 5 seconds

        if (!server_exists)
        {
            ROS_WARN("could not connect to server; will wait forever");
            return;
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        action_client.sendGoal(goal);
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
}
