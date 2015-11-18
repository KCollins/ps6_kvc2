// "Copyright [2015] <Kristina Collins>"  [legal/copyright]
/// Kristina Collins, kvc2@case.edu.
/// Executes the 3 moves from the library in ps6_kvc2. 
#include <ros/ros.h>
#include <ps6_kvc2/my_interesting_moves.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ps6_kvc2");
    ros::NodeHandle nh;

    InterestingMoves im(&nh);
    im.hello_i_am_baymax();
    im.fistbump();
    im.salute();
    return 0;
}
