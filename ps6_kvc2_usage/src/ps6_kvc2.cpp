 /** ... text ...*/

#include <ros/ros.h>
#include <ps6_kvc2/my_interesting_moves.h>

int main(int argc, char** argv)
{
 /**
 * ... text ...
 */
    ros::init(argc, argv, "ps6_kvc2");
    ros::NodeHandle nh;


///
/// ... text ...
///
    InterestingMoves im(&nh);
    im.hello_i_am_baymax();
    im.fistbump();
    im.salute();
    return 0;
}
