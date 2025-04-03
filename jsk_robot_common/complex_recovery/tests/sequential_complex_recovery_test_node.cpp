#include <ros/ros.h>
#include "complex_recovery/sequential_complex_recovery.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "complex_recovery_simple_node");
    ros::NodeHandle nh;

    complex_recovery::SequentialComplexRecovery recovery;
    recovery.initialize(std::string("recovery"),NULL,NULL,NULL);

    ros::Rate rate(10);
    while (ros::ok()) {
        ROS_WARN("Spoken a test");
        recovery.runBehavior();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
