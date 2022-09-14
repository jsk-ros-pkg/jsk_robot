#include <ros/ros.h>
#include "update_move_base_parameter_recovery/update_costmap_parameter_recovery.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_costmap_parameter_recovery_test_node");
    ros::NodeHandle nh;

    update_move_base_parameter_recovery::UpdateCostmapParameterRecovery recovery;
    recovery.initialize(std::string("recovery"),NULL,NULL,NULL);

    ros::Rate rate(1);
    while (ros::ok()) {
        ROS_WARN("Run a behavior.");
        recovery.runBehavior();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
