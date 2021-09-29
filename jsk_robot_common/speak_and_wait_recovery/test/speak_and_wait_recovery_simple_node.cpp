#include <ros/ros.h>
#include "speak_and_wait_recovery/speak_and_wait_recovery.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speak_and_wait_recovery_simple_node");

    speak_and_wait_recovery::SpeakAndWaitRecovery recovery;
    recovery.initialize(std::string("recovery"),NULL,NULL,NULL);

    ros::Rate rate(10);
    while (ros::ok()) {
        recovery.runBehavior();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
