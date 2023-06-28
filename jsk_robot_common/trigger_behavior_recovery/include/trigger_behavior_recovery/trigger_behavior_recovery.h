#ifndef trigger_behavior_recovery_H
#define trigger_behavior_recovery_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <actionlib/client/simple_action_client.h>
#include <trigger_behavior_msgs/TriggerBehaviorAction.h>
#include <string>
#include <memory>

namespace trigger_behavior_recovery
{

class TriggerBehaviorRecovery : public nav_core::RecoveryBehavior
{
public:
    TriggerBehaviorRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~TriggerBehaviorRecovery();

private:
    void trigger();

    std::shared_ptr<actionlib::SimpleActionClient<trigger_behavior_msgs::TriggerBehaviorAction>> ptr_action_client_;
    double duration_timeout_;
    double result_timeout_;
    std::string trigger_action_name_;
    bool initialized_;
};
};

#endif
