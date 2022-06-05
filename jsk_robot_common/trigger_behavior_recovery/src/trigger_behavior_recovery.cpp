#include <trigger_behavior_recovery/trigger_behavior_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <actionlib/client/simple_action_client.h>
#include <trigger_behavior_msgs/TriggerBehaviorAction.h>

PLUGINLIB_EXPORT_CLASS(trigger_behavior_recovery::TriggerBehaviorRecovery, nav_core::RecoveryBehavior)

namespace trigger_behavior_recovery
{
TriggerBehaviorRecovery::TriggerBehaviorRecovery(): initialized_(false)
{
}

void TriggerBehaviorRecovery::initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("duration_timeout", duration_timeout_, 1.0);
        private_nh.param("result_timeout", result_timeout_, 3.0);
        std::string trigger_action_name;
        private_nh.param("trigger_action", trigger_action_name, std::string("trigger_default_behavior"));
        ptr_action_client_ = std::shared_ptr<actionlib::SimpleActionClient<trigger_behavior_msgs::TriggerBehaviorAction>>(new actionlib::SimpleActionClient<trigger_behavior_msgs::TriggerBehaviorAction>(trigger_action_name, true));
        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

TriggerBehaviorRecovery::~TriggerBehaviorRecovery()
{
}

void TriggerBehaviorRecovery::runBehavior()
{
  if (not initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  this->trigger();
}

void TriggerBehaviorRecovery::trigger()
{
    trigger_behavior_msgs::TriggerBehaviorGoal goal;
    if ( not ptr_action_client_->waitForServer(ros::Duration(duration_timeout_)) ) {
        ROS_ERROR("Behavior action server is not responding.");
        return;
    }
    ptr_action_client_->sendGoal(goal);
    if ( not ptr_action_client_->waitForResult(ros::Duration(result_timeout_)) ) {
        ROS_ERROR("Behavior action server did not finish a behavior in a specified duration.");
        return;
    }
    auto result = ptr_action_client_->getResult();
    if ( result->success ) {
        ROS_INFO("Behavior succeeded. message:");
        return;
    } else {
        ROS_ERROR("Behavior failed. message:");
        return;
    }
}

};
