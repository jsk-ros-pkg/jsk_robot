#include <complex_recovery/sequential_complex_recovery.h>
#include <complex_recovery/utils.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(complex_recovery::SequentialComplexRecovery, nav_core::RecoveryBehavior)

namespace complex_recovery
{

SequentialComplexRecovery::SequentialComplexRecovery():
    initialized_(false),
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior")
{
}

void SequentialComplexRecovery::initialize(
            std::string name,
            tf2_ros::Buffer* tf_buffer,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        bool success = loadRecoveryBehaviors(
                    name,
                    private_nh,
                    recovery_loader_,
                    recovery_behaviors_,
                    recovery_behavior_names_,
                    tf_buffer,
                    global_costmap,
                    local_costmap
                    );
        if ( not success ) {
            ROS_ERROR("Failed to load behaviors.");
        } else {
            ROS_INFO("Behaviors are loaded.");
            for (auto behavior_name = recovery_behavior_names_.begin(); behavior_name != recovery_behavior_names_.end(); behavior_name++) {
                ROS_INFO("behavior: %s", behavior_name->c_str());
            }
        }

        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

SequentialComplexRecovery::~SequentialComplexRecovery()
{
  recovery_behaviors_.clear();
}

void SequentialComplexRecovery::runBehavior()
{
  if (not initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ROS_INFO("Start executing behaviors sequentially.");
  for (auto index = 0; index < recovery_behaviors_.size(); index++) {
      ROS_INFO("executing behavior %s", recovery_behavior_names_[index].c_str());
      recovery_behaviors_[index]->runBehavior();
  }

  ROS_INFO("All behaviors have finished.");
}

};
