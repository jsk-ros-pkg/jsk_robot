#include <shrink_inflation_recovery/shrink_inflation_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <actionlib/client/simple_action_client.h>

PLUGINLIB_EXPORT_CLASS(shrink_inflation_recovery::ShrinkInflationRecovery, nav_core::RecoveryBehavior)

namespace shrink_inflation_recovery
{
ShrinkInflationRecovery::ShrinkInflationRecovery():
    initialized_(false)
{
}

void ShrinkInflationRecovery::initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        std::string parameter_name;
        private_nh.param("parameter_name", parameter_name, std::string("/move_base/local_costmap/inflation_radius"));
        ptr_dynamic_param_client_ = std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>>(new dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>(parameter_name));
        double inflation_radius;
        private_nh.param("inflation_radius", inflation_radius, 0.0);
        inflation_config_ = costmap_2d::InflationPluginConfig::__getDefault__();
        ptr_dynamic_param_client_->getCurrentConfiguration(inflation_config_);
        inflation_config_.inflation_radius = inflation_radius;
        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

ShrinkInflationRecovery::~ShrinkInflationRecovery()
{
}

void ShrinkInflationRecovery::runBehavior()
{
  if (not initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ptr_dynamic_param_client_->setConfiguration(inflation_config_);
}

};
