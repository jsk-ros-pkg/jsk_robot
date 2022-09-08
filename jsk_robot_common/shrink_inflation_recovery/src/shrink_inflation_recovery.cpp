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
        ROS_ERROR_STREAM("parameter_name: " << parameter_name);
        private_nh.param("inflation_radius", inflation_radius_, 0.0);
        private_nh.param("timeout_duration", timeout_duration_, 5.0);
        inflation_config_ = costmap_2d::InflationPluginConfig::__getDefault__();
        ROS_INFO_STREAM("Initialize a plugin \"" << name << "\"");
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

    bool success = ptr_dynamic_param_client_->getCurrentConfiguration(inflation_config_, ros::Duration(timeout_duration_));
    if ( success ) {
        inflation_config_.inflation_radius = inflation_radius_;
        ptr_dynamic_param_client_->setConfiguration(inflation_config_);
        ROS_INFO_STREAM("Shrinked inflation radius to " << inflation_radius_);
    } else {
        ROS_ERROR("Failed to get Current Configuration.");
    }
}

};
