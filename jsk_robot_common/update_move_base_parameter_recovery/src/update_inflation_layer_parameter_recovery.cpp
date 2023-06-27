#include <update_move_base_parameter_recovery/update_inflation_layer_parameter_recovery.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(
    update_move_base_parameter_recovery::UpdateInflationLayerParameterRecovery,
    nav_core::RecoveryBehavior
)

namespace update_move_base_parameter_recovery
{
UpdateInflationLayerParameterRecovery::UpdateInflationLayerParameterRecovery():
    initialized_(false),
    store_original_(false),
    wait_for_deadline_(false),
    spin_thread_(std::thread(&UpdateInflationLayerParameterRecovery::spinRestore, this))
{
}

void UpdateInflationLayerParameterRecovery::initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("parameter_name", parameter_name_, std::string(""));
        ptr_dynamic_param_client_ = std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>>(new dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>(parameter_name_));

        double duration_deadline;
        private_nh.param("duration_deadline", duration_deadline, 10.0);
        duration_deadline_ = ros::Duration(duration_deadline);
        private_nh.param("timeout_duration", timeout_duration_, 5.0);

        if (private_nh.hasParam("enabled")) {
            valid_enabled_ = true;
            private_nh.getParam("enabled", enabled_);
        } else {
            valid_enabled_ = false;
        }
        if (private_nh.hasParam("cost_scaling_factor")) {
            valid_cost_scaling_factor_ = true;
            private_nh.getParam("cost_scaling_factor", cost_scaling_factor_);
        } else {
            valid_cost_scaling_factor_ = false;
        }
        if (private_nh.hasParam("inflation_radius")) {
            valid_inflation_radius_ = true;
            private_nh.getParam("inflation_radius", inflation_radius_);
        } else {
            valid_inflation_radius_ = false;
        }
        if (private_nh.hasParam("inflate_unknown")) {
            valid_inflate_unknown_ = true;
            private_nh.getParam("inflate_unknown", inflate_unknown_);
        } else {
            valid_inflate_unknown_ = false;
        }

        inflation_config_ = costmap_2d::InflationPluginConfig::__getDefault__();
        ROS_INFO_STREAM("Initialize a plugin \"" << name << "\"");
        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

UpdateInflationLayerParameterRecovery::~UpdateInflationLayerParameterRecovery()
{
}

void UpdateInflationLayerParameterRecovery::runBehavior()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (not initialized_)
    {
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
    }

    bool success = ptr_dynamic_param_client_->getCurrentConfiguration(inflation_config_, ros::Duration(timeout_duration_));
    if ( success ) {
        if ( not store_original_ ) {
            original_inflation_config_ = inflation_config_;
            store_original_ = true;
        }
        if ( valid_enabled_ ) inflation_config_.enabled = enabled_;
        if ( valid_cost_scaling_factor_ ) inflation_config_.cost_scaling_factor = cost_scaling_factor_;
        if ( valid_inflation_radius_ ) inflation_config_.inflation_radius = inflation_radius_;
        if ( valid_inflate_unknown_ ) inflation_config_.inflate_unknown = inflate_unknown_;
        success = ptr_dynamic_param_client_->setConfiguration(inflation_config_);
        if ( success ) {
            ROS_INFO_STREAM("Update parameter of \"" << parameter_name_ << "\".");
            ROS_INFO_STREAM("  enabled: " << inflation_config_.enabled);
            ROS_INFO_STREAM("  cost_scaling_factor: " << inflation_config_.cost_scaling_factor);
            ROS_INFO_STREAM("  inflation_radius: " << inflation_config_.inflation_radius);
            ROS_INFO_STREAM("  inflate_unknown: " << inflation_config_.inflate_unknown);
            deadline_ = ros::Time::now() + duration_deadline_;
            wait_for_deadline_ = true;
        } else {
            ROS_ERROR("Failed to set Configuration.");
        }
    } else {
        ROS_ERROR("Failed to get Current Configuration.");
    }
}

void UpdateInflationLayerParameterRecovery::spinRestore()
{
    ros::Rate rate(1);
    while (ros::ok()) {
        rate.sleep();
        std::lock_guard<std::mutex> lock(mtx_);
        if ( store_original_ && ros::Time::now() > deadline_ && wait_for_deadline_ ) {
            ptr_dynamic_param_client_->setConfiguration(original_inflation_config_);
            ROS_WARN("Restored original config.");
            wait_for_deadline_ = false;
        }
    }
}

};
