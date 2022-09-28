#include <update_move_base_parameter_recovery/update_costmap_parameter_recovery.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(
    update_move_base_parameter_recovery::UpdateCostmapParameterRecovery,
    nav_core::RecoveryBehavior
)

namespace update_move_base_parameter_recovery
{
UpdateCostmapParameterRecovery::UpdateCostmapParameterRecovery():
    initialized_(false),
    store_original_(false),
    wait_for_deadline_(false),
    spin_thread_(std::thread(&UpdateCostmapParameterRecovery::spinRestore, this))
{
}

void UpdateCostmapParameterRecovery::initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("parameter_name", parameter_name_, std::string(""));
        ptr_dynamic_param_client_ = \
            std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig>>(
                    new dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig>(parameter_name_)
                    );

        double duration_deadline;
        private_nh.param("duration_deadline", duration_deadline, 10.0);
        duration_deadline_ = ros::Duration(duration_deadline);
        private_nh.param("timeout_duration", timeout_duration_, 5.0);

        if (private_nh.hasParam("footprint")) {
            valid_footprint_ = true;
            private_nh.getParam("footprint", footprint_);
        } else {
            valid_footprint_ = false;
        }
        if (private_nh.hasParam("robot_radius")) {
            valid_robot_radius_ = true;
            private_nh.getParam("robot_radius", robot_radius_);
        } else {
            valid_robot_radius_ = false;
        }
        if (private_nh.hasParam("footprint_padding")) {
            valid_footprint_padding_ = true;
            private_nh.getParam("footprint_padding", footprint_padding_);
        } else {
            valid_footprint_padding_ = false;
        }

        costmap_config_ = costmap_2d::Costmap2DConfig::__getDefault__();
        ROS_INFO_STREAM("Initialize a plugin \"" << name << "\"");
        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

UpdateCostmapParameterRecovery::~UpdateCostmapParameterRecovery()
{
}

void UpdateCostmapParameterRecovery::runBehavior()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (not initialized_)
    {
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
    }

    bool success = ptr_dynamic_param_client_->getCurrentConfiguration(costmap_config_, ros::Duration(timeout_duration_));
    if ( success ) {
        if ( not store_original_ ) {
            original_costmap_config_ = costmap_config_;
            store_original_ = true;
        }
        if ( valid_footprint_ ) costmap_config_.footprint = footprint_;
        if ( valid_robot_radius_ ) costmap_config_.robot_radius = robot_radius_;
        if ( valid_footprint_padding_ ) costmap_config_.footprint_padding = footprint_padding_;
        success = ptr_dynamic_param_client_->setConfiguration(costmap_config_);
        if ( success ) {
            ROS_INFO_STREAM("Update parameter of \"" << parameter_name_ << "\".");
            ROS_INFO_STREAM("  footprint: " << costmap_config_.footprint);
            ROS_INFO_STREAM("  robot_radius: " << costmap_config_.robot_radius);
            ROS_INFO_STREAM("  footprint_padding: " << costmap_config_.footprint_padding);
            deadline_ = ros::Time::now() + duration_deadline_;
            wait_for_deadline_ = true;
        } else {
            ROS_ERROR("Failed to set Configuration.");
        }
    } else {
        ROS_ERROR("Failed to get Current Configuration.");
    }
}

void UpdateCostmapParameterRecovery::spinRestore()
{
    ros::Rate rate(1);
    while (ros::ok()) {
        rate.sleep();
        std::lock_guard<std::mutex> lock(mtx_);
        if ( store_original_ && ros::Time::now() > deadline_ && wait_for_deadline_ ) {
            ptr_dynamic_param_client_->setConfiguration(original_costmap_config_);
            ROS_WARN("Restored original config.");
            wait_for_deadline_ = false;
        }
    }
}

};
