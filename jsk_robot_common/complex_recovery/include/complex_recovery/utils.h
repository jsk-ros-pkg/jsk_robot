#include <pluginlib/class_list_macros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <pluginlib/class_loader.hpp>

namespace complex_recovery
{


bool loadRecoveryBehaviors(
        std::string parent_name,
        ros::NodeHandle& node,
        pluginlib::ClassLoader<nav_core::RecoveryBehavior>& recovery_loader,
        std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors,
        std::vector<std::string> recovery_behavior_names,
        tf2_ros::Buffer* ptr_tf_buffer,
        costmap_2d::Costmap2DROS* ptr_global_costmap,
        costmap_2d::Costmap2DROS* ptr_local_costmap
        );
};
