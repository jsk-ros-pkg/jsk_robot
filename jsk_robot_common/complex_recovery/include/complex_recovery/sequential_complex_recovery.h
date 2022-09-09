#ifndef COMPLEX_RECOVERY_H
#define COMPLEX_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <pluginlib/class_loader.hpp>

namespace complex_recovery
{

class SequentialComplexRecovery : public nav_core::RecoveryBehavior
{
public:
    SequentialComplexRecovery();
    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);
    void runBehavior();
    ~SequentialComplexRecovery();

private:
    bool initialized_;
    std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
    std::vector<std::string> recovery_behavior_names_;
    pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
};
};

#endif
