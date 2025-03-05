#ifndef COMPLEX_RECOVERY_H
#define COMPLEX_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <actionlib/client/simple_action_client.h>

namespace complex_recovery
{

class ParallelComplexRecovery : public nav_core::RecoveryBehavior
{
public:
    ParallelComplexRecovery();
    void initialize(
            std::string name,
            tf2_ros::Buffer* tf_buffer,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);
    void runBehavior();
    ~ParallelComplexRecovery();

private:
    bool initialized_;
    std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
    std::vector<std::string> recovery_behavior_names_;
    pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
};
};

#endif
