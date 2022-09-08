#ifndef COMPLEX_RECOVERY_H
#define COMPLEX_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <actionlib/client/simple_action_client.h>

namespace complex_recovery
{

class ConcurrentComplexRecovery : public nav_core::RecoveryBehavior
{
public:
    ConcurrentComplexRecovery();
    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);
    void runBehavior();
    ~ConcurrentComplexRecovery();

private:
    bool initialized_;
};
};

#endif
