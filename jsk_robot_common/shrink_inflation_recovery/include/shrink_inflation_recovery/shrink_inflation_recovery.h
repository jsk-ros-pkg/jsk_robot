#ifndef SHRINK_INFLATION_RECOVERY_H
#define SHRINK_INFLATION_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/client.h>
#include <costmap_2d/InflationPluginConfig.h>

namespace shrink_inflation_recovery
{

class ShrinkInflationRecovery : public nav_core::RecoveryBehavior
{
public:
    ShrinkInflationRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~ShrinkInflationRecovery();

private:
    bool initialized_;

    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> dynamic_param_client_;
    costmap_2d::InflationPluginConfig inflation_config_;
};
};

#endif
