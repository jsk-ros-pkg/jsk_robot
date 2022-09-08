#ifndef UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#define UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/client.h>
#include <costmap_2d/InflationPluginConfig.h>

namespace update_move_base_parameter_recovery
{

class UpdateInflationLayerRecovery : public nav_core::RecoveryBehavior
{
public:
    UpdateInflationLayerRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~UpdateInflationLayerRecovery();

private:
    bool initialized_;
    double inflation_radius_;
    double timeout_duration_;
    std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>> ptr_dynamic_param_client_;
    costmap_2d::InflationPluginConfig inflation_config_;
};
};

#endif
