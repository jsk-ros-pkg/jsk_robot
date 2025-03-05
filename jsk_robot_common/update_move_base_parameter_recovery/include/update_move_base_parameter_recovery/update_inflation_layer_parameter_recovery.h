#ifndef UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#define UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/client.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <thread>
#include <mutex>

namespace update_move_base_parameter_recovery
{

class UpdateInflationLayerParameterRecovery : public nav_core::RecoveryBehavior
{
public:
    UpdateInflationLayerParameterRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~UpdateInflationLayerParameterRecovery();

private:
    bool initialized_;

    // members for dynamic parameters
    bool valid_enabled_;
    bool enabled_;
    bool valid_cost_scaling_factor_;
    double cost_scaling_factor_;
    bool valid_inflation_radius_;
    double inflation_radius_;
    bool valid_inflate_unknown_;
    bool inflate_unknown_;

    double timeout_duration_;
    std::string parameter_name_;
    std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>> ptr_dynamic_param_client_;
    costmap_2d::InflationPluginConfig inflation_config_;
    bool store_original_;
    costmap_2d::InflationPluginConfig original_inflation_config_;
    ros::Duration duration_deadline_;
    ros::Time deadline_;
    bool wait_for_deadline_;
    std::mutex mtx_;
    std::thread spin_thread_;

    void spinRestore();
};
};

#endif
