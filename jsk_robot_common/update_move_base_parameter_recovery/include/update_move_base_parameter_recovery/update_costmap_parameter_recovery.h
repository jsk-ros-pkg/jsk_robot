#ifndef UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#define UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/client.h>
#include <costmap_2d/Costmap2DConfig.h>

namespace update_move_base_parameter_recovery
{

class UpdateCostmapParameterRecovery : public nav_core::RecoveryBehavior
{
public:
    UpdateCostmapParameterRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~UpdateCostmapParameterRecovery();

private:
    bool initialized_;
    std::string parameter_name_;
    double timeout_duration_;
    double footprint_padding_;
    std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig>> ptr_dynamic_param_client_;
    costmap_2d::Costmap2DConfig costmap_config_;
};
};

#endif
