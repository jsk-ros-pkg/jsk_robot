#ifndef UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#define UPDATE_MOVE_BASE_PARAMETER_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/client.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <thread>
#include <mutex>

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

    // members for dynamic parameters
    bool valid_footprint_;
    std::string footprint_;
    bool valid_robot_radius_;
    double robot_radius_;
    bool valid_footprint_padding_;
    double footprint_padding_;

    std::string parameter_name_;
    double timeout_duration_;
    std::shared_ptr<dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig>> ptr_dynamic_param_client_;
    costmap_2d::Costmap2DConfig costmap_config_;
    bool store_original_;
    costmap_2d::Costmap2DConfig original_costmap_config_;
    ros::Duration duration_deadline_;
    ros::Time deadline_;
    bool wait_for_deadline_;
    std::mutex mtx_;
    std::thread spin_thread_;

    void spinRestore();
};
};

#endif
