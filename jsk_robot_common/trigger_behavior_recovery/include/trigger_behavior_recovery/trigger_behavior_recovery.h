#ifndef trigger_behavior_recovery_H
#define trigger_behavior_recovery_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/SoundRequest.h>
#include <sound_play/SoundRequestAction.h>

namespace trigger_behavior_recovery
{

class TriggerBehaviorRecovery : public nav_core::RecoveryBehavior
{
public:
    TriggerBehaviorRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~TriggerBehaviorRecovery();

private:
    void trigger();

    double duration_timeout_;
    double result_timeout_;
    bool initialized_;
};
};

#endif
