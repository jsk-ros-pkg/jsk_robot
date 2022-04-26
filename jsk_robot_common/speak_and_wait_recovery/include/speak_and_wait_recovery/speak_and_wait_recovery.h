#ifndef SPEAK_AND_WAIT_RECOVERY_H
#define SPEAK_AND_WAIT_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/SoundRequest.h>
#include <sound_play/SoundRequestAction.h>

namespace speak_and_wait_recovery
{

class SpeakAndWaitRecovery : public nav_core::RecoveryBehavior
{
public:
    SpeakAndWaitRecovery();

    void initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap);

    void runBehavior();

    ~SpeakAndWaitRecovery();

private:
    void say(std::string text);

    std::shared_ptr<actionlib::SimpleActionClient<sound_play::SoundRequestAction>> ptr_action_client_;
    std::string speak_text_;
    double duration_wait_;
    double duration_timeout_;
    double result_timeout_;
    bool initialized_;
};
};

#endif
