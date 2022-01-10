#include <speak_and_wait_recovery/speak_and_wait_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/SoundRequestAction.h>
#include <sound_play/SoundRequest.h>

PLUGINLIB_EXPORT_CLASS(speak_and_wait_recovery::SpeakAndWaitRecovery, nav_core::RecoveryBehavior)

namespace speak_and_wait_recovery
{
SpeakAndWaitRecovery::SpeakAndWaitRecovery(): initialized_(false)
{
}

void SpeakAndWaitRecovery::initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("speak_text", speak_text_, std::string("Make way, Please."));
        private_nh.param("duration_wait", duration_wait_, 5.0);
        private_nh.param("duration_timeout", duration_timeout_, 1.0);
        std::string sound_action_name;
        private_nh.param("sound_action", sound_action_name, std::string("sound_play"));
        ptr_action_client_ = std::shared_ptr<actionlib::SimpleActionClient<sound_play::SoundRequestAction>>(new actionlib::SimpleActionClient<sound_play::SoundRequestAction>(sound_action_name, true));
        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

SpeakAndWaitRecovery::~SpeakAndWaitRecovery()
{
}

void SpeakAndWaitRecovery::runBehavior()
{
  if (not initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  this->say(speak_text_);
  ros::Duration(duration_wait_).sleep();
}

void SpeakAndWaitRecovery::say(
        std::string text
        )
{
    sound_play::SoundRequestGoal goal;
    goal.sound_request.sound = sound_play::SoundRequest::SAY;
    goal.sound_request.volume = 1;
    goal.sound_request.command = sound_play::SoundRequest::PLAY_ONCE;
    goal.sound_request.arg = text; // speeched text
    goal.sound_request.arg2 = ""; // voice
    if ( ptr_action_client_->waitForServer(ros::Duration(duration_timeout_)) ) {
        ptr_action_client_->sendGoal(goal);
        ptr_action_client_->waitForResult();
    } else {
        ROS_ERROR("No sound_play server found.");
    }
}

};
