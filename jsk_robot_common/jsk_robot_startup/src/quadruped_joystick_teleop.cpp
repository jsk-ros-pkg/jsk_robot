#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <sound_play/SoundRequest.h>

class TeleopManager
{
public:
    void init(ros::NodeHandle& nh)
    {
        ros::param::param<int>("~button_estop_hard", button_estop_hard_, -1);
        ros::param::param<int>("~button_estop_gentle", button_estop_gentle_, -1);
        ros::param::param<int>("~button_power_off", button_power_off_, -1);
        ros::param::param<int>("~button_power_on", button_power_on_, -1);
        ros::param::param<int>("~button_self_right", button_self_right_, -1);
        ros::param::param<int>("~button_sit", button_sit_, -1);
        ros::param::param<int>("~button_stand", button_stand_, -1);
        ros::param::param<int>("~button_stop", button_stop_, -1);
        ros::param::param<int>("~button_release", button_release_, -1);
        ros::param::param<int>("~button_claim", button_claim_, -1);
        ros::param::param<int>("~button_stair_mode", button_stair_mode_, -1);
        ros::param::param<int>("~axe_dock", axe_dock_, -1);
        ros::param::param<int>("~dock_id", dock_id_, -1);

        ros::param::param<int>("~num_buttons", num_buttons_, 0);
        num_buttons_ = num_buttons_ < 0 ? 0 : num_buttons_;
        pressed_.resize(num_buttons_);
        for ( auto index: pressed_ ) {
            index = false;
        }
        ros::param::param<int>("~num_axes", num_axes_, 0);
        num_axes_ = num_axes_ < 0 ? 0 : num_buttons_;
        pressed_axes_.resize(num_axes_);
        for ( auto index: pressed_axes_ ) {
            index = false;
        }

        pub_sound_play_ = nh.advertise<sound_play::SoundRequest>("/robotsound", 1);

        client_estop_hard_ = nh.serviceClient<std_srvs::Trigger>("estop/hard");
        client_estop_gentle_ = nh.serviceClient<std_srvs::Trigger>("estop/gentle");
        client_power_off_ = nh.serviceClient<std_srvs::Trigger>("power_off");
        client_power_on_ = nh.serviceClient<std_srvs::Trigger>("power_on");
        client_self_right_ = nh.serviceClient<std_srvs::Trigger>("self_right");
        client_sit_ = nh.serviceClient<std_srvs::Trigger>("sit");
        client_stand_ = nh.serviceClient<std_srvs::Trigger>("stand");
        client_stop_ = nh.serviceClient<std_srvs::Trigger>("stop");
        client_release_ = nh.serviceClient<std_srvs::Trigger>("release");
        client_claim_ = nh.serviceClient<std_srvs::Trigger>("claim");
        client_stair_mode_ = nh.serviceClient<std_srvs::SetBool>("stair_mode");
        client_undock_ = nh.serviceClient<std_srvs::Trigger>("undock");

        sub_joy_ = nh.subscribe<sensor_msgs::Joy, TeleopManager>("joy", 1, &TeleopManager::callbackJoy, this);
        ROS_DEBUG_STREAM("Subscribe : " << sub_joy_.getTopic());

        req_next_stair_mode_.data = true;
    }

    void say(std::string message)
    {
        sound_play::SoundRequest msg;
        msg.sound = sound_play::SoundRequest::SAY;
        msg.command = sound_play::SoundRequest::PLAY_ONCE;
        msg.volume = 1.0;
        msg.arg = message;
        pub_sound_play_.publish(msg);
    }

    void say(const char *str)
    {
        say(std::string(str));
    }

    void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        std_srvs::Trigger srv;

        ROS_DEBUG_STREAM("Received : " << *msg);
        // estop_hard
        if ( button_estop_hard_ >= 0
                and button_estop_hard_ < msg->buttons.size()
                and button_estop_hard_ < num_buttons_ ) {
            if ( msg->buttons[button_estop_hard_] == 1 ) {
                if ( not pressed_[button_estop_hard_] ) {
                    this->say("estop hard calling");
                    if ( client_estop_hard_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_estop_hard_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_estop_hard_.getService() << " failed.");
                    }
                    pressed_[button_estop_hard_] = true;
                }
            } else {
                if ( pressed_[button_estop_hard_] ) {
                    pressed_[button_estop_hard_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_estop_hard_.getService() << " is disabled.");
        }

        // estop_gentle
        if ( button_estop_gentle_ >= 0
                and button_estop_gentle_ < msg->buttons.size()
                and button_estop_gentle_ < num_buttons_ ) {
            if ( msg->buttons[button_estop_gentle_] == 1 ) {
                if ( not pressed_[button_estop_gentle_] ) {
                    this->say("estop gentle calling");
                    if ( client_estop_gentle_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_estop_gentle_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_estop_gentle_.getService() << " failed.");
                    }
                    pressed_[button_estop_gentle_] = true;
                }
            } else {
                if ( pressed_[button_estop_gentle_] ) {
                    pressed_[button_estop_gentle_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_estop_gentle_.getService() << " is disabled.");
        }

        // power_off
        if ( button_power_off_ >= 0
                and button_power_off_ < msg->buttons.size()
                and button_power_off_ < num_buttons_ ) {
            if ( msg->buttons[button_power_off_] == 1 ) {
                if ( not pressed_[button_power_off_] ) {
                    this->say("power off calling");
                    if ( client_power_off_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_power_off_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_power_off_.getService() << " failed.");
                    }
                    pressed_[button_power_off_] = true;
                }
            } else {
                if ( pressed_[button_power_off_] ) {
                    pressed_[button_power_off_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_power_off_.getService() << " is disabled.");
        }

        // power_on
        if ( button_power_on_ >= 0
                and button_power_on_ < msg->buttons.size()
                and button_power_on_ < num_buttons_ ) {
            if ( msg->buttons[button_power_on_] == 1 ) {
                if ( not pressed_[button_power_on_] ) {
                    this->say("power on calling");
                    if ( client_power_on_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_power_on_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_power_on_.getService() << " failed.");
                    }
                    pressed_[button_power_on_] = true;
                }
            } else {
                if ( pressed_[button_power_on_] ) {
                    pressed_[button_power_on_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_power_on_.getService() << " is disabled.");
        }

        // self_right
        if ( button_self_right_ >= 0
                and button_self_right_ < msg->buttons.size()
                and button_self_right_ < num_buttons_ ) {
            if ( msg->buttons[button_self_right_] == 1 ) {
                if ( not pressed_[button_self_right_] ) {
                    this->say("self right calling");
                    if ( client_self_right_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_self_right_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_self_right_.getService() << " failed.");
                    }
                    pressed_[button_self_right_] = true;
                }
            } else {
                if ( pressed_[button_self_right_] ) {
                    pressed_[button_self_right_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_self_right_.getService() << " is disabled.");
        }

        // sit
        if ( button_sit_ >= 0
                and button_sit_ < msg->buttons.size()
                and button_sit_ < num_buttons_ ) {
            if ( msg->buttons[button_sit_] == 1 ) {
                if ( not pressed_[button_sit_] ) {
                    this->say("sit calling");
                    if ( client_sit_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_sit_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_sit_.getService() << " failed.");
                    }
                    pressed_[button_sit_] = true;
                }
            } else {
                if ( pressed_[button_sit_] ) {
                    pressed_[button_sit_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_sit_.getService() << " is disabled.");
        }

        // stand
        if ( button_stand_ >= 0
                and button_stand_ < msg->buttons.size()
                and button_stand_ < num_buttons_ ) {
            if ( msg->buttons[button_stand_] == 1 ) {
                if ( not pressed_[button_stand_] ) {
                    this->say("stand calling");
                    if ( client_stand_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_stand_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_stand_.getService() << " failed.");
                    }
                    pressed_[button_stand_] = true;
                }
            } else {
                if ( pressed_[button_stand_] ) {
                    pressed_[button_stand_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_stand_.getService() << " is disabled.");
        }

        // stop
        if ( button_stop_ >= 0
                and button_stop_ < msg->buttons.size()
                and button_stop_ < num_buttons_ ) {
            if ( msg->buttons[button_stop_] == 1 ) {
                if ( not pressed_[button_stop_] ) {
                    this->say("stop calling");
                    if ( client_stop_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_stop_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_stop_.getService() << " failed.");
                    }
                    pressed_[button_stop_] = true;
                }
            } else {
                if ( pressed_[button_stop_] ) {
                    pressed_[button_stop_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_stop_.getService() << " is disabled.");
        }

        // release
        if ( button_release_ >= 0
                and button_release_ < msg->buttons.size()
                and button_release_ < num_buttons_ ) {
            if ( msg->buttons[button_release_] == 1 ) {
                if ( not pressed_[button_release_] ) {
                    this->say("release calling");
                    if ( client_release_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_release_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_release_.getService() << " failed.");
                    }
                    pressed_[button_release_] = true;
                }
            } else {
                if ( pressed_[button_release_] ) {
                    pressed_[button_release_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_release_.getService() << " is disabled.");
        }

        // claim
        if ( button_claim_ >= 0
                and button_claim_ < msg->buttons.size()
                and button_claim_ < num_buttons_ ) {
            if ( msg->buttons[button_claim_] == 1 ) {
                if ( not pressed_[button_claim_] ) {
                    this->say("claim calling");
                    if ( client_claim_.call(srv) && srv.response.success ) {
                        ROS_INFO_STREAM("Service " << client_claim_.getService() << " succeeded.");
                    } else {
                        ROS_ERROR_STREAM("Service " << client_claim_.getService() << " failed.");
                    }
                    pressed_[button_claim_] = true;
                }
            } else {
                if ( pressed_[button_claim_] ) {
                    pressed_[button_claim_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_claim_.getService() << " is disabled.");
        }

        // stair_mode
        if ( button_stair_mode_ >= 0
                and button_stair_mode_ < msg->buttons.size()
                and button_stair_mode_ < num_buttons_ ) {
            if ( msg->buttons[button_stair_mode_] == 1 ) {
                if ( not pressed_[button_stair_mode_] ) {
                    if ( req_next_stair_mode_.data ) {
                        this->say("stair mode on");
                    } else {
                        this->say("stair mode off");
                    }
                    std_srvs::SetBool::Response res;
                    if ( client_stair_mode_.call(req_next_stair_mode_,res) && res.success ) {
                        ROS_INFO_STREAM("Service " << client_stair_mode_.getService() << " succeeded. set to " << req_next_stair_mode_.data);
                        req_next_stair_mode_.data = not req_next_stair_mode_.data;
                    } else {
                        ROS_ERROR_STREAM("Service " << client_stair_mode_.getService() << " failed.");
                    }
                    pressed_[button_stair_mode_] = true;
                }
            } else {
                if ( pressed_[button_stair_mode_] ) {
                    pressed_[button_stair_mode_] = false;
                }
            }
        } else {
            ROS_DEBUG_STREAM("Button " << client_stair_mode_.getService() << " is disabled.");
        }
    }

private:
    int button_estop_hard_;
    int button_estop_gentle_;
    int button_power_off_;
    int button_power_on_;
    int button_self_right_;
    int button_sit_;
    int button_stand_;
    int button_stop_;
    int button_release_;
    int button_claim_;
    int button_stair_mode_;
    int button_locomotion_mode_;
    int axe_dock_;
    int dock_id_;

    ros::ServiceClient client_estop_hard_;
    ros::ServiceClient client_estop_gentle_;
    ros::ServiceClient client_power_off_;
    ros::ServiceClient client_power_on_;
    ros::ServiceClient client_self_right_;
    ros::ServiceClient client_sit_;
    ros::ServiceClient client_stand_;
    ros::ServiceClient client_stop_;
    ros::ServiceClient client_release_;
    ros::ServiceClient client_claim_;
    ros::ServiceClient client_stair_mode_;
    ros::ServiceClient client_locomotion_mode_;
    ros::ServiceClient client_dock_;
    ros::ServiceClient client_undock_;

    ros::Publisher pub_sound_play_;

    ros::Subscriber sub_joy_;

    int num_buttons_;
    std::vector<bool> pressed_;
    int num_axes_;
    std::vector<bool> pressed_axes_;

    //
    std_srvs::SetBool::Request req_next_stair_mode_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "quadruped_teleop");
    ros::NodeHandle nh;

    TeleopManager teleop;
    teleop.init(nh);

    ros::spin();
}
