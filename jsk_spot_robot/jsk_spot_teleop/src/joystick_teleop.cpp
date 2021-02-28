#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <spot_msgs/SetLocomotion.h>

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
        ros::param::param<int>("~button_locomotion_mode", button_locomotion_mode_, -1);

        ros::param::param<int>("~num_buttons", num_buttons_, 0);
        num_buttons_ = num_buttons_ < 0 ? 0 : num_buttons_;
        pressed_.resize(num_buttons_);
        for ( auto index: pressed_ ) {
            index = false;
        }

        client_estop_hard_ = nh.serviceClient<std_srvs::Trigger>("/spot/estop/hard");
        client_estop_gentle_ = nh.serviceClient<std_srvs::Trigger>("/spot/estop/gentle");
        client_power_off_ = nh.serviceClient<std_srvs::Trigger>("/spot/power_off");
        client_power_on_ = nh.serviceClient<std_srvs::Trigger>("/spot/power_on");
        client_self_right_ = nh.serviceClient<std_srvs::Trigger>("/spot/self_right");
        client_sit_ = nh.serviceClient<std_srvs::Trigger>("/spot/sit");
        client_stand_ = nh.serviceClient<std_srvs::Trigger>("/spot/stand");
        client_stop_ = nh.serviceClient<std_srvs::Trigger>("/spot/stop");
        client_release_ = nh.serviceClient<std_srvs::Trigger>("/spot/release");
        client_claim_ = nh.serviceClient<std_srvs::Trigger>("/spot/claim");
        client_stair_mode_ = nh.serviceClient<std_srvs::SetBool>("/spot/stair_mode");
        client_locomotion_mode_ = nh.serviceClient<spot_msgs::SetLocomotion>("/spot/locomotion_mode");

        sub_joy_ = nh.subscribe<sensor_msgs::Joy, TeleopManager>("joy", 1, &TeleopManager::callbackJoy, this);

        req_next_stair_mode_.data = true;
        req_next_locomotion_mode_.locomotion_mode = 4;
    }

    void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        std_srvs::Trigger srv;

        // estop_hard
        if ( button_estop_hard_ >= 0
                and button_estop_hard_ < msg->buttons.size()
                and button_estop_hard_ < num_buttons_ ) {
            if ( msg->buttons[button_estop_hard_] == 1 ) {
                if ( not pressed_[button_estop_hard_] ) {
                    if ( client_estop_hard_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'estop_hard' succeeded.");
                    } else {
                        ROS_ERROR("Service 'estop_hard' failed.");
                    }
                    pressed_[button_estop_hard_] = true;
                }
            } else {
                if ( pressed_[button_estop_hard_] ) {
                    pressed_[button_estop_hard_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'estop_hard' is disabled.");
        }

        // estop_gentle
        if ( button_estop_gentle_ >= 0
                and button_estop_gentle_ < msg->buttons.size()
                and button_estop_gentle_ < num_buttons_ ) {
            if ( msg->buttons[button_estop_gentle_] == 1 ) {
                if ( not pressed_[button_estop_gentle_] ) {
                    if ( client_estop_gentle_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'estop_gentle' succeeded.");
                    } else {
                        ROS_ERROR("Service 'estop_gentle' failed.");
                    }
                    pressed_[button_estop_gentle_] = true;
                }
            } else {
                if ( pressed_[button_estop_gentle_] ) {
                    pressed_[button_estop_gentle_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'estop_gentle' is disabled.");
        }

        // power_off
        if ( button_power_off_ >= 0
                and button_power_off_ < msg->buttons.size()
                and button_power_off_ < num_buttons_ ) {
            if ( msg->buttons[button_power_off_] == 1 ) {
                if ( not pressed_[button_power_off_] ) {
                    if ( client_power_off_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'power_off' succeeded.");
                    } else {
                        ROS_ERROR("Service 'power_off' failed.");
                    }
                    pressed_[button_power_off_] = true;
                }
            } else {
                if ( pressed_[button_power_off_] ) {
                    pressed_[button_power_off_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'power_off' is disabled.");
        }

        // power_on
        if ( button_power_on_ >= 0
                and button_power_on_ < msg->buttons.size()
                and button_power_on_ < num_buttons_ ) {
            if ( msg->buttons[button_power_on_] == 1 ) {
                if ( not pressed_[button_power_on_] ) {
                    if ( client_power_on_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'power_on' succeeded.");
                    } else {
                        ROS_ERROR("Service 'power_on' failed.");
                    }
                    pressed_[button_power_on_] = true;
                }
            } else {
                if ( pressed_[button_power_on_] ) {
                    pressed_[button_power_on_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'power_on' is disabled.");
        }

        // self_right
        if ( button_self_right_ >= 0
                and button_self_right_ < msg->buttons.size()
                and button_self_right_ < num_buttons_ ) {
            if ( msg->buttons[button_self_right_] == 1 ) {
                if ( not pressed_[button_self_right_] ) {
                    if ( client_self_right_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'self_right' succeeded.");
                    } else {
                        ROS_ERROR("Service 'self_right' failed.");
                    }
                    pressed_[button_self_right_] = true;
                }
            } else {
                if ( pressed_[button_self_right_] ) {
                    pressed_[button_self_right_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'self_right' is disabled.");
        }

        // sit
        if ( button_sit_ >= 0
                and button_sit_ < msg->buttons.size()
                and button_sit_ < num_buttons_ ) {
            if ( msg->buttons[button_sit_] == 1 ) {
                if ( not pressed_[button_sit_] ) {
                    if ( client_sit_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'sit' succeeded.");
                    } else {
                        ROS_ERROR("Service 'sit' failed.");
                    }
                    pressed_[button_sit_] = true;
                }
            } else {
                if ( pressed_[button_sit_] ) {
                    pressed_[button_sit_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'sit' is disabled.");
        }

        // stand
        if ( button_stand_ >= 0
                and button_stand_ < msg->buttons.size()
                and button_stand_ < num_buttons_ ) {
            if ( msg->buttons[button_stand_] == 1 ) {
                if ( not pressed_[button_stand_] ) {
                    if ( client_stand_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'stand' succeeded.");
                    } else {
                        ROS_ERROR("Service 'stand' failed.");
                    }
                    pressed_[button_stand_] = true;
                }
            } else {
                if ( pressed_[button_stand_] ) {
                    pressed_[button_stand_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'stand' is disabled.");
        }

        // stop
        if ( button_stop_ >= 0
                and button_stop_ < msg->buttons.size()
                and button_stop_ < num_buttons_ ) {
            if ( msg->buttons[button_stop_] == 1 ) {
                if ( not pressed_[button_stop_] ) {
                    if ( client_stop_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'stop' succeeded.");
                    } else {
                        ROS_ERROR("Service 'stop' failed.");
                    }
                    pressed_[button_stop_] = true;
                }
            } else {
                if ( pressed_[button_stop_] ) {
                    pressed_[button_stop_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'stop' is disabled.");
        }

        // release
        if ( button_release_ >= 0
                and button_release_ < msg->buttons.size()
                and button_release_ < num_buttons_ ) {
            if ( msg->buttons[button_release_] == 1 ) {
                if ( not pressed_[button_release_] ) {
                    if ( client_release_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'release' succeeded.");
                    } else {
                        ROS_ERROR("Service 'release' failed.");
                    }
                    pressed_[button_release_] = true;
                }
            } else {
                if ( pressed_[button_release_] ) {
                    pressed_[button_release_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'release' is disabled.");
        }

        // claim
        if ( button_claim_ >= 0
                and button_claim_ < msg->buttons.size()
                and button_claim_ < num_buttons_ ) {
            if ( msg->buttons[button_claim_] == 1 ) {
                if ( not pressed_[button_claim_] ) {
                    if ( client_claim_.call(srv) && srv.response.success ) {
                        ROS_INFO("Service 'claim' succeeded.");
                    } else {
                        ROS_ERROR("Service 'claim' failed.");
                    }
                    pressed_[button_claim_] = true;
                }
            } else {
                if ( pressed_[button_claim_] ) {
                    pressed_[button_claim_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'claim' is disabled.");
        }

        // stair_mode
        if ( button_stair_mode_ >= 0
                and button_stair_mode_ < msg->buttons.size()
                and button_stair_mode_ < num_buttons_ ) {
            if ( msg->buttons[button_stair_mode_] == 1 ) {
                if ( not pressed_[button_stair_mode_] ) {
                    std_srvs::SetBool::Response res;
                    if ( client_stair_mode_.call(req_next_stair_mode_,res) && res.success ) {
                        ROS_INFO_STREAM("Service 'stair_mode' succeeded. set to " << req_next_stair_mode_.data);
                        req_next_stair_mode_.data = not req_next_stair_mode_.data;
                    } else {
                        ROS_ERROR("Service 'stair_mode' failed.");
                    }
                    pressed_[button_stair_mode_] = true;
                }
            } else {
                if ( pressed_[button_stair_mode_] ) {
                    pressed_[button_stair_mode_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'stair_mode' is disabled.");
        }

        // locomotion_mode
        if ( button_locomotion_mode_ >= 0
                and button_locomotion_mode_ < msg->buttons.size()
                and button_locomotion_mode_ < num_buttons_ ) {
            if ( msg->buttons[button_locomotion_mode_] == 1 ) {
                if ( not pressed_[button_locomotion_mode_] ) {
                    std_srvs::SetBool::Response res;
                    if ( client_locomotion_mode_.call(req_next_locomotion_mode_,res) && res.success ) {
                        ROS_INFO_STREAM("Service 'locomotion_mode' succeeded. set to " << req_next_locomotion_mode_.locomotion_mode);
                        switch (req_next_locomotion_mode_.locomotion_mode) {
                            case 1:
                                req_next_locomotion_mode_.locomotion_mode = 4;
                            default:
                                req_next_locomotion_mode_.locomotion_mode = 1;
                        }
                    } else {
                        ROS_ERROR("Service 'locomotion_mode' failed.");
                    }
                    pressed_[button_locomotion_mode_] = true;
                }
            } else {
                if ( pressed_[button_locomotion_mode_] ) {
                    pressed_[button_locomotion_mode_] = false;
                }
            }
        } else {
            ROS_DEBUG("Button 'locomotion_mode' is disabled.");
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

    ros::Subscriber sub_joy_;

    int num_buttons_;
    std::vector<bool> pressed_;

    //
    std_srvs::SetBool::Request req_next_stair_mode_;
    spot_msgs::SetLocomotion::Request req_next_locomotion_mode_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "spot_teleop");
    ros::NodeHandle nh;

    TeleopManager teleop;
    teleop.init(nh);

    ros::spin();
}
