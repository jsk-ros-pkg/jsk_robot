#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>

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
        ros::param::param<int>("~button_release", button_release_, -1);
        ros::param::param<int>("~button_claim", button_claim_, -1);

        client_estop_hard_ = nh.serviceClient<std_srvs::Trigger>("/spot/estop/hard");
        client_estop_gentle_ = nh.serviceClient<std_srvs::Trigger>("/spot/estop/gentle");
        client_power_off_ = nh.serviceClient<std_srvs::Trigger>("/spot/power_off");
        client_power_on_ = nh.serviceClient<std_srvs::Trigger>("/spot/power_on");
        client_self_right_ = nh.serviceClient<std_srvs::Trigger>("/spot/self_right");
        client_sit_ = nh.serviceClient<std_srvs::Trigger>("/spot/sit");
        client_stand_ = nh.serviceClient<std_srvs::Trigger>("/spot/stand");
        client_release_ = nh.serviceClient<std_srvs::Trigger>("/spot/release");
        client_claim_ = nh.serviceClient<std_srvs::Trigger>("/spot/claim");

        sub_joy_ = nh.subscribe<sensor_msgs::Joy, TeleopManager>("joy", 1, &TeleopManager::callbackJoy, this);
    }

    void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        std_srvs::Trigger srv;

        // estop_hard
        if ( ( button_estop_hard_ >= 0 and button_estop_hard_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_estop_hard_] == 1 ) {
                if ( client_estop_hard_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'estop_hard' succeeded.");
                } else {
                    ROS_ERROR("Service 'estop_hard' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'estop_hard' is disabled.");
        }

        // estop_gentle
        if ( ( button_estop_gentle_ >= 0 and button_estop_gentle_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_estop_gentle_] == 1 ) {
                if ( client_estop_gentle_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'estop_gentle' succeeded.");
                } else {
                    ROS_ERROR("Service 'estop_gentle' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'estop_gentle' is disabled.");
        }

        // power_off
        if ( ( button_power_off_ >= 0 and button_power_off_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_power_off_] == 1 ) {
                if ( client_power_off_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'power_off' succeeded.");
                } else {
                    ROS_ERROR("Service 'power_off' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'power_off' is disabled.");
        }

        // power_on
        if ( ( button_power_on_ >= 0 and button_power_on_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_power_on_] == 1 ) {
                if ( client_power_on_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'power_on' succeeded.");
                } else {
                    ROS_ERROR("Service 'power_on' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'power_on' is disabled.");
        }

        // self_right
        if ( ( button_self_right_ >= 0 and button_self_right_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_self_right_] == 1 ) {
                if ( client_self_right_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'self_right' succeeded.");
                } else {
                    ROS_ERROR("Service 'self_right' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'self_right' is disabled.");
        }

        // sit
        if ( ( button_sit_ >= 0 and button_sit_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_sit_] == 1 ) {
                if ( client_sit_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'sit' succeeded.");
                } else {
                    ROS_ERROR("Service 'sit' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'sit' is disabled.");
        }

        // stand
        if ( ( button_stand_ >= 0 and button_stand_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_stand_] == 1 ) {
                if ( client_stand_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'stand' succeeded.");
                } else {
                    ROS_ERROR("Service 'stand' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'stand' is disabled.");
        }

        // release
        if ( ( button_release_ >= 0 and button_release_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_release_] == 1 ) {
                if ( client_release_.call(srv) && srv.response.success ) {
                    ROS_INFO("Service 'release' succeeded.");
                } else {
                    ROS_ERROR("Service 'release' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'release' is disabled.");
        }

        // claim
        if ( ( button_claim_ >= 0 and button_claim_ < msg->buttons.size() ) ) {
            if ( msg->buttons[button_claim_] == 1 ) {
                if ( client_claim_.call(srv) ) {
                    ROS_INFO("Service 'claim' succeeded.");
                } else {
                    ROS_ERROR("Service 'claim' failed.");
                }
                return;
            }
        } else {
            ROS_DEBUG("Button 'claim' is disabled.");
        }
    }

private:
    int button_sit_;
    int button_stand_;
    int button_self_right_;
    int button_claim_;
    int button_release_;
    int button_power_on_;
    int button_power_off_;
    int button_estop_gentle_;
    int button_estop_hard_;

    ros::ServiceClient client_sit_;
    ros::ServiceClient client_stand_;
    ros::ServiceClient client_self_right_;
    ros::ServiceClient client_claim_;
    ros::ServiceClient client_release_;
    ros::ServiceClient client_power_on_;
    ros::ServiceClient client_power_off_;
    ros::ServiceClient client_estop_gentle_;
    ros::ServiceClient client_estop_hard_;

    ros::Subscriber sub_joy_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "spot_teleop");
    ros::NodeHandle nh;

    TeleopManager teleop;
    teleop.init(nh);

    ros::spin();
}
