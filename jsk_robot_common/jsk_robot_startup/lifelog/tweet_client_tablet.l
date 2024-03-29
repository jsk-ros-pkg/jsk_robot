#!/usr/bin/env roseus

(ros::roseus "twitter_client_tablet")
(ros::roseus-add-msgs "roseus")

(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(defun twit-cb (msg)
  (let ((twit-str (send msg :data)))
    (tweet-string twit-str
                  :warning-time nil
                  :with-image "/tablet/marked/image_rect_color")))

(ros::advertise "/tweet" std_msgs::String 1)
(ros::subscribe "/pr2twit_from_tablet" roseus::StringStamped #'twit-cb)

(ros::spin)
