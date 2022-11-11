#!/usr/bin/env roseus

(ros::roseus "twitter_client_tablet")
(ros::roseus-add-msgs "roseus")

(load "package://roseus/euslisp/dynamic-reconfigure-server.l")
(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(setq *volume* (ros::get-param "~volume" 1.0))
(setq *reconfigure-server*
      (def-dynamic-reconfigure-server
        ;;; ((name type level description (default) (min) (max) (edit_method)) ... )
        (("volume"  double_t  0  "tweet speak volume" 1.0  0.0 1.0))
        ;; use lamda-closure to avoid memory error
        '(lambda-closure nil 0 0 (cfg level)
            (setq *volume* (cdr (assoc "volume" cfg :test #'equal)))
            (ros::ros-warn "Volume changed to: ~A" *volume*)
            cfg)))

(defun twit-cb (msg)
  (let ((twit-str (send msg :data)))
    (tweet-string twit-str
                  :warning-time nil
                  :with-image "/tablet/marked/image_rect_color"
                  :volume *volume*)))

(ros::advertise "/tweet" std_msgs::String 1)
(ros::subscribe "/pr2twit_from_tablet" roseus::StringStamped #'twit-cb)

(ros::spin)
