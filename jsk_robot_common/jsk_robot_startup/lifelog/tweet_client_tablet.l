#!/usr/bin/env roseus

(ros::roseus "twitter_client_tablet")
(ros::roseus-add-msgs "roseus")

(load "package://roseus/euslisp/dynamic-reconfigure-server.l")
(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(setq *volume* (ros::get-param "~volume" 1.0))
(setq *speak-enable* (ros::get-param "~speak_enable" t))
(setq *enable* (ros::get-param "~enable" t))
(setq *reconfigure-server*
      (def-dynamic-reconfigure-server
        ;;; ((name type level description (default) (min) (max) (edit_method)) ... )
        (("volume"  double_t  0  "tweet speak volume" 1.0  0.0 1.0)
         ("speak_enable"  bool_t  0  "tweet speak enable" t)
         ("enable"  bool_t  0  "tweet enable" t))
        ;; use lamda-closure to avoid memory error
        '(lambda-closure nil 0 0 (cfg level)
            (let ((prev-volume *volume*)
                  (prev-speak-enable *speak-enable*)
                  (prev-enable *enable*))
              (setq *volume* (cdr (assoc "volume" cfg :test #'equal)))
              (setq *speak-enable* (cdr (assoc "speak_enable" cfg :test #'equal)))
              (setq *enable* (cdr (assoc "enable" cfg :test #'equal)))
              (if (null (equal *volume* prev-volume))
                (ros::ros-warn "Volume changed to: ~A" *volume*))
              (if (null (equal *enable* prev-enable))
                (ros::ros-warn "Enable changed to: ~A" *enable*))
              (if (null (equal *speak-enable* prev-speak-enable))
                (ros::ros-warn "Speak enable changed to: ~A" *speak-enable*)))
            cfg)))

(defun twit-cb (msg)
  (if *enable*
    (let ((twit-str (send msg :data)))
      (tweet-string twit-str
                    :warning-time nil
                    :with-image "/tablet/marked/image_rect_color"
                    :speak *speak-enable*
                    :volume *volume*))))

(ros::advertise "/tweet" std_msgs::String 1)
(ros::advertise "/tweet_image_server/tweet/goal" rostwitter::TweetActionGoal 1)
(ros::subscribe "/pr2twit_from_tablet" roseus::StringStamped #'twit-cb)

(ros::spin)
