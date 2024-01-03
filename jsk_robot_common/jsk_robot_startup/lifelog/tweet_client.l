#!/usr/bin/env roseus

(ros::roseus-add-srvs "topic_tools")
(ros::roseus-add-msgs "rostwitter")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "rostwitter")

(load "package://pr2eus/speak.l")

(defun tweet-string (twit-str &key (warning-time) (with-image) (image-wait 30) (speak t)
                              (volume 1.0))
  (if (null (ros::get-num-subscribers "/tweet"))
    (ros::advertise "/tweet" std_msgs::String 1))
  (if (null (ros::get-num-subscribers "/tweet_image_server/tweet/goal"))
    (ros::advertise "/tweet_image_server/tweet/goal" rostwitter::TweetActionGoal 1))
  (let (prev-image-topic img)
  (when warning-time
    (unless (numberp warning-time)
      (setq warning-time 3))
    (when speak
      (speak-jp (format nil "~Aびょうまえ"
                        (case warning-time
                          (0 "ぜろ")
                          (1 "いち")
                          (2 "に")
                          (3 "さん")
                          (4 "よん")
                          (5 "ご")
                          (6 "ろく")
                          (7 "なな")
                          (8 "はち")
                          (9 "きゅう")
                          (10 "じゅう")
                          (t "じゅういじょう")))
                :volume volume))
    (unix::sleep warning-time))

  (cond
    (with-image
      (unix::system (format nil "rm -f /tmp/tweet_image.jpg"))
      ;; camera shot sound
      (when speak
        (play-sound (pathname (ros::resolve-ros-path "package://jsk_pr2_startup/jsk_pr2_lifelog/camera.wav"))
                    :topic-name "robotsound_jp" :wait t :volume volume))
      ;; specify camera
      (when (stringp with-image)
        (ros::wait-for-service "/tweet_image_mux/list")
        (let ((current-image-list
               (send (ros::service-call "/tweet_image_mux/list" (instance topic_tools::muxlistrequest :init)) :topics)))
          (unless (find with-image current-image-list :test #'string=)
            (ros::service-call "/tweet_image_mux/add" (instance topic_tools::muxaddrequest :init :topic with-image)))
          (setq prev-image-topic
                (send (ros::service-call "/tweet_image_mux/select" (instance topic_tools::muxselectrequest :init :topic with-image)) :prev_topic))))
      (unless prev-image-topic
        (setq prev-image-topic (send (one-shot-subscribe "/tweet_image_mux/selected" std_msgs::String) :data)))

      ;; retrieve image
      ;; (call-empty-service "/tweet_image_saver/save" :wait t)
      ;;rostwitter.msg.TweetActionGoal(goal=rostwitter.msg.TweetGoal(text=&quot;[Audible warning] &quot; + m.data, image=True, image_topic_name=&quot;/edgetpu_human_pose_estimator/output/image&quot;))'

      (ros::ros-info "tweeting ~A" twit-str)
      (ros::publish "/tweet_image_server/tweet/goal"
                    (instance rostwitter::TweetActionGoal :init
                              :goal
                              (instance rostwitter::TweetGoal :init
                                        :text twit-str
                                        :image t
                                        :image_topic_name "/tweet_image")))
      (ros::service-call "/tweet_image_mux/select" (instance topic_tools::muxselectrequest :init :topic prev-image-topic)))
    (t
      (ros::publish "/tweet" (instance std_msgs::String :init :data twit-str))))
  (when speak (speak-jp "ついーとしました" :wait t :volume volume))))
