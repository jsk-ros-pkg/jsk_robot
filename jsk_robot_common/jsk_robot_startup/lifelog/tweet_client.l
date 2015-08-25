#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "topic_tools")
(load "package://pr2eus/speak.l")

(ros::roseus "tweet_client")
(ros::advertise "/tweet" std_msgs::String 1)

;; next tweet timing ( x(i+1) = x(i) * 2, 5 * 60 = 300 sec)
(while (not (ros::has-param "/active_user/elapsed_time"))
  (unix::sleep 3)
  (ros::ros-info "Wait for /active_user/elapsed_time parameter ..."))

(cond
 ((ros::has-param "/active_user/tweet_second")
  (setq *tweet-second* (ros::get-param "/active_user/tweet_second")))
 (t
  (setq *tweet-second* 300)
  ))

(setq *target-second* (+ (ros::get-param "/active_user/elapsed_time")
                         *tweet-second*))

(setq *waking-tweet-second* 3600.0)
(cond
 ((ros::has-param "/active_user/start_time")
  (let ((st (ros::get-param "/active_user/start_time")))
    (setq *waking-target-second*
          (+ (- (send (ros::time-now) :to-sec) st)
             *waking-tweet-second*))))
 (t
  (setq *waking-target-second* *waking-tweet-second*)))

(defun tweet-string (twit-str &key (warning-time) (with-image) (image-wait 30) (speek t))
  (let (prev-image-topic)
  (when warning-time
    (unless (numberp warning-time)
      (setq warning-time 3))
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
                        (t "じゅういじょう"))))
    (unix::sleep warning-time))

  (when with-image
    (unix::system (format nil "rm -f /tmp/tweet_image.jpg")))

  ;; camera shot sound
  (unless (ros::get-topic-publisher "robotsound_jp")
    (ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
    (unix:sleep 1))
  (ros::publish "robotsound_jp" 
                (instance sound_play::SoundRequest :init
                          :sound sound_play::SoundRequest::*play_file*
                          :command sound_play::SoundRequest::*play_once*
                          :arg (ros::resolve-ros-path "package://jsk_pr2_startup/jsk_pr2_lifelog/camera.wav")))

  ;; specify camera
  (when (stringp with-image)
    (ros::wait-for-service "/tweet_image_mux/list")
    (let ((current-image-list
           (send (ros::service-call "/tweet_image_mux/list" (instance topic_tools::muxlistrequest :init)) :topics)))
      (unless (find with-image current-image-list :test #'string=)
        (ros::service-call "/tweet_image_mux/add" (instance topic_tools::muxaddrequest :init :topic with-image)))
      (setq prev-image-topic
            (send (ros::service-call "/tweet_image_mux/select" (instance topic_tools::muxselectrequest :init :topic with-image)) :prev_topic))))

  ;; retrieve image
  (call-empty-service "/tweet_image_saver/save" :wait t)
  (when prev-image-topic
    (ros::service-call "/tweet_image_mux/select" (instance topic_tools::muxselectrequest :init :topic prev-image-topic)))

    (ros::ros-info "tweeting ~A" twit-str)
    (cond
     (with-image
      (ros::publish "/tweet"
                    (instance std_msgs::String :init
                              :data (format nil "~A ~A"
                                            twit-str (format nil "/tmp/tweet_image.jpg")))))
     (t
      (ros::publish "/tweet" (instance std_msgs::String :init :data twit-str))
      ))
    (when speek (speak-jp "ついーとしました"))
    )

