#!/usr/bin/env roseus

(ros::roseus-add-srvs "topic_tools")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")

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

(defun tweet-string (twit-str &key (warning-time) (with-image) (image-wait 30) (speak t))
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
                          (t "じゅういじょう")))))
    (unix::sleep warning-time))

  (cond
    (with-image
      (unix::system (format nil "rm -f /tmp/tweet_image.jpg"))
      ;; camera shot sound
      (play-sound (pathname (ros::resolve-ros-path "package://jsk_pr2_startup/jsk_pr2_lifelog/camera.wav"))
                  :topic-name "robotsound_jp" :wait t)
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
      (setq img (one-shot-subscribe "/tweet_image" sensor_msgs::Image :timeout 1000))
      (if img
        (progn
          (write-image-file "/tmp/tweet_image.jpg" (ros::sensor_msgs/Image->image img))
          (ros::ros-info "tweeting ~A" twit-str)
          (ros::publish "/tweet"
                        (instance std_msgs::String :init
                                  :data (format nil "~A ~A"
                                                twit-str (format nil "/tmp/tweet_image.jpg")))))
        (ros::ros-error "fail saving image and tweeting"))
      (ros::service-call "/tweet_image_mux/select" (instance topic_tools::muxselectrequest :init :topic prev-image-topic)))
    (t
      (ros::publish "/tweet" (instance std_msgs::String :init :data twit-str))))
  (when speak (speak-jp "ついーとしました" :wait t))))

