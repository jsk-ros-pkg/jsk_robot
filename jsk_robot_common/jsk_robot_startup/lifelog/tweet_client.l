#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "topic_tools")
(require :speak "package://pr2eus/speak.l")

(ros::roseus "tweet_client")

(defun tweet-string (twit-str
                     &key (with-speech t)
                          (with-image t)
                          (tweet-topic "/tweet")
                          (wait-for-camera 5))
  (when (and with-image
             (ros::wait-for-service "/tweet_image_saver/save" 1)
             (ros::wait-for-service "/tweet_image_mux/list" 1))
    (unix::system (format nil "rm -f /tmp/tweet_image.jpg"))
    (when with-image
      (setq wait-for-camera (floor wait-for-camera))
      (when (and with-speech (> wait-for-camera 0))
        (speak-jp (format nil "~A秒前" wait-for-camera)
                  :wait nil))
      (unix::sleep wait-for-camera)
      ;; specify camera
      (when (stringp with-image)
        (let ((image-topics (send (ros::service-call "/tweet_image_mux/list"
                                                     (instance topic_tools::MuxListRequest :init))
                                  :topics))
              (selected-topic (send (one-shot-subscribe "/tweet_image_mux/selected" std_msgs::String) :data))
              prev-image-topic)
          (unless (find with-image image-topics :test #'string=)
            (ros::service-call "/tweet_image_mux/add"
                               (instance topic_tools::MuxAddRequest :init :topic with-image)))
          (unless (string= with-image selected-topic)
            (setq prev-iamge-topic
                  (send (ros::service-call "/tweet_image_mux/select"
                                           (instance topic_tools::MuxSelectRequest :init :topic with-image))
                        :prev_topic)))))
      ;; camera shot sound
      (when with-speech
        (play-sound
         (pathname (ros::resolve-ros-path "package://jsk_pr2_startup/jsk_pr2_lifelog/camera.wav"))
         :wait t))
      ;; retrieve image
      (call-empty-service "/tweet_image_saver/save" :wait t)))

  (ros::ros-info "tweeting ~A" twit-str)
  (unless (ros::get-topic-publisher tweet-topic)
    (ros::advertise tweet-topic std_msgs::String 1)
    (unix:sleep 1))
  (ros::publish tweet-topic
                (instance std_msgs::String :init
                          :data (if with-image
                                    (format nil "~A /tmp/tweet_image.jpg" twit-str)
                                    twit-str)))

  (when with-speech (speak-jp "ついーとしました" :wait t))
  t)

(provide :tweet_client)
