#!/usr/bin/env roseus

(ros::roseus-add-srvs "jsk_robot_startup")

(ros::roseus "tweet_server")
(load "package://jsk_robot_startup/lifelog/tweet_client.l")


(defun service-cb (req)
  (let* ((image-topic-name (send req :image_topic_name))
         (with-image
           (if (send req :image) (if (eq (length image-topic-name) 0) t image-topic-name)))
         (warningp (send req :warning))
         (warning-time (if warningp (send req :warning_time)))
         (speak (send req :speak))
         (tweetp (tweet-string (send req :text) :warning-time warning-time :with-image with-image :speak speak))
         (res (instance jsk_robot_startup::TweetResponse :init :success tweetp)))
    res))

(ros::advertise-service "tweet" jsk_robot_startup::Tweet #'service-cb)
(ros::rate 0.5)
(ros::spin)
