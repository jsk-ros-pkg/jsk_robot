#!/usr/bin/env roseus
;; tweet.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :tweet-client "package://jsk_robot_startup/lifelog/tweet_client.l")

(ros::roseus "run_tweet")

(speak-jp "写真をとります")
(unix:sleep 2)
(speak-jp "さん")
(unix:sleep 1)
(speak-jp "に")
(unix:sleep 1)
(speak-jp "いち")
(unix:sleep 1)
(speak-jp "カシャ！")

(tweet-string "Hello!!"
              :warning-time nil
              :with-image "/kinect_head/rgb/image_rect_color")
(exit)
