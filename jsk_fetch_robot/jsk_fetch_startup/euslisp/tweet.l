#!/usr/bin/env roseus

(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(tweet-string "I took a photo." :warning-time 3 :with-image t :speak t)
(exit)
