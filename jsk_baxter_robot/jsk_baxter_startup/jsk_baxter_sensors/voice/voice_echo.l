#!/usr/bin/env roseus
(ros::roseus "voice_echo")
(ros::load-ros-manifest "std_msgs")

(defvar *text2wave-jp-path*
  (format nil "~A/jsk_baxter_sensors/voice/text2wave-jp" (read-line (piped-fork "rospack find jsk_baxter_startup"))))
(defvar *text2wave-path*
  (format nil "~A/text2wave-jp.sh" (read-line (piped-fork "rospack find jtalk"))))


(defun mei-voice-message-callback
  (msg)
  (print "aaa")
  (print msg)
  (unix:system
   (format nil "~A ~A happy /tmp/tmp.wav && aplay /tmp/tmp.wav"
	   *text2wave-jp-path*
	   (send msg :data))
   ))

(ros::subscribe
 "/voice_echo/mei"
 std_msgs::string
 #'mei-voice-message-callback 1)

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )
