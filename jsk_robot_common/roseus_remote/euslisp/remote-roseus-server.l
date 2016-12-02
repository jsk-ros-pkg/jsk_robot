#!/usr/bin/env roseus
;; remote-roseus-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "roseus_remote")
(ros::roseus "remote_roseus_server")


(ros::advertise "output" roseus_remote::RawCommand)

(defun command-cb (msg)
  (let ((cmd-str (send msg :data))
        result pub-msg
        (res-str (make-array 1024 :initial-element 0 :element-type :char)))
    (setq cmd-str (string-right-trim " \t\n\0" cmd-str))
    (setq result (eval (read-from-string cmd-str)))
    (replace res-str (format nil "~A" result) :start2 0)
    (setq pub-msg (instance roseus_remote::RawCommand :init
                            :data res-str))
    (when result
      (ros::publish "output" pub-msg))))

(ros::subscribe "input" roseus_remote::RawCommand #'command-cb)

(ros::rate 10)
(ros::spin)

(exit)
