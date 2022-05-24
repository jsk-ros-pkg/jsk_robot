#!/usr/bin/env roseus

(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(ros::roseus "sample-email-topic-client")
(init-mail)
(ros::spin-once)
(setq receiver-address (ros::get-param "~receiver_address"))
(setq attached-files (ros::get-param "~attached_files" nil))
(unix::sleep 10)  ;; wait for server

(ros::ros-info "Sending a mail to ~A" receiver-address)
(send-mail "test-mail" receiver-address "test" :attached-files attached-files)
(ros::ros-info "Sent a mail")
(ros::roseus "shutdown")
(exit)
