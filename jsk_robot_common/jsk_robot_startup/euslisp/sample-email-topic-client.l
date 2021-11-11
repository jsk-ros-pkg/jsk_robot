#!/usr/bin/env roseus

(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(ros::roseus "sample-email-topic-client")

(setq receiver-address (ros::get-param "~receiver_address"))
(ros::ros-info "Sending a mail to ~A" receiver-address)

(unix::sleep 10)

(send-mail "test-mail" receiver-address "test")
(ros::ros-info "Sent a mail")
(unix::sleep 5)

(ros::roseus "shutdown")
(exit)
