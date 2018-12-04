#!/usr/bin/env roseus
(ros::roseus "sample2")
(require :pepper-interface "package://peppereus/pepper-interface.l")
(pepper-init nil "robot2")
(unix:sleep 10)
(send *ri* :speak "Hello Pepper!")
