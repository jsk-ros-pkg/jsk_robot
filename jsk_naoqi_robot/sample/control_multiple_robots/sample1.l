#!/usr/bin/env roseus
(ros::roseus "sample1")
(require :pepper-interface "package://peppereus/pepper-interface.l")
(pepper-init nil "robot1")
(unix:sleep 10)
(send *ri* :speak "Hello NAO!")
