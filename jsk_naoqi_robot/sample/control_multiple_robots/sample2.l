#!/usr/bin/env roseus
(ros::roseus "sample2")
(require :nao-interface "package://naoeus/nao-interface.l")
(nao-init nil "robot2")
(unix:sleep 10)
(send *ri* :speak "Hello Pepper!")
