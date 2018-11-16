;; go-dock.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "fetch_go_dock")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(auto-dock)
(exit)
