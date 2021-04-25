#!/usr/bin/env roseus

;;
;; This script is for demonstration of GraphNav interface with euslisp.
;; By default, it is assumed that Spot is at the entrance of 73B2 and headed to the AR marker on the door.
;;

(load "package://spoteus/spot-interface.l")

(spot-init nil) ;; do not create-viewer

(setq *path* (ros::get-param "~path" (format nil "~A/autowalk/eng2_73b2_to_81c1_night.walk" (ros::rospack-find "spoteus_demo"))))
(setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
(setq *upload* (ros::get-param "~upload" t))

;; Upload graphnav files to the robot.
(if *upload*  (send *ri* :upload-path *path*))

;; Localize the robot in the map
(ros::ros-info "initialize position with waypoint of ~A" *init-waypoint*)
(send *ri* :initial-localization-waypoint *init-waypoint*)
;; you can also use following command to initialize localization if you start from 73B2
;; (send *ri* :initial-localization-fiducial)
;; the difference is,  :initial-localization-waypoint can initialize with any waypoint, meaning you can start from arbitary wapoint,  e.g., -1 -> 0,  1 -> -1

(ros::ros-info "ready go to 81C1?")
(if (y-or-n-p) t (exit))

;; go to 81C1
(send *ri* :navigate-to -1)

(ros::ros-info "ready go back to 73B2?")
(if (y-or-n-p) t (exit))

;; go back to 73B2
(send *ri* :navigate-to 0)

(send *ri* :sit)
