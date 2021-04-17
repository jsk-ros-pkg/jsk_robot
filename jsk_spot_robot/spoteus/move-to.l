(load "package://spoteus/spot-interface.l")

(spot-init nil) ;; do not create-viewer

(setq *path* (ros::get-param "~path" "/home/spot/autowalk_files/2021-04-16_eng2_73b2_to_81c1_night.walk"))
(setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
(setq *upload* (ros::get-param "~upload" t))

(if *upload*  (send *ri* :upload-path *path*))

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
