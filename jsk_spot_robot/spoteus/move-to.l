(load "spot-interface.l")

(spot-init nil) ;; do not create-viewer

(setq path "/home/lokada/work/2020-11-18 203458 73B1.walk" )
;; use fedical start to goal, -1 means goal
(send *ri* :navigate-to path -1 :initial-localization-fiducial t :initial-localization-waypoint 0)
;; do not use fedical but start 73b2 to 73a2
(send *ri* :navigate-to path 7 :initial-localization-fiducial t :initial-localization-waypoint 0)
;; back to 73b2
(send *ri* :navigate-to path 0 :initial-localization-fiducial nil :initial-localization-waypoint 7)
;; go to 73a2 again, you do not need fedical
(send *ri* :navigate-to path 7 :initial-localization-fiducial nil :initial-localization-waypoint 0)
;; and got to follow original route
(send *ri* :navigate-to path -1 :initial-localization-fiducial nil :initial-localization-waypoint 7)
(ros::ros-info "done")

(send *ri* :sit)(unix::sleep 3) ;; need to fix...
(send *ri* :power-off)(unix::sleep 3) ;; need to fix
(send *ri* :release)
