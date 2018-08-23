(ros::roseus "go_to_kitchen")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(defun go-to-kitchen ()
  ;; go to kitchen
  (send *ri* :speak "go to kitchen.")
  (go-to-spot "/eng2/7f/room73B2-sink-front1" (make-coords :pos #f(100 -1000 0)))
  (unix:sleep 1)
  (send *ri* :speak "arrived at kitchen." :wait t)
  (unix:sleep 1)
  (send *ri* :speak "return to dock." :wait t)
  ;; auto docking
  (auto-dock))

(go-to-kitchen)
