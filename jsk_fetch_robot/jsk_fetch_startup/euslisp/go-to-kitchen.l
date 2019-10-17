(ros::roseus "go_to_kitchen")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(defun go-to-kitchen ()
  ;; go to kitchen
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (send *ri* :speak "go to kitchen.")
  (if (go-to-spot "/eng2/7f/room73B2-sink-front1" (make-coords :pos #f(100 -1000 0)))
    (progn ;; succeeded to go to kitchen
      (unix:sleep 1)
      (send *ri* :speak "arrived at kitchen." :wait t)
      (unix:sleep 1)
      (send *ri* :speak "return to dock." :wait t)
      (auto-dock))
    (progn ;; failed to go to kitchen
      (send *ri* :speak "I could not go to kitchen, so return to dock." :wait t)
      (auto-dock))))


(go-to-kitchen)
