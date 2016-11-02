;;
;;
;;
(require :fetch "package://fetcheus/fetch.l")

(defmethod fetch-robot
  (:inverse-kinematics
   (target-coords &rest args &key link-list move-arm (use-torso t) move-target &allow-other-keys)
   (unless move-arm (setq move-arm :rarm))
   (unless move-target (setq move-target (send self :rarm :end-coords)))
   (unless link-list
     (setq link-list (send self :link-list (send move-target :parent)
                           (unless use-torso (car (send self :rarm))))))
   (send-super* :inverse-kinematics target-coords
                :move-target move-target
                :link-list link-list
                args)
   ))
