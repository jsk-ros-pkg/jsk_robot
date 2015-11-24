#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")

(ros::roseus "hand_interface")

(ros::wait-for-service "/aero_hand_controller")

(defmethod aero-upper-interface
  (:grasp (arm)
    (let (req
	  res
	  )
      (setq req (instance
		 aero_startup::AeroHandControllerRequest
		 :init))
      (cond ((eq arm :larm) (send req :hand "left"))
	    ((eq arm :rarm) (send req :hand "right"))
	    ((eq arm :arms) (send req :hand "both"))
	    )
      (send req :command "grasp")
      (setq res (ros::service-call "/aero_hand_controller" req))
      (print (send res :status))
      ))
  (:ungrasp (arm)
    (let (req
	  res
	  )
      (setq req (instance
		 aero_startup::AeroHandControllerRequest
		 :init))
      (cond ((eq arm :larm) (send req :hand "left"))
	    ((eq arm :rarm) (send req :hand "right"))
	    ((eq arm :arms) (send req :hand "both"))
	    )
      (send req :command "ungrasp")
      (setq res (ros::service-call "/aero_hand_controller" req))
      (print (send res :status))
      ))

  );; end of defmethod aero-upper-interface
