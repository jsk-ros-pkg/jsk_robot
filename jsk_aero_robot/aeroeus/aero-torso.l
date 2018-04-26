#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")

(ros::roseus "torso_interface")

(ros::wait-for-service "/aero_torso_controller")

(defmethod aero-upper-interface
  (:move-waist (goal &optional (wrt))
    (let (req
	  res
	  )
      (setq req (instance
		 aero_startup::AeroTorsoControllerRequest
		 :init))
      (send req :x (elt goal 0))
      (send req :z (elt goal 2))
      (if (eq wrt :world)
	  (send req :coordinate "world")
	(send req :coordinate "local")
	)
      (setq res (ros::service-call "/aero_torso_controller" req))
      (print (send res :status))
      (if (not (equal (send res :status) "success"))
	  (send res :status)
	(list (send res :x) (send res :z))) ))

  );; end of defmethod aero-upper-interface
