#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "aero_startup")

(ros::roseus "wheels_interface")
(ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped)

(ros::wait-for-service "/aero_torso_controller")

(defmethod aero-upper-interface
  (:go-pos (x y theta)
    (ros::publish "/move_base_simple/goal"
	(instance geometry_msgs::PoseStamped
	   :init
	   :pose (instance geometry_msgs::Pose
		    :init
		    :position (instance geometry_msgs::Point
				 :init
				 :x x :y y :z 0)
		    :orientation (instance geometry_msgs::Quaternion
				    :init
				    :x (cos (deg2rad theta))
				    :y 0
				    :z 0
				    :w (sin (deg2rad theta)))
		    ))
	)
    )
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
      ))

  );; end of defmethod aero-upper-interface
