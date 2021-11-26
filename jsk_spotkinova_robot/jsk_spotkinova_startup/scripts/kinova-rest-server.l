#!/usr/bin/env roseus

(defclass kinova-arm-rest-server
  :slots ())
(defmethod kinova-arm-rest-server
  (:init ()
   (ros::roseus "kinova_rest")
   (ros::roseus-add-srvs "std_srvs")
   (load "package://spotkinovaeus/spotkinova-interface.l")
   (spotkinova-init)
   (ros::advertise-service "/arm_gen3/rest" std_srvs::Trigger #'send self :rest-arm))
  (:rest-arm (req)
    (let ((r (send req :response)))
      (send r :success
            (progn
              (send *ri* :state :potentio-vector)
              (send *ri* :angle-vector (send *spotkinova* :kinova-rest-pose) :fast)
              (send *ri* :wait-interpolation)))
      r)))

(setq a (instance kinova-arm-rest-server :init))
(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
