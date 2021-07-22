#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(spot-init nil) ;; do not create-viewer

;; power on
(send *ri* :claim)
(send *ri* :power-on)

(send *ri* :stand)
(ros::duration-sleep 1)

;; states
(format t "(send *ri* :state :metrics) : ~A~%" (send *ri* :state :metrics))
(format t "(send *ri* :state :leases) : ~A~%" (send *ri* :state :leases))
(format t "(send *ri* :state :estop) : ~A~%" (send *ri* :state :estop))
(format t "(send *ri* :state :wifi) : ~A~%" (send *ri* :state :wifi))
(format t "(send *ri* :state :power-state) : ~A~%" (send *ri* :state :power-state))
(format t "(send *ri* :state :battery-faults) : ~A~%" (send *ri* :state :battery-faults))
(format t "(send *ri* :state :system-faults) : ~A~%" (send *ri* :state :system-faults))
(format t "(send *ri* :state :feedback) : ~A~%" (send *ri* :state :feedback))

;; basic services
(format t "Testing Basic Functionality~%")
(send *ri* :sit)
(ros::duration-sleep 3)

(send *ri* :power-off)
(ros::duration-sleep 3)

(send *ri* :release)
(ros::duration-sleep 3)

(send *ri* :claim)
(ros::duration-sleep 3)

(send *ri* :power-on)
(ros::duration-sleep 3)

(send *ri* :stand)
(ros::duration-sleep 5)

;; cmd-vel
(format t "Testing cmd-vel~%")
(setq end-time (ros::time+ (ros::time 10) (ros::time-now)))
(while (ros::time< (ros::time-now) end-time)
  (send *ri* :go-velocity 0 0 0.8)
  )

(setq end-time (ros::time+ (ros::time 10) (ros::time-now)))
(while (ros::time< (ros::time-now) end-time)
  (send *ri* :go-velocity 0 0 -0.8)
  )

;; body-pose
(format t "Testing body-pose~%")
(send *ri* :body-pose 0 0.2 0)
(ros::duration-sleep 1)
(send *ri* :body-pose 0 -0.2 0)
(ros::duration-sleep 1)
(send *ri* :body-pose 0 0 0.2)
(ros::duration-sleep 1)
(send *ri* :body-pose 0 0 -0.2)
(ros::duration-sleep 1)
(send *ri* :body-pose 0.1 0 0)
(ros::duration-sleep 1)
(send *ri* :body-pose -0.1 0 0)
(ros::duration-sleep 1)
(send *ri* :body-pose 0 0 0)
(ros::duration-sleep 1)

;; go pos
(format t "Testing go pos")
(send *ri* :go-pos 1 0)
(send *ri* :go-pos 0 0 90)
(send *ri* :go-pos 0 1 -90)

;;
(exit 1)
