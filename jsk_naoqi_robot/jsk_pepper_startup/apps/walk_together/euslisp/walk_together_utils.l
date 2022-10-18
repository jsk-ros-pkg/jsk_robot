#!/usr/bin/env roseus

(ros::load-ros-package "naoqi_bridge_msgs")
(ros::load-ros-package "speech_recognition_msgs")
(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "jsk_rviz_plugins")

;; init node
(ros::roseus "walk_together_utils")

;; advertise topic
(ros::advertise "/pictogram" jsk_rviz_plugins::Pictogram 1)

(defclass walk_together_utils
  :super propertied-object
  :slots (moving-vec moving-direction sum)
  )

(defmethod walk_together_utils
  (:init 
   ()
   (ros::subscribe "/pepper_robot/bumper" naoqi_bridge_msgs::Bumper #'send self :bumper-cb)
   ;; TODO: add laser
   (send self :show-pictogram "(^_^)" (/ 0 255.0) (/ 255 255.0) (/ 255 255.0) 0)
   )
  (:bumper-cb
   (msg)
   (if (eq (send msg :state) 1)
     (progn
       (send *ri* :go-stop)
       (unix:sleep 3)
       )
     ))
  (:sum-of-squares
   (vec)
   (let ((sum))
     (setq sum (+ (* (elt vec 0) (elt vec 0)) (* (elt vec 1) (elt vec 1)) (* (elt vec 2) (elt vec 2))))
     sum))
  (:get-moving-vec
   ()
   (let (vec p1-left p1-right p2-left p2-right)
     (send *pepper* :reset-pose)
     ;; p1-left
     ;; #<coordinates #X9ccd300  63.714 203.238 -252.333 / 0.069 0.999 -0.229>
     ;; send p1-left :worldpos
     ;; #f(63.7136 203.238 -252.333)
     (setq p1-left (send *pepper* :larm :end-coords :copy-worldcoords))
     ;; p1-right
     ;; #<coordinates #X925a358  63.714 -203.238 -252.333 / -0.069 0.999 0.229>
     ;; send p1-right :worldpos
     ;; #f(63.7136 -203.238 -252.333)
     (setq p1-right (send *pepper* :rarm :end-coords :copy-worldcoords))
     (send *pepper* :angle-vector (send *ri* :state :potentio-vector))
     (setq p2-left (send *pepper* :larm :end-coords :copy-worldcoords))
     (setq p2-right (send *pepper* :rarm :end-coords :copy-worldcoords))
     (setq vec-left (v- (send p2-left :worldpos) (send p1-left :worldpos)))
     (setq vec-right (v- (send p2-right :worldpos) (send p1-right :worldpos)))
     (if (> (send self :sum-of-squares vec-right) (send self :sum-of-squares vec-left))
	 (setq moving-vec vec-right)
       (setq moving-vec vec-left))
     )
   moving-vec)
  (:set-moving-direction
   (vec)
   ;; rotation
   (let ((x nil)
	 (y nil)
	 (z nil)
	 (theta nil))
     (setq x (elt vec 0))
     (setq y (elt vec 1))
     (setq z (elt vec 2))
     (setq theta (atan y x))
     (setq theta  (* theta (/ 180 pi)))
     (if (and
	  (> (abs z) 100)
	  (> (abs theta) 40) (< (abs theta) 150))
	 (progn
	   (if (> theta 0)
	       (progn
		 (setq moving-direction "ccw")
		 (send self :show-pictogram "←" (/ 255 255.0) (/ 0 255.0) (/ 255 255.0) 0)
		 )
	     (progn
	       (setq moving-direction "cw")
	       (send self :show-pictogram "→" (/ 0 255.0) (/ 255 255.0) (/ 0 255.0) 0)
	       )))
       (setq moving-direction nil)
       ))
   ;; move forward
   (if (eq moving-direction nil)
       (progn
	 (let ((x nil)
	       (y nil)
	       (z nil))
	   (setq x (elt vec 0))
	   (setq y (elt vec 1))
	   (setq z (elt vec 2))
	   (if (and (> (abs x) 50) (> (abs z) 50))
	       (cond ((< x -50)
		      (setq moving-direction "bw")
		      (send self :show-pictogram "↓" (/ 138 255.0) (/ 43 255.0) (/ 226 255.0) 0)
		      )
		     ((> z 50)
		      (setq moving-direction "fw")
		      (send self :show-pictogram "↑" (/ 255 255.0) (/ 69 255.0) (/ 0 255.0) 0)
		      )
		     (t
		      (setq moving-direction "stop")
		      (send self :show-pictogram "---" (/ 0 255.0) (/ 255 255.0) (/ 255 255.0) 0)
		      ))
	     ))))
   )
  (:get-moving-direction
   ()
   moving-direction)
  (:look-at
   (moving-direction)
   (if (string= moving-direction "cw")
       (progn
	 (send *pepper* :head :neck-p :joint-angle 0)
	 (send *pepper* :head :neck-y :joint-angle -20) ;; -10
	 (send *ri* :angle-vector (send *pepper* :angle-vector) 700)))
   (if (string= moving-direction "ccw")
       (progn
	 (send *pepper* :head :neck-p :joint-angle 0)
	 (send *pepper* :head :neck-y :joint-angle 20) ;; 10
	 (send *ri* :angle-vector (send *pepper* :angle-vector) 700)))
   (if (string= moving-direction "fw")
       (progn
	 (send *pepper* :head :neck-p :joint-angle -10) ;; -10
	 (send *pepper* :head :neck-y :joint-angle 0)
	 (send *ri* :angle-vector (send *pepper* :angle-vector) 700)))
   (if (string= moving-direction "bw")
       (progn
	 (send *pepper* :head :neck-p :joint-angle 10)
	 (send *pepper* :head :neck-y :joint-angle -20) ;; -10
	 (send *ri* :angle-vector (send *pepper* :angle-vector) 700)
	 ))
   (if (string= moving-direction "stop")
       (progn
	 (send *pepper* :head :neck-p :joint-angle 0)
	 (send *pepper* :head :neck-y :joint-angle 0) ;; -10
	 (send *ri* :angle-vector (send *pepper* :angle-vector) 700)))
   )
  (:rotate
   (moving-direction)
   (if (string= moving-direction "ccw")
       (send *ri* :go-velocity 0 0 0.2) ;; 0.5
     )
   (if (string= moving-direction "cw")
       (send *ri* :go-velocity 0 0 -0.2) ;; -0.5
     ))
  (:go-forward
   (moving-direction)
   (if (string= moving-direction "bw")
       (send *ri* :go-velocity -0.3 0 0) ;; -0.4
     )
   (if (string= moving-direction "fw")
       (send *ri* :go-velocity 0.3 0 0) ;; 0.6
     )
   (if (string= moving-direction "stop")
       (send *ri* :go-stop))
   )
  (:show-pictogram
   (sentence r g b mode)
   (let ((msg (instance jsk_rviz_plugins::Pictogram :init)))
     (send msg :action mode) ; JUMP_ONCE: 6, ADD: 0
     (send msg :mode 1)   ; STRING_MODE
     (send msg :character sentence)
     (send msg :header :frame_id "/base_link")
     (send msg :header :stamp (ros::time-now)) ; rospy.Time.now()
     (send msg :pose :position :z 1.6)
     (send msg :pose :orientation :w 0.7)
     (send msg :pose :orientation :x 0)
     (send msg :pose :orientation :y -0.7)
     (send msg :pose :orientation :z 0)
     (send msg :speed 1.0)
     (send msg :size 1)
     (send msg :color :r r)
     (send msg :color :g g)
     (send msg :color :b b)
     (send msg :color :a 1.0)
     (ros::publish "/pictogram" msg)
     )
   )
  )
