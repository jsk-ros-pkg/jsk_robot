(ros::load-ros-manifest "roseus")

(ros::roseus "test_nao_walker")
(ros::advertise "cmd_vel" geometry_msgs::Twist 1)
(ros::advertise "speech" std_msgs::string 1)

;; range of y,theta?
(defun test-cmd-vel (x y theta)
  (setq msg (instance geometry_msgs::Twist :init))
  (if (> x 1)
      (progn
	(format t "x must be in range [-1.0 <-> 1.0]~%")
	(setq x 1)))
  (if (< x -1)
      (progn
	(format t "x must be in range [-1.0 <-> 1.0]~%")
	(setq x -1)))
  (send msg :linear :x x)
  (send msg :linear :y y)
  (send msg :linear :z 0)  
  (send msg :angular :x 0)
  (send msg :angular :y 0)
  (send msg :angular :z theta)
  (ros::publish "cmd_vel" msg)
)

(defun test-speech (text)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil text))
  (ros::publish "speech" msg))