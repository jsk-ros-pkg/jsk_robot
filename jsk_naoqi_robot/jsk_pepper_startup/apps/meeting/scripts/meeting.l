#!/usr/bin/env roseus

;; load package
;;(ros::load-ros-package "naoqi_bridge_msgs")
(ros::load-ros-package "peppereus")
(ros::load-ros-package "sensor_msgs")
;;(ros::load-ros-package "roseus")

;; load simple model for fast-boot
(require :pepper "package://peppereus/pepper-simple.l")

;; load files
(require :pepper-interface "package://peppereus/pepper-interface.l")

;; var
(defvar *process-rate* 1)
;; (setq *touch_ears* #f(20.0 -2.0 -30.0 -15.0 20.0 -70.0 -88.0 -40.0 -15.0 -20.0 70.0 88.0 40.0 0.0 30.0))
;; (setq *open_pepper* #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0)) 
;; (setq *open-arms* #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 0.0 -10.0))
;; (setq *wondering-pose* #f(2.0 -15.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -10.0 -5.0))
;; (setq *reset-pose* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))
;; (setq *raise-right-hand* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0))
;; (setq *ok_pose* #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0))

;; (list *touch_ears* *open_pepper* *open_pepper* *reset-pose*)
(defvar *pose1* (list #f(20.0 -2.0 -30.0 -15.0 20.0 -70.0 -88.0 -40.0 -15.0 -20.0 70.0 88.0 40.0 0.0 30.0) #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time1* (list 2000 2000 4000 2000))

;; (list *open-arms* *wondering-pose* *wondering-pose* *reset-pose*)
(defvar *pose2* (list #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 0.0 -10.0) #f(2.0 -15.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -10.0 -5.0) #f(2.0 -15.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -10.0 -5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time2* (list 3000 2000 3000 2000))

;; (list *raise-right-hand* *raise-right-hand* *raise-right-hand* *reset-pose*)
(defvar *pose3* (list #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time3* (list 3000 1000 1000 2000))

;; (list *ok_pose* *ok_pose* *ok_pose* *reset-pose*)
(defvar *pose4* (list #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0) #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0) #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time4* (list 3000 2000 2000 2000))

;; interactive_robot_demo/miraikan_live/scripts/actions/small-exercise.l
;;(setq *start-pose-larm* #f(2.0 -2.0 -5.0 -70.0 40.0 -20.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))
;;(setq *end-pose-larm* #f(2.0 -20.0 -5.0 -70.0 20.0 -20.0 -50.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -5.0 -5.0))
;;(setq *start-pose-rarm #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -40.0 20.0 20.0 40.0 0.0 0.0))
;;(setq *end-pose-rarm #f(2.0 20.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -20.0 20.0 50.0 40.0 5.0 5.0))
(defvar *pose5* (list #f(2.0 -2.0 -5.0 -70.0 40.0 -20.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -20.0 -5.0 -70.0 20.0 -20.0 -50.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -5.0 -5.0) #f(2.0 -2.0 -5.0 -70.0 40.0 -20.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -20.0 -5.0 -70.0 20.0 -20.0 -50.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -5.0 -5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -40.0 20.0 20.0 40.0 0.0 0.0) #f(2.0 20.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -20.0 20.0 50.0 40.0 5.0 5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -40.0 20.0 20.0 40.0 0.0 0.0) #f(2.0 20.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -20.0 20.0 50.0 40.0 5.0 5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time5* (list 4000 4000 4000 4000 3000 4000 3000 3000 3000 3000 3000))

;; (list *nodding-pose* *nodding-pose* *reset-pose*)
(defvar *pose6* (list #f(2.0 -2.0 -5.0 110.0 35.0 -40.0 -60.0 -40.0 110.0 -35.0 40.0 60.0 40.0 0.0 15.0) #f(2.0 -2.0 -5.0 110.0 35.0 -40.0 -60.0 -40.0 110.0 -35.0 40.0 60.0 40.0 0.0 15.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time6* (list 2000 3000 1000))

;; 
(defvar *reset* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))
(defvar *a* #f(2.0 -2.0 -5.0 0.0 10.0 -100.0 -70.0 60.0 0.0 -10.0 100.0 70.0 -60.0 0.0 0.0))
(defvar *d* #f(2.0 -2.0 -5.0 0.0 10.0 -110.0 -70.0 60.0 0.0 -10.0 110.0 70.0 -60.0 0.0 -10.0))
(defvar *e* #f(2.0 -2.0 -5.0 0.0 10.0 -110.0 -70.0 60.0 0.0 -10.0 110.0 70.0 -60.0 0.0 -10.0))

(defvar *img-list* (list "golden-saying-by-pepper-1.png" "golden-saying-by-pepper-2.png" "golden-saying-by-pepper-3.png" "golden-saying-by-pepper-4.png" "golden-saying-by-pepper-5.png" "golden-saying-by-pepper-6.png"))
(defvar *pose-list* (list *pose1* *pose2* *pose3* *pose4* *pose5* *pose6*))
(defvar *time-list* (list *time1* *time2* *time3* *time4* *time5* *time6*))
(defvar *comment-list* (list "\\vol=40\\\\vct=120\\\\rspd=80\\研究会に出て，\\vct=150\\えらいッ！\\rspd=100\\" "\\vct=120\\\\rspd=80\\ご飯，食べた？\\rspd=100\\" "\\vct=120\\\\rspd=80\\今週も，\\vct=150\\頑張ろうッ！\\rspd=100\\" "\\vct=120\\\\rspd=80\\コメント少ないのは，順調ってことだよッ！\\rspd=100\\" "\\vct=120\\\\rspd=80\\座りっぱなし，\\vct=150\\気をつけてッ！\\rspd=100\\" "\\vct=120\\\\rspd=80\\報告して，\\vct=150\\えらいッ！\\rspd=100\\"))

;; parameter
(defparameter *continue* t)
(defparameter *count* 0)

(ros::roseus "online-meeting-enhancement-main")

(ros::subscribe "joy" sensor_msgs::Joy
		#'(lambda (msg)
		    (let (button)
		      ;; reset value
		      (setq *continue* t)
		      ;; update value
		      (setq button (send msg :buttons))
		      ;; Y button: stop
		      (print (elt button 3))
		      (if (eq (elt button 3) 1)
			  (setq *continue* nil))
		      )))

(defun init ()
  (unless (boundp '*ri*)
    (pepper-init))
  ;; check pepper's life
  (if (not (string= (send *ri* :get-life) "disabled"))
      (send *ri* :disable-life))
  (send *ri* :servo-on)
  (send *ri* :set-language "Japanese")
  (send *ri* :stop-grasp)
  (send *ri* :set-basic-awareness-enabled nil)
  (send *ri* :set-background-movement-enabled nil)
  (send *ri* :show-image (elt *img-list* 0))
  (send *ri* :speak (elt *comment-list* 0))
  (send *ri* :angle-vector-sequence (elt *pose-list* 0) (elt *time-list* 0))
  (send *ri* :wait-interpolation)
  ;;(send *ri* :angle-vector (send *pepper* :spread-hands-smally))
  (send *ri* :set-basic-awareness-enabled t)
  (send *ri* :set-background-movement-enabled t)  
  )

(defun main (&optional (wait 600))
  (ros::rate *process-rate*)
  (init)
  (print "start")
  (let ((tm nil))
    (while *continue*
      (if (not tm)
	  (setq tm (ros::time-now)))
      ;;(print (send (ros::time- (ros::time-now) tm) :to-sec))
      (if (and tm
	       (> (send (ros::time- (ros::time-now) tm) :to-sec) wait))
	  (progn
	    ;;    (unix:sleep 30) ;; sleep 30s
	    (setq *count* (+ *count* 1))
	    (if (eq *count* (length *img-list*))
		(setq *count* 0))
	    (send *ri* :set-basic-awareness-enabled nil)
	    (send *ri* :set-background-movement-enabled nil)
	    (send *ri* :show-image (elt *img-list* *count*))
	    (send *ri* :speak (elt *comment-list* *count*))
	    (send *ri* :angle-vector-sequence (elt *pose-list* *count*) (elt *time-list* *count*))
	    (send *ri* :wait-interpolation)
	    ;;(send *ri* :angle-vector (send *pepper* :spread-hands-smally))
	    (send *ri* :set-basic-awareness-enabled t)
	    (send *ri* :set-background-movement-enabled t)
	    (setq tm nil)
	    ))
      (ros::spin-once)
      (ros::sleep)
      ); while
    )
  ;;(send *ri* :set-language "Japanese")
  (send *ri* :show-image "golden-saying-by-pepper-7.png")
  (send *ri* :angle-vector-sequence (list #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 30.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -30.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)) (list 1000 1000 1000 1000))
  (send *ri* :speak "\\vct=120\\\\rspd=80\\みなさん")
  (send *ri* :wait-interpolation)  

  (send *ri* :angle-vector-sequence (list *a* *d* *a* *d* *a* *d* *a* *d* *a* *e* *reset*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 3000))
  (send *ri* :speak "\\vct=120\\\\rspd=80\\おつかれさまでしたッ！\\rspd=100\\")
  (send *ri* :wait-interpolation)
  (send *ri* :hide-image)
  (send *ri* :set-language "English")
  (print "end")
  )





