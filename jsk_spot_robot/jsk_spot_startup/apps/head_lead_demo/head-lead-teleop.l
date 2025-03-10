#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(setq *continue* t)
;; control head-lead
(setq *head-lead* t)
;; (setq *head-lead* nil)
(load "package://spoteus/spot-interface.l")
(spot-init)
(if (eq (send *ri* :state :power-state-shore-power-state) 'on-shore-power)
    (send *ri* :undock))
;; (send *ri* :set-impedance-param :linear-stiffness #f(250 250 250) :rotational-stiffness #f(30 30 30) :linear-damping #f(1.5 1.5 1.5) :rotational-damping #f(0.5 0.5 0.5))
;; (send *ri* :set-impedance-param :linear-stiffness #f(250 25 25) :rotational-stiffness #f(30 5 5) :linear-damping #f(1.5 1.5 1.5) :rotational-damping #f(0.5 0.5 0.5))

(setq *default-av* #f(0.0 -130.0 120.0 0.0 10.0 0.0))

;; sample
(setq *person* nil)
(setq *ball* nil)
(setq *ball-name* nil)
(setq *last-detected-time* (ros::time-now))
(defclass ncb-result-synchronizer
  :super exact-time-message-filter)

(defmethod ncb-result-synchronizer
  (:callback (bbox rects result)
             (setq *last-detected-time* (ros::time-now))
             (let (len ret)
               (setq len (length (send bbox :boxes)))
               (dotimes (i len)
                 (ros::ros-info (format nil "found ~A (~A) in ~A sec ago in ~A"
                                        (elt (send result :label_names) i)
                                        (elt (send result :label_proba) i)
                                        (send (ros::time- (ros::time-now) (send bbox :header :stamp)) :to-sec)
                                        (cdr (assoc "topic" (send result :connection-header) :test #'string=))))
                 (cond ((string= (elt (send result :label_names) i) "person")
                        (send *ri* :speak "p")
                        (push (list (send result :header :frame_id) (elt (send rects :rects) i) (elt (send bbox :boxes) i)) *person*))
                       ((or (string= (elt (send result :label_names) i) "soccer ball")
                            (string= (elt (send result :label_names) i) "pokemon ball")
                            (string= (elt (send result :label_names) i) "dice"))
                        (send *ri* :speak "ball")
                        (push (list (send result :header :frame_id)
				    (elt (send rects :rects) i)
				    (elt (send bbox :boxes) i)
				    (elt (send result :label_names) i))
			      *ball*))))
               ;; results are (list image rect box); to get box user (elt x 2) or (third x)
               (setq *person* (sort *person* #'< #'(lambda (x) (send (elt x 2) :value))))
               (setq *ball* (sort *ball* #'< #'(lambda (x) (send (elt x 2) :value))))
               )) ;; :callback
  )
(ros::roseus-add-msgs "jsk_recognition_msgs")

(setq synchronizers nil)
(dolist (image (list "left_fisheye_image" "right_fisheye_image"
                     "frontleft_fisheye_image" "frontright_fisheye_image"
                     "hand_color_image" "back_fisheye_image"))
  (push (instance ncb-result-synchronizer :init
                  (list (list (format nil "/spot/ncb_provider/~A/bbox_array" image) jsk_recognition_msgs::BoundingBoxArray)
                        (list (format nil "/spot/ncb_provider/~A/rects" image) jsk_recognition_msgs::RectArray)
                        (list (format nil "/spot/ncb_provider/~A/class" image) jsk_recognition_msgs::ClassificationResult))) synchronizers))

(defun init-walk-arm-setting ()
  (send *spot* :arm :angle-vector #f(0.0 -130.0 120.0 0.0 10.0 0.0))
  (send *ri* :angle-vector (send *spot* :angle-vector) 1000 :default-controller)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (send *ri* :set-impedance-param
	:linear-stiffness #f(250 25 75)  ;; z(up) y(side) x(front)
	:rotational-stiffness #f(3 30 30)
	:linear-damping #f(1.5 1.0 1.0)
	:rotational-damping #f(0.1 0.5 0.5))
  (setq hand-pos (ros::pos->tf-point (send *spot* :hand :end-coords :worldpos)))
  (ros::set-param "end_effector_to_joy/center_x" (send hand-pos :x))
  (ros::set-param "end_effector_to_joy/center_y" (send hand-pos :y))
  (unless (ros::wait-for-service "end_effector_to_joy/set_enabled" 10)
    (ros::ros-error "end_effector_to_joy/set_enabled not found, so quitting")
    (return-from init-walk-arm-setting :finished))
  (ros::service-call "end_effector_to_joy/set_enabled" (instance std_srvs::SetBoolRequest :init :data *head-lead*))
  )

(defun start-func (args)
  (when (not (eq (send *ri* :state :power-state-shore-power-state) 'off-shore-power))
    (send *ri* :speak "Robot is on dock")
    (ros::ros-error "Robot is on dock or powered, quit from apps")
    ;(return-from start-func :finished)
    )
  (send *ri* :speak "Hello, let's start walking")
  (if *head-lead*
    (init-walk-arm-setting)
    (progn 
      (send *spot* :arm :angle-vector #f(0 -180 180 90 0 -90))
      (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
      (send *ri* :stow-arm)
      (send *ri* :stop-grasp)
      (send *ri* :wait-interpolation)))
  :started)

(defun end-func (args)
  (ros::ros-info "end-func")
  (send *ri* :speak "Thank You")  
  (send *ri* :stow-arm)
  (unless (ros::wait-for-service "end_effector_to_joy/set_enabled" 10)
    (ros::ros-error "end_effector_to_joy/set_enabled not found, so quitting")
    (return-from end-func :finished))
  (ros::service-call "end_effector_to_joy/set_enabled" (instance std_srvs::SetBoolRequest :init :data nil))
  :finished)

;; *cmd-vel* is (cons (ros::time-now) msg)
(defun walk-func (args)
  (let ((still-sec 0)
        (countdown 0))
    (ros::ros-info "walk-func")
    (do-until-key
     (ros::rate 2)
     (ros::spin-once)
     (if *cmd-vel*
         (setq still-sec (send (ros::time- (ros::time-now) (car *cmd-vel*)) :to-sec)))
     (ros::ros-info "continue... ~A, still-sec ~A" *continue* still-sec)
     (if (null *continue*) (return-from walk-func :finished))
     (if *cmd-vel*
         (ros::ros-info "cmd-vel ... ~A sec ago ~A (~A)"
                        still-sec
                        (float-vector (send (cdr *cmd-vel*) :linear :x) (send (cdr *cmd-vel*) :linear :y) (send (cdr *cmd-vel*) :angular :z))
                        (norm (float-vector (send (cdr *cmd-vel*) :linear :x) (send (cdr *cmd-vel*) :linear :y) (send (cdr *cmd-vel*) :angular :z)))))
     (when (> still-sec countdown)
       (send *ri* :speak (format nil "~A" (case countdown (0 "z") (2 "fu") (4 "shi") (6 "mu") (8 "ya") (10 "tou") (t countdown))))
       (setq countdown (+ (ceiling still-sec) 1)))
     (if (and *cmd-vel* (> still-sec 10))
       (return-from walk-func :not-pull))
   (ros::sleep))
  :finished))

(setq *distance-thre* 2000)
(defun ball-play-func (args)
  (ros::ros-info "ball-play")
  (ros::spin-once)
  (block
   :cond
   (cond ((send *ri* :interpolatingp)
          (ros::ros-info "robot is moving..."))
         (*ball*
          (send *ri* :speak "found ball")
          (ros::service-call "end_effector_to_joy/set_enabled" (instance std_srvs::SetBoolRequest :init :data nil))
          (ros::ros-info "found ball ~A" *ball*)
          (ros::ros-info "found      ~A" (mapcar #'ros::tf-pose->coords (send-all (mapcar #'third *ball*) :pose)))
          (ros::ros-info "           ~A" (mapcar #'second *ball*))
          (ros::ros-info "           ~A" (mapcar #'fourth *ball*))
          (send *ri* :stow-arm)
          (let ((image-source (first (car *ball*)))
                (bbox (third (car *ball*)))
                (rect (second (car *ball*)))
		(ball-name (fourth (car *ball*)))
                result)
            (ros::ros-info "distance   ~A (< ~A)" (norm (send (ros::tf-pose->coords (send bbox :pose)) :worldpos)) *distance-thre*)
            (when (> (norm (send (ros::tf-pose->coords (send bbox :pose)) :worldpos)) *distance-thre*)
              (send *ri* :speak "too far")
              (setq *ball* nil)
              (return-from :cond nil))
            ;; look at the ball
            (send *spot* :head :look-at 
  		(v+ (float-vector 0 0 (/ (send bbox :dimensions :z) 2))
  		    (send (ros::tf-pose->coords (send bbox :pose)) :worldpos)))
            (send *ri* :angle-vector (send *spot* :angle-vector) 1000)
	    (send *ri* :wait-interpolation)
            (unix::sleep 1)
            (send *ri* :pick-object-in-image image-source
                  (+ (send rect :x) (/ (send rect :width) 2))
                  (+ (send rect :y) (/ (send rect :height) 2)))
            (ros::publish "/spot/pick_object_in_image/pick_object"
                          (instance std_msgs::String :init :data ball-name))
            (while (= (send *ri* :pick-object-in-image-get-state) actionlib_msgs::GoalStatus::*active*)
              (ros::ros-info "wait for pick... ~A"
                             (if (send *ri* :pick-object-in-image-feedback-msg)
                                 (send *ri* :pick-object-in-image-feedback-msg :feedback :status)))
              (ros::duration-sleep 1))
            (setq result (send *ri* :pick-object-in-image-wait-for-result))
            (if (send result :success)
                (progn
                  (send *ri* :speak "good")
                  (ros::ros-info "succeeded to pick object"))
              (progn
                (send *ri* :speak "too bad")
                (ros::ros-info "failed to pick object")
                (send *ri* :stop-grasp)
                ;;(unix::sleep 2) ;; ?? prevent from walking????
                ))
	    (if *head-lead*
              (progn
                (send *spot* :arm :angle-vector *default-av*)
       	        (send *ri* :angle-vector (send *spot* :angle-vector) 1000)
                (send *ri* :wait-interpolation)
              )
              (progn 
                (send *spot* :arm :angle-vector #f(0 -180 180 90 0 -90))
       	        (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                (send *ri* :wait-interpolation)
	        (send *ri* :stow-arm)
              ))
            (when (send result :success)
              (send *ri* :go-pos -0.5 0 0)
	      (when *head-lead*
                (send *spot* :arm :wrist-p :joint-angle 30)
                (send *ri* :angle-vector (send *spot* :angle-vector) 500))
              (send *ri* :stop-grasp))
            (setq *ball* nil)
            (ros::rate 1)
	    (if *head-lead* (init-walk-arm-setting))
            ))
         (*person*
          (send *ri* :speak "found person")
          (ros::ros-info "found person ~A" *person*)
          (ros::ros-info "found person ~A" (mapcar #'ros::tf-pose->coords (send-all (mapcar #'third *person*) :pose)))
          (ros::ros-info "           ~A" (mapcar #'second *person*))
          (ros::service-call "end_effector_to_joy/set_enabled" (instance std_srvs::SetBoolRequest :init :data nil))
          (send *spot* :arm :angle-vector *default-av*)
          (send *spot* :head :look-at (v+
                                       (float-vector 0 0 (/ (send (third (car *person*)) :dimensions :z) 2))
                                       (send (ros::tf-pose->coords (send (third (car *person*)) :pose)) :worldpos)))
          (send *ri* :angle-vector (send *spot* :angle-vector) 500)
          (setq *person* nil)
          (send *ri* :wait-interpolation)
	  (if *head-lead* (init-walk-arm-setting)))
         (t
          (ros::ros-info (format nil "nothing found ..... last detected object is ~A sec ago" (send (ros::time- (ros::time-now) *last-detected-time*) :to-sec)))
	  (when (and 
                 *head-lead*
                 (> (send (ros::time- (ros::time-now) *last-detected-time*) :to-sec) 10)
                 (> (norm (v- (coerce (mapcar #'(lambda (x) (elt (send *ri* :state :angle-vector) x)) (mapcar #'(lambda (x) (position x (send *spot* :joint-list))) (send *spot* :arm :joint-list))) float-vector) *default-av*)) 10))
            (ros::service-call "end_effector_to_joy/set_enabled" (instance std_srvs::SetBoolRequest :init :data nil))
            (send *spot* :arm :angle-vector *default-av*)
            (send *ri* :angle-vector (send *spot* :angle-vector) 500)
            (send *ri* :wait-interpolation)
	    (init-walk-arm-setting)
	    ))
         )) ;; cond
  :walk)
;;
(load "package://roseus_smach/src/state-machine-ros.l")
(defun walk-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:start :started :walk)    ;; transitions (node transition node)
             (:start :finished :end)
             (:walk :finished :end)
             (:walk :not-pull :ball-play)
             (:ball-play :walk :walk)
             (:ball-play :finished :end)
             (:end :finished :goal)
             )
           '((:start    'start-func)       ;; node-to-function maps
             (:end      'end-func)
             (:walk     'walk-func)
	     (:ball-play 'ball-play-func)
             )
           '(:start)                    ;; initial node
           '(:goal)                     ;; goal node
           ))
    (send sm :arg-keys 'description)
    sm))

;; create robot interface
(unless (boundp '*ri*) (spot-init))
(objects (list *spot*))

;;

;; this does not work...
;; (unix:signal unix::sigint '(lambda-closure nil 0 0 (sig code) (setq *continue* nil)))

;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/717
;; did not work, when we subscribe image data ???

(defun ros::roseus-sigint-handler (sig code)
  (ros::ros-warn (format nil "ros::roseus-sigint-handler ~A" sig))
  (setq *continue* nil))
(unix:signal unix::sigint 'ros::roseus-sigint-handler)

;; send start-func (set-impedance) when R Stick button is pressed
(ros::subscribe "/joy_dualsense" sensor_msgs::Joy
               #'(lambda (msg)
                   (if (and (> (length (send msg :buttons)) 11)
                            (= (elt (send msg :buttons) 11) 1))
                       (start-func nil)))
               20)
(setq *cmd-vel* (cons (ros::time-now) (instance geometry_msgs::Twist :init)))
(ros::subscribe "/spot/cmd_vel" geometry_msgs::Twist
                #'(lambda (msg) (setq *cmd-vel* (cons (ros::time-now) msg))) 20)

(ros::advertise "/spot/pick_object_in_image/pick_object" std_msgs::String 1 t)

;; state machine
(exec-state-machine (walk-sm) '((description . "お散歩しました！")(image . "")))
(exit)
