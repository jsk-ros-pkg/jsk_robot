#!/usr/bin/env roseus

;; copied from https://github.com/jsk-ros-pkg/jsk_demos/blob/0a82540da013b8e21307354bb1c6a552185e36a2/jsk_spot_watch_dog/scripts/watch-dog.l

(load "package://unitreeeus/unitree-interface.l")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "nav_msgs")

(load "package://roseus_smach/src/state-machine-ros.l")


(defclass watchdog-interface
  :super propertied-object
  :slots (luminance ypr-euler luminance-threshold stand-start-time yaw-offset battery-temp temp-threshold human-rects rest-time-per-hour human-width-threshold human-height-threshold max-tracking-yaw paranoma-width paranoma-height lookup-angle change-body-duration bark-sound))
(defmethod watchdog-interface
  (:init
   (&rest args)
   (let ()
     (setq luminance-threshold (ros::get-param "~luminance_threshold" 0))
     (setq human-height-threshold (ros::get-param "~human_height_threshold" 0.0))
     (setq rest-time-per-hour (ros::get-param "~rest_time_per_hour" 0))
     (setq human-width-threshold (ros::get-param "~human_width_threshold" 2.0))
     (setq max-tracking-yaw (ros::get-param "~max_tracking_yaw" 0.0))
     (setq lookup-angle  (ros::get-param "~lookup_angle" -0.4)) ;; radian
     (setq change-body-duration (ros::get-param "~change_body_duration" 1.0))
#|
     (setq temp-threshold (ros::get-param "~temp_threshold" 50))
|#
     (setq bark-sound (pathname (ros::get-param "~bark_sound" "/home/pi/Music/bark.wav")))
#|
     (let ((ros-img (one-shot-subscribe "~panorama_image" sensor_msgs::Image)))
       (setq paranoma-width (send ros-img :width))
       (setq paranoma-height (send ros-img :height)))
     (ros::subscribe "~input" sensor_msgs::Image #'send self :image-cb)
|#
     (ros::subscribe "/people_pose" jsk_recognition_msgs::PeoplePoseArray #'send self :detect-cb)
#|
     (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'send self :odom-cb)
|#
     (setq stand-start-time (ros::time-now))
#|
     (setq yaw-offset 0)
     (setq luminance 0)
     (setq battery-temp 0)
|#
     (setq ypr-euler '(0 -0.3 0))
     ))
#|
  (:spot-status-battery-states-callback  (msg)   ;; overwrite
                                         (send-super :spot-status-battery-states-callback msg)
                                         (setq battery-temp (apply #'max (coerce (send (elt (send msg :battery_states) 0) :temperatures) cons)))
                                         (ros::ros-debug "battery temp: ~A" battery-temp)
                                         )
  (:image-cb (msg)
             (let ((img (instance grayscale-image :init
                                  (send msg :width)
                                  (send msg :height)
                                  (copy-object (send msg :data)))))
               (dotimes (w (send img :width))
                 (dotimes (h (send img :height))
                   (setq luminance (+ luminance (send img :pixel w h)))))
               (setq luminance (/ luminance (* (send img :width) (send img :height))))
               (ros::ros-debug "receive ~A, luminance is ~A" (send (send msg :header) :frame_id) luminance)
               t)
             )
|#
  (:detect-cb (msg)
              (let (bbox largest-box)
                (setq bbox (mapcar #'(lambda (p)
                                       (geo::make-bounding-box (mapcar #'(lambda (x)
                                                                           (float-vector (send x :position :x) (send x :position :y)))
                                                                       (send p :poses)))) (send msg :poses)))
                (setq bbox (sort bbox #'> #'(lambda (x) (* (elt (send x :diagonal) 0) (elt (send x :diagonal) 1)))))
                (setq human-rects (instance jsk_recognition_msgs::RectArray :init
                                            :rects
                                            (mapcar #'(lambda (x) (instance jsk_recognition_msgs::Rect :init
                                                                            :x (elt (send x :center) 1)
                                                                            :y (elt (send x :center) 0)
                                                                            :width (elt (send x :diagonal) 1)
                                                                            :height (elt (send x :diagonal) 0))) bbox)))
                (setq human-rects (send human-rects :rects))))
#|
  (:odom-cb (msg)
            (let ()
              (setq q (float-vector
                      (send msg :pose :pose :orientation :w)
                      (send msg :pose :pose :orientation :x)
                      (send msg :pose :pose :orientation :y)
                      (send msg :pose :pose :orientation :z)))
              (setq ypr-euler (elt (rpy-angle (quaternion2matrix q)) 0))
              (if (and
                   (< (abs (elt (send *ri* :get-val 'ypr-euler) 2)) 0.05)
                   (< (abs (elt (send *ri* :get-val 'ypr-euler) 1)) 0.05))
                  (send *ri* :set-val 'yaw-offset (elt (send *ri* :get-val 'ypr-euler) 0)))
              (ros::ros-debug  "body quaternion is ~A, YPR euler is ~A ~%" q ypr-euler)
              ))
  (:bright-surrounding () (if (> luminance luminance-threshold) t nil))
  ;;(:high-battery-temp () (if (> battery-temp temp-threshold) t nil))
  (:high-battery-temp () (if (> battery-temp temp-threshold)
                             (progn
                               (ros::ros-info "bat temp: ~A; temp thresh: ~A" battery-temp temp-threshold)
                             t)
		   nil))
|#
  )


(defun watchdog-init (&optional (create-viewer))
  (unless (ros::ok) (ros::roseus "robot_watchdog"))
  (setq *wi* (instance watchdog-interface :init))

  (ros::spin-once)
  (when create-viewer (objects (list *robot*)))
  (send *robot* :body-pose (send *wi* :get-val 'ypr-euler))
  (send *ri* :body-pose (send *robot* :worldcoords))
  (send *ri* :play-sound (send *wi* :get-val 'bark-sound))
  )


(defun func-day-basic (userdata-alist)
  (let ((ret nil))
#|
    (ros::ros-debug "day motion, luminance: ~A, stand duration: ~A min" (send *ri* :get-val 'luminance) (/ (send (ros::time- (ros::time-now) (send *ri* :get-val 'stand-start-time)) :to-sec) 60))
    (when (null (send *ri* :bright-surrounding))
      (send *ri* :sit)
      (send *ri* :power-off)
      (ros::ros-info "sit down because surrounding is dark")
      (setq ret :light-off))
    (unless (ros::ok)
      (setq ret :preempted))
|#
    ret))

(defun func-rest (userdata-alist)
  (let ((ret nil))
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (setq ret (func-day-basic userdata-alist))
      (if ret (return-from func-watch ret))
      (when (> (/ (send (ros::time- (ros::time-now) (send *wi* :get-val 'stand-start-time)) :to-sec) 60)
             (send *wi* :get-val 'rest-time-per-hour))
        (ros::ros-info  "finish rest")
        (send *wi* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :power-on)
        (send *ri* :stand)
        (setq ret :stand))
      (unix::sleep 1) ;; (ros::sleep)
      )
    ret))

(defun func-watch (userdata-alist)
  (let ((ret nil))
    ;;(ros::rate (/ 1 (float (send *ri* :get-val 'change-body-duration))))
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (setq ret (func-day-basic userdata-alist))
#|
      (if ret (return-from func-watch ret))
|#
      ;; rest
      (when (> (/ (send (ros::time- (ros::time-now) (send *wi* :get-val 'stand-start-time)) :to-sec) 60)
             (- 60  (send *wi* :get-val 'rest-time-per-hour)))
        (ros::ros-info "have a rest")
        (send *wi* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :sit)
        (send *ri* :power-off)
        (setq ret :sit)
        (return-from func-watch ret))
#|
      ;; check battery temperature
      (when (send *ri* :high-battery-temp)
        (ros::ros-warn "the battery temperature is higher than the threshold: ~A vs ~A, sit down and turn off servo" (send *ri* :get-val 'battery-temp) (send *ri* :get-val 'temp-threshold))
        (send *ri* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :sit)
        (send *ri* :power-off)
        (setq ret :sit)
        (return-from func-watch ret))
|#
      ;; watch person
      (let
          ((max-person-height 0)
           (target-person nil)
           (human-height-threshold (send *wi* :get-val 'human-height-threshold))
           target-rpy-euler)
        ;; workaround to handle the smaller bbox height when robot look up (head up).
        (if (< (elt (send *wi* :get-val 'ypr-euler) 1) (* (send *wi* :get-val 'lookup-angle) 0.5))
            (setq human_height_threshold (* (send *wi* :get-val 'human-height-threshold)  0.8)))
        (dolist (rect (send *wi* :get-val 'human-rects))
          (ros::ros-info "received rect center: ~A ~A, size: ~A ~A" (send rect :x) (send rect :y) (send rect :width) (send rect :height))
          (ros::ros-info "         width  ~A < ~A" (send rect :width) (* (send *wi* :get-val 'human-width-threshold) (send rect :height)))
          (if (send *wi* :get-val 'paranoma-height)
              (ros::ros-info "         width  ~A < ~A" (send rect :width) (* (send *wi* :get-val 'human-height-threshold) (send *wi* :get-val 'paranoma-height))))
          (ros::ros-info "         height ~A > ~A" (send rect :height) max-person-height)
          (when (and (< (send rect :width) (* (send *wi* :get-val 'human-width-threshold) (send rect :height)))
                     (or (null (send *wi* :get-val 'paranoma-height))
                         (> (send rect :height) (* (send *wi* :get-val 'human-height-threshold) (send *wi* :get-val 'paranoma-height))))
                     (> (send rect :height) max-person-height))
            (setq max-person-height (send rect :height))
            (setq target-person rect)))
        (ros::ros-info "target-> rect ~A" target-person)
        ;; for go1
        (send *robot* :reset-pose)
        (send *robot* :newcoords (make-cascoords))
        (send *irtviewer* :viewer :viewsurface :clear)
        (if (null target-person)
            (send *robot* :body-pose #f(0 0 0))
          (progn
            (let (target-pos)
              (setq target-pos (send (send *robot* :head-end-coords :copy-worldcoords)
                                     :transform-vector (float-vector (* 4.0 (- 200 (send target-person :x)))
                                                                     (max (send target-person :y) 500)
                                                                     800)))
              (send target-pos :draw-on :flush nil :size 100)
              (send *robot* :look-at target-pos :revert-if-fail nil)
	      (print (send *wi* :get-val 'bark-sound))
	      (send *ri* :play-sound (send *wi* :get-val 'bark-sound))
	      )))
        (send *irtviewer* :draw-objects :clear nil :flush nil)
        (send *irtviewer* :viewer :flush)
        (x::window-main-one)
        (send *ri* :body-pose (send *robot* :worldcoords))
        )
      (let ((start-time (ros::time-now)))
        ;; single thred, so consider the image luminance calculation (~0.2s)
        (while (< (send (ros::time- (ros::time-now) start-time) :to-sec) (send *wi* :get-val 'change-body-duration))
          (ros::spin-once)
          (unix::usleep (* 10 1000))))
      )
    ret))

(defun func-dark (userdata-alist)
  (let ((ret nil))
#|
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (ros::ros-debug "dark motion, luminance: ~A, stand duration: ~A min" (send *ri* :get-val 'luminance) (/ (send (ros::time- (ros::time-now) (send *ri* :get-val 'stand-start-time)) :to-sec) 60))
      (when (send *ri* :bright-surrounding)
        (ros::ros-info "stand up because surrounding is bright")
        (send *ri* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :power-on)
        (send *ri* :stand)
        (setq ret :light-on))
      (unless (ros::ok)
        (setq ret :preempted))
      (unix::sleep 1))
|#
    (setq ret :light-on)
    ret))


(defun watchdog-sm ()
  (let (sm sm-day)
    (setq sm-day
          (make-state-machine
           '((:watch :preempted :preempted) ;; transitions
             (:watch :sit :rest)
             (:watch :light-off :light-off)
             (:rest :preempted :preempted)
             (:rest :stand :watch)
             (:rest :light-off :light-off))
           '((:watch 'func-watch) ;; function maps
             (:rest 'func-rest))
           '(:watch)      ;; initial
           '(:light-off :preempted) ;; goal
           ))
    (setq sm
          (make-state-machine
           '((:dark-motion :light-on :day-motion) ;; transitions
             (:dark-motion :preempted :preempted)
             (:day-motion :light-off :dark-motion)
             (:day-motion :preempted :preempted))
           `((:dark-motion 'func-dark)  ;; functon maps
             (:day-motion ,sm-day))   ;; set "nestaed state machine"
           '(:dark-motion)      ;; initial
           '(:preempted) ;; goal
           ))
    sm))

;; create robot interface
(go1-init) (setq *robot* *go1*)
(objects (list *robot*))
(watchdog-init nil)

;; state machine
(exec-state-machine (watchdog-sm))

