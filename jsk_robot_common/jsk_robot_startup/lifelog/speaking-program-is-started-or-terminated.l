#!/usr/bin/env roseus
(ros::roseus "speaking_program_is_started_or_terminated")
(ros::load-ros-manifest "roseus")
(ros::advertise "/speech" std_msgs::String 1)
;; redefine https://github.com/jsk-ros-pkg/jsk_roseus/blob/1.5.3/roseus/euslisp/roseus.l#L17
(defun speak-and-returns-valid-string (str)
  (ros::publish "/speech" (instance std_msgs::string :init :data str))
  ;; trim // ... //
  (while (setq term1 (position #\\ str :count 1) term2 (position #\\ str :count 2))
    (setq str (concatenate string (subseq str 0 term1) (subseq str (+ term2 1)))))
  str)

(defun good-night ()
  (let (str)
    (if (or (substringp "pepper" (ros::get-param "/robot/type"))
            (substringp "nao" (ros::get-param "/robot/type")))
        (setq str "\\vct=150\\Good night\\vct=100\\")
      (setq str "Robot stop"))
    (speak-and-returns-valid-string str)))

(defun good-morning ()
  (let (str)
    (if (or (substringp "pepper" (ros::get-param "/robot/type"))
            (substringp "nao" (ros::get-param "/robot/type")))
        (setq str "\\vct=150\\Good morning\\vct=100\\")
      (setq str "Robot start"))
    (speak-and-returns-valid-string str)))

(defun ros::roseus-sigint-handler (sig code)
  (ros::ROS-WARN (format nil "ros::roseus-sigint-handler ~A" sig))
  (good-night)
  (exit 1))

(require :base64 "lib/llib/base64.l")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://roseus_smach/src/state-machine-ros.l")

(setq *img* nil)
(defun image-cb (msg)
  (setq *img* (copy-object msg)))
(ros::subscribe "image_raw" sensor_msgs::CompressedImage #'image-cb)

(setq *wait-for-terminate* nil)   ;; when this flag is true
                                  ;; keep waiting at start-func, and goes to good-nigfht by C-c siganl
(setq *morning-called* nil)
(defun start-func (args)
  ;; clean up description
  (set-alist 'description "" args)
  (set-alist 'image "" args)
  (when *morning-called*  ;; when start-func called twice, it transit to :good-night
    (return-from start-func :night-started))
  (setq *morning-called* t)
  :morning-started)

(defun good-morning-func (args)
  (let ();(image (one-shot-subscribe "/pepper_robot/camera/front/image_rect_color/compressed"
         ;                           sensor_msgs::CompressedImage :timeout (* 10 1000))))
    (set-alist 'description (good-morning) args)
    (when (and *img* (< (send (ros::time- (ros::time-now) (send *img* :header :stamp)) :to-sec) 20))
      (set-alist 'image (remove #\newline (base64encode (send *img* :serialize))) args))
    ;; set *wait-for-terminate* t, so that end-func returns with :wait-for-terminate and start-func wait until *wait-for-terminate* because nil again
    (setq *wait-for-terminate* t)
    :finished))

(defun good-night-func (args)
  (set-alist 'description (good-night) args)
  (when (and *img* (< (send (ros::time- (ros::time-now) (send *img* :header :stamp)) :to-sec) 20))
    (set-alist 'image (remove #\newline (base64encode (send *img* :serialize))) args))
  :finished)

(defun end-func (args)
  ;; clean up description
  (set-alist 'description "" args)
  (set-alist 'image "" args)
  (if *wait-for-terminate*
      ;; wait-for-terminate is set T at good-morning-func, wait until C-c is pressed...
      :wait-for-terminate
    ;; when C-c is called, we set *wait-for-terminate* is nil, and goes to goal state
    :finished))

(defun wait-func (args)
  (let ((counter 0))
    ;; (ros::ros-info "wait for terminates.....") ;; when C-c is called *wait-for-terminate* goes to nil
    ;; (unix::usleep (* 1000 1000)) ;; duration-sleep did not work wel with C-c ???
    (while (and *wait-for-terminate* (< counter 1))
      (ros::duration-sleep 2)
      (incf counter))
    (cond (*wait-for-terminate*
           :wait-for-terminate)
          (t
           ;; set 2nd mail title
           (set-alist 'description (format nil "~A terminates" (ros::get-param "/robot/name")) args)
           :terminating))
    ))

(defun goodmorning-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:start :morning-started :good-morning)
             (:good-morning :finished :end)
             (:end :wait-for-terminate :wait)
             (:wait :wait-for-terminate :wait)
             (:wait :terminating :start)
             (:start :night-started :good-night)
             (:good-night :finished :end)
             (:end :finished :goal))
           '((:start 'start-func) ;; first state name must be start for email
             (:good-morning 'good-morning-func)
             (:good-night 'good-night-func)
             (:wait 'wait-func)
             (:end 'end-func)) ;; last state name must be end for email
           '(:start)
           '(:goal)
           ))
    (send sm :arg-keys 'description)
  sm))


(defun start-smach ()
  (defun ros::roseus-sigint-handler (sig code)
    (ros::ros-warn (format nil "ros::roseus-sigint-handler ~A" sig))
    (setq *wait-for-terminate* nil))
  (unix:signal unix::sigint 'ros::roseus-sigint-handler)
  (exec-state-machine (goodmorning-sm)
                      ;; set mail title
                      `((description . ,(format nil "~A started" (ros::get-param "/robot/name")))
                        (image . ""))
                      ;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/717
                      ;; did not work, when we subscribe image data, even with
                      ;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/667
                      ;; ....
                      ;; :exit-state :start
                      ;; :exit-signal-hook-func '(lambda-closure nil 0 0 (userdata) (setq *wait-for-terminate* nil))
                      )

  (unix::sleep 2)
  (ros::ros-info "exitting...")
  (sys::exit 0))
