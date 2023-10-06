#!/usr/bin/env roseus

(load "package://roseus_smach/src/state-machine-ros.l")

(require :base64 "lib/llib/base64.l")
(require :json-decode "package://roseus_mongo/euslisp/json/json-decode.l")

(ros::roseus-add-msgs "sensor_msgs")
(setq *target-service* (ros::get-param "~trigger" "/location_node/get_location"))

(setq *img* nil)
(setq joy-msg nil)

(defun get-location-name ()
  (let ((req (instance std_srvs::TriggerRequest :init))
        response
        parsed-json)
    (setq response (ros::service-call *target-service* req t))
    (setq parsed-json (json::parse-from-string (send response :message)))
    (cdr (assoc :formatted_address (cadr (assoc :results parsed-json))))))

(defun joy-cb (msg)
  (setq joy-msg msg)
  (setq last-time-input-joy (send msg :header :stamp)))

(defun image-cb (msg)
  (setq *img* (copy-object msg)))

(defun init-func (args)
  (let ()
    (while (null joy-msg)
      (ros::ros-info "[walk_notifyer] init: wait lead input.")
      (ros::spin-once)
      (ros::sleep))
    :started))

(defun start-func (args)
  (let ()
    :next))

(defun get-location-func (args)
  (let (location-name)
    (setq location-name (get-location-name))
    (set-alist 'description (format nil "~Aを歩いているよ" location-name) args)
    (when *img* (set-alist 'image (remove #\newline (base64encode (send *img* :serialize))) args))
    :next))

(defun end-func (args)
  (let ()
    (when (> (send (ros::time- (ros::time-now) last-time-input-joy) :to-sec) walk-end-duration)
      (return-from walk-func :finished))
    (set-alist 'description (format nil "Unitree お散歩中です") args)
    :next))

(defun walk-func (args)
  (let ((walk-start-time (ros::time-now)))
    (while (< (send (ros::time- (ros::time-now) walk-start-time) :to-sec) notify-interval)
      (ros::spin-once)
      (ros::sleep))
    :next))

(defun walk-notify-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:init :started :start)
             (:start :next :get-location)
             (:get-location :next :end) ;; to notify
             (:end :finished :finish)
             (:end :next :walk)
             (:walk :next :start))
           '((:init 'init-func)
             (:start 'start-func)
             (:get-location 'get-location-func)
             (:walk 'walk-func) ;; function maps
             (:end 'end-func))
           '(:init)      ;; initial
           '(:finish)       ;; goal
           ))
    (send sm :arg-keys 'description)
    sm))

(ros::roseus "walk_notify_smach")
(ros::wait-for-service *target-service*)
(setq last-time-input-joy (ros::time))
(setq notify-interval (ros::get-param "~notify_interval" 60))
(setq walk-end-duration (ros::get-param "~walk_end_duration" 180))

(ros::rate 1)
(setq last-time-input-joy (ros::time-now))
(ros::subscribe "/joy_head/joy_raw" sensor_msgs::Joy #'joy-cb)
(ros::subscribe "/front_camera/output/compressed" sensor_msgs::CompressedImage #'image-cb)
(do-until-key
  (exec-state-machine (walk-notify-sm)
                      `((description . ,(format nil "Unitree お散歩中です"))
                        (image . "")))
  (setq joy-msg nil)
  (setq last-time-input-joy (ros::time)))
(ros::exit)
