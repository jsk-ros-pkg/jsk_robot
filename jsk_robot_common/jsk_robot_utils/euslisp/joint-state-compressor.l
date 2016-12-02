#!/usr/bin/env roseus
(require :compressed-angle-vector "package://jsk_robot_utils/euslisp/angle-vector-compress.l")
(ros::roseus-add-msgs "sensor_msgs")

(ros::roseus "joint_state_compressor")


(setq *msg-type-string* (ros::get-param "~message_type"
                          "jsk_network_tools/CompressedAngleVectorPR2"))
(unless *msg-type-string*
  (error "Please specify ~~message_type"))

;; msg-type-string should be like "pkg_name/MessageType"
;; euslisp string operation is poor, so we use pathname utilities to parse it
(setq *message-package* (car (pathname-directory (pathname *msg-type-string*))))

(ros::roseus-add-msgs *message-package*)

(setq *message-type* (symbol-value
                      (intern (string-upcase (send (pathname *msg-type-string*) :name))
                              (string-upcase *message-package*))))
;; load robot model

(setq *robot-name* (ros::get-param "~robot"
                     (string-upcase (unix::getenv "ROBOT"))))
(unless *robot-name*
  (error "Please specify environment variable \"ROBOT\""))
;; load robot files... it's not so good
(load-robot-model *robot-name*)
(setq *robot* (make-robot-model-from-name *robot-name*))

(defun joint-state-callback (inmsg)
  (let ((msg (instance *message-type* :init))
        (avs nil)
        (effort-vector nil))
    ;; update robot-model
    (dolist (j (send *robot* :joint-list))
      ;; lookup msg
      (dotimes (i (length (send inmsg :name)))
        (let ((n (elt (send inmsg :name) i)))
          (if (string= n (send j :name))
              (setq avs (append avs (list
                                     (if (derivedp j linear-joint)
                                         (* 1000 (elt (send inmsg :position) i))
                                       (rad2deg (elt (send inmsg :position) i))))))))))
    (let ((angle-vector (compress-angle-vector *robot* avs)))
      (dotimes (i (length angle-vector))
        (setf (elt (send msg :angles) i) (elt angle-vector i))))
    ;; Check if effort is supported
    (when (and (find-method msg :effort) (> (length (send inmsg :effort)) 0))
      (dolist (j (send *robot* :joint-list))
        (dotimes (i (length (send inmsg :name)))
          (let ((n (elt (send inmsg :name) i)))
            (when (string= n (send j :name))
              (setq effort-vector (append effort-vector
                                          (list
                                           (round (min 255
                                                       (max (* 255
                                                               (/ (abs (elt (send inmsg :effort) i))
                                                                  (send j :max-joint-torque)))))))))))))
      (dotimes (i (length effort-vector))
        (setf (elt (send msg :effort) i) (elt effort-vector i))))
    (ros::publish "/joint_states_compressed" msg)
    ))
(ros::advertise "/joint_states_compressed" *message-type*)
(ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-state-callback)

(ros::spin)
