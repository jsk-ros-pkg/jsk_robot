#!/usr/bin/env roseus
(require :compressed-angle-vector "package://jsk_robot_utils/euslisp/angle-vector-compress.l")
(ros::roseus-add-msgs "sensor_msgs")

(ros::roseus "joint_state_decompressor")


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
  (error "Please specify ~~robot_name"))
;; load robot files... it's not so good
(load-robot-model *robot-name*)
(setq *robot* (make-robot-model-from-name *robot-name*))

(defun joint-state-compressed-callback (inmsg)
  (let ((msg (instance sensor_msgs::JointState :init)))
    (send msg :name (send-all (send *robot* :joint-list) :name))
    (let ((avs (coerce (decompress-angle-vector
                        *robot* (send inmsg :angles))
                       cons)))
      (setq avs (subseq avs 0 (length (send *robot* :joint-list))))
      (send msg :position (mapcar #'(lambda (v j)
                                      (if (derivedp j linear-joint)
                                          (* 0.001 v)
                                        (deg2rad v)))
                                  avs (send *robot* :joint-list))))
    ;; Check if inmsg support :effort or not
    (when (find-method inmsg :effort)
      (let ((evs nil))
        (dotimes (i (length (send *robot* :joint-list)))
          (let ((j (elt (send *robot* :joint-list) i)))
            (setq evs (append evs (list (* (/ (elt (send inmsg :effort) i) 255.0)
                                           (send j :max-joint-torque)))))))
        (send msg :effort evs)))
    (send msg :header :stamp (ros::time-now))
    (ros::publish "/joint_states_decompressed" msg)))

(ros::advertise "/joint_states_decompressed" sensor_msgs::JointState 1 t)
(ros::subscribe "/joint_states_compressed" *message-type*
  #'joint-state-compressed-callback)

(ros::spin)
