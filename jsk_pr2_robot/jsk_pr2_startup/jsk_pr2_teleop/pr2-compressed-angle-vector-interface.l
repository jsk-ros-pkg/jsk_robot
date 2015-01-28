#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-compressed-angle-vector-bridge "pr2-compressed-angle-vector-bridge.l")

(defclass pr2-compressed-angle-vector-interface
  :super pr2-interface
  :slots (current-angle-vector groupname-compressed updating))

(defmethod pr2-compressed-angle-vector-interface
  (:init
   (&rest args &key (type :default-controller)
          (move-base-action-name "move_base") &allow-other-keys)
   (setq groupname-compressed "compressed")
   (ros::create-nodehandle groupname-compressed)
   (ros::advertise "angle_vector_compressed" jsk_pr2_startup::AngleVectorCompressed 1)
   (ros::subscribe "potentio_vector_compressed" jsk_pr2_startup::AngleVectorCompressed
                   #'send self :potentio-vector-compressed-callback :groupname groupname-compressed)
   (send-super* :init :type type :move-base-action-name move-base-action-name args))
  (:angle-vector
   (av &optional tm &rest args)
   (if tm (ros::ros-warn "time is ignored"))
   (ros::publish "angle_vector_compressed" (compress-angle-vector robot av))
   (send robot :angle-vector av))
  (:angle-vector-sequence
   (avs &optional tms &rest args)
   (if tms (ros::ros-warn "time-lst is ignored"))
   (dolist (av avs)
     (send self :angle-vector av)
     (unix:sleep 3)))
  (:state
   (&rest args)
   (case (car args)
     (:potentio-vector
      (send self :update-current-angle-vector)
      current-angle-vector)
     (:reference-vector
      (send self :update-current-angle-vector)
      (while updating)
      current-angle-vector)
     (t
      (send-super* :state args))))
  (:update-current-angle-vector
   (&key (timeout 1000))
   (let ((start-time (ros::time-now)))
     (setq updating t)
     (while (and updating (< (send (ros::time- (ros::time-now) start-time) :to-nsec) (* timeout 1000 1000)))
          (ros::spin-once groupname-compressed))))
  (:potentio-vector-compressed-callback
   (msg)
   (setq current-angle-vector (decompress-angle-vector robot msg))
   (send robot :angle-vector current-angle-vector)
   (ros::ros-info "current-angle-vector: ~A" current-angle-vector)
   (setq updating nil)))


(unless (fboundp 'pr2-init-org)
  (setf (symbol-function 'pr2-init-org) (symbol-function 'pr2-init)))

(defun pr2-init (&rest args)
  (unless (boundp '*ri*)
    (setq *ri* (instance pr2-compressed-angle-vector-interface :init)))
  (apply #'pr2-init-org args))

(ros::roseus "hoge")
(pr2-init)

(provide :pr2-compressed-angle-vector-interface)
