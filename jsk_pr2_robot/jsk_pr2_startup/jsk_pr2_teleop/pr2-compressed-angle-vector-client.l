#!/usr/bin/env roseus

(ros::roseus "pr2_compressed_angle_vector_client")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :compressed-angle-vector "package://jsk_network_tools/euslisp/angle-vector-compress.l")
(ros::roseus-add-msgs "jsk_network_tools")
(pr2-init)

(ros::subscribe "angle_vector_compressed" jsk_network_tools::CompressedAngleVectorPR2
  #'(lambda (msg)
      (let ((av (decompress-angle-vector *pr2* (send msg :angles))))
        (ros::ros-info "angle-vector comming: ~A" av)
        (send *pr2* :angle-vector av)
        (send *ri* :angle-vector av))))

(ros::advertise "potentio_vector_compressed" jsk_network_tools::CompressedAngleVectorPR2 1)

(ros::rate 10)

(defun publish-compressed-joint-state ()
  (let ((msg (instance jsk_network_tools::CompressedAngleVectorPR2 :init)))
    (send msg :angles
          (compress-angle-vector *pr2* (send *ri* :state :potentio-vector)))
    (ros::ros-info "compressed msg: ~A" (send *ri* :state :potentio-vector))
    (ros::publish "potentio_vector_compressed" msg)))

(while (ros::ok)
  (ros::spin-once)
  (publish-compressed-joint-state)
  (ros::sleep))
