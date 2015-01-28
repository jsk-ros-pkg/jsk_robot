#!/usr/bin/env roseus

(ros::roseus "pr2_compressed_angle_vector_client")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-compressed-angle-vector-bridge "pr2-compressed-angle-vector-bridge.l")

(pr2-init)

(ros::subscribe "angle_vector_compressed" jsk_pr2_startup::AngleVectorCompressed
                #'(lambda (msg)
                    (let ((av (decompress-angle-vector *pr2* msg)))
                      (ros::ros-info "angle-vector comming: ~A" av)
                      (send *pr2* :angle-vector av)
                      (send *ri* :angle-vector av))))

(ros::advertise "potentio_vector_compressed" jsk_pr2_startup::AngleVectorCompressed 1)

(ros::rate 10)

(defun publish-compressed-joint-state ()
  (let ((msg (compress-angle-vector *pr2* (send *ri* :state :potentio-vector))))
    (ros::ros-info "compressed msg: ~A" (send *ri* :state :potentio-vector))
    (ros::publish "potentio_vector_compressed" msg)))

(while (ros::ok)
       (ros::spin-once)
       (publish-compressed-joint-state)
       (ros::sleep))
