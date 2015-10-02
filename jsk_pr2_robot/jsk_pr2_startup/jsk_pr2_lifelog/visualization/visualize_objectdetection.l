#!/usr/bin/env roseus

(ros::roseus "visualize_objectdetection") ;;:level ros::*rosdebug*)

(require :mongo-client.l "package://roseus_mongo/euslisp/mongo-client.l")
(load "models/room73b2-hitachi-fiesta-refrigerator-object.l")
(setq *mongo-database* "jsk_robot_lifelog")
;;(setq *mongo-collection* "object_detection_db")
(setq *json-parse-key-function* #'string->keyword)
(setq *marker-topic* "visualize_objectdetection_marker_array")

(setq json::*tzoffset* -8)
(setq *visualize-before-days* 10)

(ros::load-ros-manifest "posedetection_msgs")
(ros::load-ros-manifest "geometry_msgs")

(defun meta->inserted-at (meta)
  (send (cdr (assoc :inserted_at meta)) :compute-seconds))

(setq start-time (send (now) :offset-day (- *visualize-before-days*)))

(setq robot-to-objs (mongo::query
                     posedetection_msgs::Object6DPose
                     :query '((:type . "fridge_handle_upper"))
                     :meta '((:inserted_at .
                              ((:$gt . start-time))))
                     :sort '((:$natural . 1))
                     ))
(setq map-to-robots (mongo::query
                     geometry_msgs::TransformStamped
                     :sort '((:$natural . 1))
                     ))

(setq *map-to-robot* (pop map-to-robots))
(setq *fridge* (room73b2-hitachi-fiesta-refrigerator))
(setq marker-list nil)
(setq count 0)
(dolist (p robot-to-objs)
  (let ((c (ros::tf-pose->coords (send (car p) :pose)))
        (d (meta->inserted-at (cdr p)))
        handle-coords)
    (while (and (car map-to-robots)
                (<= (meta->inserted-at (cdar map-to-robots)) d))
           (setq *map-to-robot* (pop map-to-robots)))
    (ros::ros-info "map-to-robot: ~A robot-to-obj: ~A"
                   (meta->inserted-at (cdr *map-to-robot*))
                   d)
    (send c :transform (send (ros::tf-transform-stamped->coords
                              (car *map-to-robot*))
                             :inverse-transformation))
    (send c :transform (send (send *fridge* :handle-door1)
                             :inverse-transformation))
    (send *fridge* :move-to c)
    (ros::ros-info "fridge: ~A" (send *fridge* :worldcoords))
    (push (object->marker-msg *fridge*
                              (instance std_msgs::Header :init
                                        :frame_id "eng2/7f")
                              :lifetime 300
                              :alpha 0.2 ;;(* (/ (float count) (length robot-to-objs)) 0.8)
                              :id (incf count)
                              :ns "object_detection")
          marker-list)
))


(setq msg (instance visualization_msgs::markerarray :init
                    :markers marker-list))
(ros::advertise *marker-topic* visualization_msgs::markerarray 1)

(while (ros::ok)
  (ros::publish *marker-topic* msg)
  (ros::ros-info "published")
  (ros::spin-once)
  (unix:sleep 3))
