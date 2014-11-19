#!/usr/bin/env roseus
(ros::load-ros-manifest "dynamic_reconfigure")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "jsk_topic_tools")

(ros::roseus "roi_reconfigure_call")

(defun call-roi (width height x_off y_off &key (decimate_x 2) (decimate_y 2)
                       (service-name "roi_crop_decimate/set_parameters")
                       (buffer-service "/roi_topic_buffer_server/update")
                       (buffer-topic "/roi_camera_output/image_raw"))
  (ros::ros-info "ROI: ~D ~D ~D ~D" width height x_off y_off)
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config
          :ints
          (list
           (instance dynamic_reconfigure::IntParameter :init :name "decimation_x" :value decimate_x)
           (instance dynamic_reconfigure::IntParameter :init :name "decimation_y" :value decimate_y)
           (instance dynamic_reconfigure::IntParameter :init :name "x_offset" :value x_off)
           (instance dynamic_reconfigure::IntParameter :init :name "y_offset" :value y_off)
           (instance dynamic_reconfigure::IntParameter :init :name "width" :value width)
           (instance dynamic_reconfigure::IntParameter :init :name "height" :value height)
           )
          )
    (ros::service-call service-name req)
    (unix::usleep (* 400 1000))
    (let ((req (instance jsk_topic_tools::UpdateRequest
                         :init :topic_name buffer-topic)))
      (ros::service-call buffer-service req))
    ))

(defun callback-angle (srvname bufsrv buftopic msg)
  (let ((pts (send msg :polygon :points))
        lst)
    (setq lst
          (mapcar #'(lambda (x) (list (round (send x :x)) (round (send x :y)))) pts))
    (let ((x_st (caar lst))
          (y_st (cadr (car lst)))
          (x_ed (car (cadr lst)))
          (y_ed (cadr (cadr lst))))
      (if (> x_st x_ed) (let ((tmp x_st)) (setq x_st x_ed x_ed tmp)))
      (if (> y_st y_ed) (let ((tmp y_st)) (setq y_st y_ed y_ed tmp)))
      (call-roi (- x_ed x_st) (- y_ed y_st) x_st y_st
                :service-name srvname
                :buffer-service bufsrv
                :buffer-topic buftopic))
    ))

(setq *default-roi-width* 320)
(setq *default-roi-height* 240)
(defun callback-point (image-width image-height srvname bufsrv buftopic msg)
  (format *error-output* "callback~%")
  (let* ((pt (send msg :point))
         (x (round (send pt :x)))
         (y (round (send pt :y)))
         (st_x (- x (/ *default-roi-width* 2)))
         (st_y (- y (/ *default-roi-height* 2)))
         )
    (if (< st_x 0) (setq st_x 0))
    (if (< st_y 0) (setq st_y 0))
    (if (> (+ st_x *default-roi-width*) image-width) (setq st_x (- image-width *default-roi-width*)))
    (if (> (+ st_y *default-roi-height*) image-height) (setq st_y (- image-height *default-roi-height*)))
    (call-roi *default-roi-width* *default-roi-height* st_x st_y
              :service-name srvname
              :buffer-service bufsrv
              :buffer-topic buftopic
              )))

(ros::subscribe "head_snap/image_rect_color/screenrectangle" geometry_msgs::PolygonStamped
                #'callback-angle
                "roi_crop_decimate/set_parameters" ;; decimate
                "/roi_topic_buffer_server/update"  ;; buffer-service
                "/roi_camera_output/image_raw")    ;; buffer-topic
(ros::subscribe "head_snap/image_rect_color/screenpoint" geometry_msgs::PointStamped
                #'callback-point
		640 480
                "roi_crop_decimate/set_parameters"
                "/roi_topic_buffer_server/update"
                "/roi_camera_output/image_raw")

(ros::subscribe "lhand_snap/image_rect/screenrectangle" geometry_msgs::PolygonStamped
                #'callback-angle
                "roi_crop_lh_decimate/set_parameters" ;; decimate
                "/roi_lh_topic_buffer_server/update"  ;; buffer-service
                "/roi_camera_output_lh/image_raw")    ;; buffer-topic
(ros::subscribe "lhand_snap/image_rect/screenpoint" geometry_msgs::PointStamped
                #'callback-point
		640 480
                "roi_crop_lh_decimate/set_parameters"
                "/roi_lh_topic_buffer_server/update"
                "/roi_camera_output_lh/image_raw")
(ros::subscribe "rhand_snap/image_rect/screenrectangle" geometry_msgs::PolygonStamped
                #'callback-angle
                "roi_crop_rh_decimate/set_parameters" ;; decimate
                "/roi_rh_topic_buffer_server/update"  ;; buffer-service
                "/roi_camera_output_rh/image_raw")    ;; buffer-topic
(ros::subscribe "rhand_snap/image_rect/screenpoint" geometry_msgs::PointStamped
                #'callback-point
		640 480
                "roi_crop_rh_decimate/set_parameters"
                "/roi_rh_topic_buffer_server/update"
                "/roi_camera_output_rh/image_raw")

(setq *update* nil)
(ros::rate 20)
(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )
