#!/usr/bin/env roseus

(ros::roseus "go_to_kitchen_with_mail_notify")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(ros::roseus-add-srvs "jsk_patrol")

(setq *place-name* (ros::get-param "~place_name"))
(setq *mail-title* (ros::get-param "~mail_title"))
(setq *from-address* (ros::get-param "~from_address"))
(setq *to-address* (ros::get-param "~to_address"))


(defun call-mail-service (place-name
                          mail-title
                          from-address
                          to-address)
  (let (service-request-instance)
    (setq service-request-instance (instance jsk_patrol::PatrolMailNotifyRequest :init))
    (send service-request-instance :place_name
          (instance std_msgs::String :init :data place-name))
    (send service-request-instance :mail_title
          (instance std_msgs::String :init :data mail-title))
    (send service-request-instance :from_address
          (instance std_msgs::String :init :data from-address))
    (send service-request-instance :to_address
          (instance std_msgs::String :init :data to-address))
    (ros::service-call "/mail_notify/notify" service-request-instance)))


(defun go-to-kitchen ()
  ;; call k-okada
  (send *ri* :speak-jp "キッチンに向かうよ")
  (go-to-spot "/eng2/7f/room73B2-sink-front1" (make-coords :pos #f(100 -1000 0)))
  (unix:sleep 1)
  (send *ri* :speak-jp "キッチンについたよ" :wait t)
  (send *fetch* :head :neck-y :joint-angle -30)
  (send *fetch* :head :neck-p :joint-angle 40)
  (send *fetch* :torso_lift_joint :joint-angle 400)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 500)
  (unix:sleep 7) ;; wait for camera image stabilization
  (call-mail-service *place-name* *mail-title* *from-address* *to-address*) ;; calling send mail service
  (send *ri* :speak-jp "かしゃっ")
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "ドックに戻るね" :wait t)
  ;; auto docking
  (auto-dock))

(go-to-kitchen)
