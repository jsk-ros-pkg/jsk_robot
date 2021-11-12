(ros::load-ros-manifest "jsk_robot_startup")

(defun init-mail ()
  (ros::advertise "email" jsk_robot_startup::Email 1)
  (ros::spin-once)
  )

(defun send-mail (subject
                  receiver-address
                  body
                  &key
                  sender-address
                  smtp-server
                  smtp-port
                  attached-files)
  (setq msg (instance jsk_robot_startup::Email :init))
  (send msg :header :stamp (ros::time-now))
  (send msg :subject subject)
  (send msg :body body)
  (send msg :sender_address sender-address)
  (send msg :receiver_address receiver-address)
  (send msg :smtp_server smtp-server)
  (send msg :smtp_port smtp-port)
  (send msg :attached_files attached-files)
  (ros::publish "email" msg)
  (ros::spin-once)
  )
