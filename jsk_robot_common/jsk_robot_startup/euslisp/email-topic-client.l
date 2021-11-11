(ros::load-ros-manifest "jsk_robot_startup")

(defun init-mail ()
  (ros::advertise "email" jsk_robot_startup::Email 1)
  (ros::spin-once)
  )

(defun send-mail (subject
                  receiver-address
                  body
                  &optional
                  &key
                  sender-address
                  smtp-server
                  smtp-port
                  attached-files)
  (setq msg (instance jsk_robot_startup::Email :init))
  (send msg :header :stamp (ros::time-now))
  (if subject (send msg :subject subject) nil)
  (if body (send msg :body body) nil)
  (if sender-address (send msg :sender_address sender-address) nil)
  (if receiver-address (send msg :receiver_address receiver-address) nil)
  (if smtp-server (send msg :smtp_server smtp-server) nil)
  (if smtp-port (send msg :smtp_port smtp-port) nil)
  (if attached-files (send msg :attached_files attached-files) nil)
  (ros::publish "email" msg)
  (ros::spin-once)
  )
