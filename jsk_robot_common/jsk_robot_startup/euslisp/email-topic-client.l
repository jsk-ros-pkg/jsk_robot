(ros::load-ros-manifest "jsk_robot_startup")

(defun send-mail (subject
                  receiver-address
                  body
                  &optional
                  &key
                  sender-address
                  smtp-server
                  smtp-port
                  attached-file)
  (ros::advertise "email" jsk_robot_startup::Email 1)
  (setq msg (instance jsk_robot_startup::Email :init))
  (send msg :header :stamp (ros::time-now))
  (if subject (send msg :subject subject) nil)
  (if body (send msg :body body) nil)
  (if sender-address (send msg :sender_address sender-address) nil)
  (if receiver-address (send msg :receiver_address receiver-address) nil)
  (if smtp-server (send msg :smtp_server smtp-server) nil)
  (if smtp-port (send msg :smtp_port smtp-port) nil)
  (if attached-file (send msg :attached_file attached-file) nil)
  (ros::publish "email" msg)
  (ros::spin-once)
  )
