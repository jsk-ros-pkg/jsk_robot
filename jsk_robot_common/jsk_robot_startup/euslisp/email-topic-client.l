(ros::load-ros-manifest "jsk_robot_startup")

(require :base64 "lib/llib/base64.l")

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
  (send msg :body
        (mapcar #'(lambda (e)
                    (cond ((derivedp e image-2d)
                           (if (> (length (send e :entity)) (* 8 8192))
                               (warning-message 3 "The size of img is too large (~A)~%You may encounter 'too long string' error, see https://github.com/euslisp/EusLisp/issues/2 for more info~%" (length (send e :entity))))
                           (instance jsk_robot_startup::EmailBody :init :type "img"
                                     :img_data (base64encode (send e :entity))
                                     :img_size 100))
                          ((probe-file e)
                           (instance jsk_robot_startup::EmailBody :init :type "img" :file_path e :img_size 100))
                          ((and (> (length e) 0) ;; check html
                                (eq (elt e 0) #\<)
                                (eq (elt e (1- (length e))) #\>))
                           (instance jsk_robot_startup::EmailBody :init :type "html" :message e))
                          (t
                           (instance jsk_robot_startup::EmailBody :init :type "text" :message (format nil "~A" e)))))
                (if (atom body) (list body) body)))
  (send msg :sender_address sender-address)
  (send msg :receiver_address receiver-address)
  (send msg :smtp_server smtp-server)
  (send msg :smtp_port smtp-port)
  (send msg :attached_files attached-files)
  (ros::publish "email" msg)
  (ros::spin-once)
  )
