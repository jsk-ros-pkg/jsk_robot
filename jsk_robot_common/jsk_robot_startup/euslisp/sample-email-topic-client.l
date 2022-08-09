#!/usr/bin/env roseus

(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(ros::roseus "sample-email-topic-client")
(init-mail)
(ros::spin-once)
(setq receiver-address (ros::get-param "~receiver_address"))
(setq attached-files (ros::get-param "~attached_files" nil))
(while (or (null (ros::get-num-subscribers "email"))
           (<= (ros::get-num-subscribers "email") 0))
  (ros::ros-warn "wait for email topic")
  (unix::sleep 1))

(ros::ros-info "Sending a mail to ~A" receiver-address)
(send-mail "test text mail" receiver-address "test")
(ros::ros-info "Sent a text mail")
(unix:sleep 1)

(send-mail "test html mail" receiver-address "<h1>test with html mail</h1>")
(ros::ros-info "Sent a html mail")
(unix:sleep 1)

(send-mail "test attached mail" receiver-address "test with attached image" :attached-files attached-files)
(ros::ros-info "Sent a mail with attached files ~A" attached-files)
(unix:sleep 1)

(when attached-files
  (send-mail "test image embedded mail (file)" receiver-address (append attached-files (list "test with embedded image")))
  (ros::ros-info "sent a mail with embedded image ~a" attached-files)
  (unix:sleep 1)
  ;; read png/jpeg file, but do not extract to raw image, keep original compressed data in (img . entity)
  (setq img (read-image-file (elt attached-files 0)))
  (setq (img . entity) (make-string (elt (unix:stat (elt attached-files 0)) 7))) ;; size of file
  (with-open-file (f (elt attached-files 0)) (unix:uread (send f :infd) (img . entity)))
  (send-mail "test image embedded mail (string)" receiver-address (list "test with embedded image" img))
  (ros::ros-info "Sent a mail with embedded image ~A" img)
  (unix:sleep 1)
  )

(ros::roseus "shutdown")
(exit)
