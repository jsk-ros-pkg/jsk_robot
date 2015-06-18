#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus_remote")
(ros::roseus "remote_roseus_client")

(ros::advertise "ocs_to_fc_eus/input" roseus_remote::RawCommand)
(setq *returnedp* nil)
(setq *result* nil)
(defun command-cb (msg)
  (let ((cmd-str (send msg :data)))
    (setq *result* (read-from-string 
                    ;; Remove null characters
                    (string-right-trim '(0) (send msg :data))))
    (setq *returnedp* t))
  )

(defun wait-return-from-server ()
  (setq *returnedp* nil)
  (while (and (not *returnedp*) (ros::ok))
    (ros::sleep)
    (ros::spin-once))
  *result*)

(ros::subscribe "ocs_from_fc_eus/output" roseus_remote::RawCommand #'command-cb)

(defmacro send-to-server (sexp)
  (let ((str (gensym))
        (msg (gensym))
        (ss (gensym))
        (res-str (gensym)))
    `(let ((,msg (instance roseus_remote::RawCommand :init))
           (,ss (make-string-output-stream 1024))
           (,res-str (make-array 1024 :initial-element 0 :element-type :char)))
       (print ',sexp ,ss)
       (replace ,res-str (get-output-stream-string ,ss) :start2 0)
       (send ,msg :data ,res-str)
       (ros::publish "ocs_to_fc_eus/input" ,msg)
       (ros::ros-info "send ~A" ',sexp)
       (wait-return-from-server))
    ))
(defmacro $ (sexp)
  `(send-to-server ,sexp))
(ros::rate 5)
