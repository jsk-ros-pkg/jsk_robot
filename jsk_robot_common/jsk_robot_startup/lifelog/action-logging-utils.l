;; action-logging-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_robot_startup")
(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")

(defparameter *task-name* :none)
(defparameter *task-id* nil)

;; task utility
(defun start-task (task-name &optional (task-id))
  (setq *task-name* task-name)
  (unless (ros::get-param "/task_id" nil)
    (ros::set-param "/task_id"
      (if (and task-id (stringp task-id))
          task-id
          (string (read (piped-fork "uuidgen -t"))))))
  (setq *task-id* (ros::get-param "/task_id"))
  (ros::ros-info "start task ~A (id: ~A)" *task-name* *task-id*))

(defun end-task ()
  (ros::ros-info "end task ~A (id: ~A)" *task-name* *task-id*)
  (setq *task-name* :none)
  (setq *task-id* nil))

(defun insert-action-event (name args status)
  (ros::ros-debug "inserted at db (id: ~A)"
   (mongo::insert
    (instance jsk_robot_startup::ActionEvent :init
              :header (instance std_msgs::Header :init :stamp (ros::time-now))
              :name name :args args :status (string status))
    :meta `((:task_name . ,*task-name*)
            (:task_id . ,*task-id*)))))

;; action utility
(defmacro defaction (name args &rest form)
  `(progn
     (defun ,(read-from-string (format nil "~A-orig" name)) ,args
       (block ,name ,@form))
     (defun ,name (&rest args)
       (let ((log-func-name (string-downcase ',name))
             (log-func-args (mapcar #'(lambda (x) (format nil "~A" x)) args))
             log-func-result log-func-result-orig)
         (ros::ros-debug "start ~A" log-func-name)
         (insert-action-event log-func-name log-func-args :start)
         (setq log-func-result
               (catch :action-failure
                 (setq log-func-result-orig
                       (apply #',(read-from-string (format nil "~A-orig" name)) args))
                 (unless (or (null log-func-result-orig)
                             (and (listp log-func-result-orig)
                                  (some #'identity log-func-result-orig)))
                   (throw :action-failure :failure))
                 :success))
         (ros::ros-debug "return-from ~A with ~A" log-func-name log-func-result)
         (insert-action-event log-func-name log-func-args log-func-result)
         log-func-result-orig))))

(defmacro declare-logging-method (cls name &key (prefunc #'identity))
  `(defmethod ,cls
     (,(read-from-string (format nil "~A-orig" name)) (&rest args) (send-super* ,name args))
     (,name (&rest args)
       (let ((log-method-name (format nil "~A:~A" (send ,cls :name) ,name))
             (log-method-args (mapcar #'(lambda (x) (format nil "~A" x)) args))
             log-method-result log-method-result-orig)
         (ros::ros-debug "start ~A" log-method-name)
         (insert-action-event log-method-name log-method-args :start)
         (setq log-method-result
               (catch :action-failure
                 (setq log-method-result-orig (funcall ,prefunc (send-super* ,name args)))
                 (unless (or (null log-method-result-orig)
                             (and (listp log-method-result-orig)
                                  (some #'identity log-method-result-orig)))
                   (throw :action-failure :failure))
                 :success))
         (ros::ros-debug "return-from ~A with ~A" log-method-name log-method-result)
         (insert-action-event log-method-name log-method-args log-method-result)
         log-method-result-orig))))


(provide :action-logging-utils) ;; end of action-logging-utils.l
