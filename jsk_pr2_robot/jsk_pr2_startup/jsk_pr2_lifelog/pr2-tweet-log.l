#!/usr/bin/env roseus
;; pr2-tweet-log.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "pr2_tweet_log")
(ros::load-ros-manifest "jsk_demo_common")

(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")
(require :tweet-client "package://jsk_robot_startup/lifelog/tweet_client.l")
(defparameter *mongo-database* "jsk_robot_lifelog")
(defparameter *mongo-collection* "pr1012")
(defparameter *json-parse-key-function* #'string->keyword)
(setq json::*tzoffset* -8)
(defvar *tweet* t)

(defclass log-tweet-worker
  :super propertied-object
  :slots (name check-cb tweet-cb check-result last-check-date))

(defmethod log-tweet-worker
  (:init (&key ((:name n)) ((:check-cb ccb)) ((:tweet-cb tcb)))
   (setq name n)
   (setq check-cb ccb)
   (setq tweet-cb tcb))
  (:name (&optional n)
   (if n (setq name n) name))
  (:last-check (&optional d)
   (if d (setq last-check-date d) last-check-date))
  (:check (&key (timeout 300))
   (ros::ros-info "[~A] checking..." name)
   (setq last-check-date (now))
   (setq check-result (funcall check-cb self))
   check-result)
  (:tweet ()
   (ros::ros-info "[~A] tweeting..." name)
   (let ((str (funcall tweet-cb self check-result)))
     (setq check-result nil)
     (unless (stringp str)
       (ros::ros-warn "[~A] expected ~A is not string. skipping" name str)
       (return-from :tweet nil))
     str))
  ) ;; end of defmethod


(defclass log-tweet-manager
  :super propertied-object
  :slots (workers check-start-date))

(defmethod log-tweet-manager
  (:init ()
   (push (now) check-start-date)
   (setq workers (make-hash-table)))
  (:add-worker (w)
   (let ((work-key (intern (send w :name) *keyword-package*)))
     (if (gethash work-key workers)
         (error "duplicate worker ~A" (gethash work-key workers))
         (setf (gethash work-key workers) w))))
  (:add-workers (w-list)
   (dolist (w w-list) (send self :add-worker w)))
  (:remove-worker (name)
   (let ((work-key (if (keywordp name) name (intern name *keyword-package*))))
     (remhash work-key workers)))
  (:last-check ()
   (car check-start-date))
  (:start ()
    (do-until-key
     (let ((with-image t))
        (ros::ros-info "check: ~A" (now))
        (dolist (k (send workers :list-keys))
          (let ((w (gethash k workers)) ret)
            (when (send w :check)
              (setq ret (send w :tweet))
              (when ret
                (if *tweet*
                    (tweet-string ret :warning-time nil :with-image with-image :speek nil :image-wait 10))
                (warn "tweeted: ~A...~%" (subseq ret 0 100))
                (setq with-image nil)
                ))))
       (ros::sleep)
      )))
  (:latest-enough-p (m)
   (when m
     (ros::ros-info "now: ~A insert: ~A" (now) (cdr (assoc :inserted_at m)))
     (< (date- (now)
               (cdr (assoc :inserted_at m)))
        *latest-enough-sec*)))
  ) ;; end of defmethod

;; utility functions
(defvar *latest-enough-sec* 20.0)
(defun date- (dl dr)
  (- (send dl :compute-seconds)
     (send dr :compute-seconds)))

(defun latest-enough-p (m)
  (when m
    (< (date- (now)
              (cdr (assoc :inserted_at m)))
       *latest-enough-sec*)))


(setq *manager* (instance log-tweet-manager :init))
(send *manager* :add-worker
      (instance log-tweet-worker :init
                :name "action_success_rate"
                :check-cb #'(lambda (self)
                              (let* ((last-check-date (send self :last-check))
                                     (q (car (mongo::query jsk_demo_common::FunctionEvent
                                                           :meta '((:status . "start"))
                                                           :sort '((:$natural . -1))
                                                           :single t))))
                                  (when (send *manager* :latest-enough-p (cdr q)) q)))
                :tweet-cb #'(lambda (self q)
                              (let* ((same-funcs (mongo::query jsk_demo_common::FunctionEvent
                                                              :query `((:name . ,(send (car q) :name)))))
                                     (success-rate
                                       (/ (* 100.0 (count-if #'(lambda (x)
                                                                 (string= "success"
                                                                          (cdr (assoc :status (cdr x)))))
                                                             same-funcs))
                                          (count-if #'(lambda (x)
                                                        (string= "start"
                                                                 (cdr (assoc :status (cdr x)))))
                                                    same-funcs)))
                                     fmt)
                                (setq fmt (format nil "Now I'm executing ~A."
                                                  (send (car q) :name)))
                                (if same-funcs
                                  (setq fmt
                                        (concatenate string fmt
                                          (format nil "Success rate of this action is ~A%, ~A"
                                                  success-rate
                                                  (cond
                                                   ((< success-rate 20.0) "Oh, no good at all...")
                                                   ((< success-rate 40.0) "Hmm, not good...")
                                                   ((< success-rate 60.0) "Well, not so bad..")
                                                   ((< success-rate 80.0) "Good!")
                                                   ((= success-rate 100.0) "Perfect!!!")
                                                   (t "Excellent!!")))))
                                  (setq fmt (concatenate string fmt
                                             "This is the first time to do.")))
                                  ))
                ))

(send *manager* :add-worker
      (instance log-tweet-worker :init
                :name "action_many_times_today"
                :check-cb #'(lambda (self)
                              (let ((q (car (mongo::query jsk_demo_common::FunctionEvent
                                                          :meta '((:status . "start"))
                                                          :sort '((:$natural . -1))
                                                          :single t))))
                                (when (send *manager* :latest-enough-p (cdr q))
                                  (let* ((today (send (cdr (assoc :inserted_at (cdr q))) :today))
                                         (todays-actions
                                          (mongo::query jsk_demo_common::FunctionEvent
                                                        :query `((:name . ,(send (car q) :name)))
                                                        :meta `((:status . "start")
                                                                (:inserted_at
                                                                 . ((:$gt . ,today)))))))
                                    (when (> (length todays-actions) 5) todays-actions)))))
                :tweet-cb #'(lambda (self q)
                              (format nil "This is ~A times I do ~A today..."
                                      (length q)
                                      (send (caar q) :name)))))

(send *manager* :add-worker
      (instance log-tweet-worker :init
                :name "failure_complaint"
                :check-cb #'(lambda (self)
                              (let ((q (car (mongo::query jsk_demo_common::FunctionEvent
                                                          :meta '((:status . "fail"))
                                                          :sort '((:$natural . -1))
                                                          :single t))))
                                (when (send *manager* :latest-enough-p (cdr q))
                                    (let* ((recent-date (send (cdr (assoc :inserted_at (cdr q)))
                                                              :subtract (instance interval-time :make
                                                                                  :day 3)))
                                           (recent-failures (mongo::query jsk_demo_common::FunctionEvent
                                                                          :query `((:name . ,(send (car q) :name)))
                                                                          :meta `((:status . :fail)
                                                                                  (:inserted_at
                                                                                   . ((:$gt . ,recent-date)))))))
                                      (ros::ros-info "recent-failures: ~A" recent-failures)
                                      (when (> (length recent-failures) 3) q)))))
                :tweet-cb #'(lambda (self q)
                              (format nil "~A failed. Hmm, Things are not going too well recently..."
                                      (send (car q) :name)))))

(send *manager* :add-worker
      (instance log-tweet-worker :init
                :name "context_change"
                :check-cb #'(lambda (self)
                              (let ((querys (mongo::query jsk_demo_common::FunctionEvent
                                                            :meta '((:status . "start"))
                                                            :single nil
                                                            :sort '((:$natural . -1))))
                                    query1 query2)
                                (setq query1 (car querys))
                                (setq query2 (cadr querys))
                                (when (and
                                       (not (string=
                                             (send (send (car query1) :header) :context)
                                             (send (send (car query2) :header) :context)))
                                       (send *manager* :latest-enough-p (cdr query1)))
                                  query1)))
                :tweet-cb #'(lambda (self q)
                              (let* ((all-results (mongo::query jsk_demo_common::FunctionEvent
                                                                :query `((:header.context . ,(send (send (car q) :header) :context)))))
                                     (success-results (remove-if-not #'(lambda (x)
                                                                         (equal (cdr (assoc :status (cdr q)))
                                                                             "success"))
                                                                     all-results)))
                                (when (> (length success-results) 0)
                                  (format nil "Success rate of ~A is ~A percent."
                                          (send (send (car q) :header) :context)
                                          (/ (* 100.0 (length success-results)) (length all-results))))))
                ))


(ros::rate 0.05) ;; [Hz]
(send *manager* :start)


(provide :pr2-tweet-log) ;; end of pr2-tweet-log.l
