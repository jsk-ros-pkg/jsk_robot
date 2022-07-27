#!/usr/bin/env roseus

;; load package
(ros::load-ros-package "peppereus")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "naoqi_bridge_msgs")

;; load simple model for fast-boot
(require :pepper "package://peppereus/pepper-simple.l")

;; load files
(require :pepper-interface "package://peppereus/pepper-interface.l")

;; base64
(require :base64 "lib/llib/base64.l")

;; var
(defvar *process-rate* 1)
;; (setq *touch_ears* #f(20.0 -2.0 -30.0 -15.0 20.0 -70.0 -88.0 -40.0 -15.0 -20.0 70.0 88.0 40.0 0.0 30.0))
;; (setq *open_pepper* #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0)) 
;; (setq *open-arms* #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 0.0 -10.0))
;; (setq *wondering-pose* #f(2.0 -15.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -10.0 -5.0))
;; (setq *reset-pose* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))
;; (setq *raise-right-hand* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0))
;; (setq *ok_pose* #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0))

;; (list *touch_ears* *open_pepper* *open_pepper* *reset-pose*)
(defvar *pose1* (list #f(20.0 -2.0 -30.0 -15.0 20.0 -70.0 -88.0 -40.0 -15.0 -20.0 70.0 88.0 40.0 0.0 30.0) #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time1* (list 2000 2000 4000 2000))

;; (list *open-arms* *wondering-pose* *wondering-pose* *reset-pose*)
(defvar *pose2* (list #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 0.0 -10.0) #f(2.0 -15.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -10.0 -5.0) #f(2.0 -15.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -10.0 -5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time2* (list 3000 2000 3000 2000))

;; (list *raise-right-hand* *raise-right-hand* *raise-right-hand* *reset-pose*)
(defvar *pose3* (list #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 10.0 -20.0 70.0 80.0 -70.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time3* (list 3000 1000 1000 2000))

;; (list *ok_pose* *ok_pose* *ok_pose* *reset-pose*)
(defvar *pose4* (list #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0) #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0) #f(2.0 -2.0 -5.0 -40.0 10.0 -20.0 -70.0 -40.0 -40.0 -10.0 20.0 70.0 40.0 0.0 -20.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time4* (list 3000 2000 2000 2000))

;; interactive_robot_demo/miraikan_live/scripts/actions/small-exercise.l
;;(setq *start-pose-larm* #f(2.0 -2.0 -5.0 -70.0 40.0 -20.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))
;;(setq *end-pose-larm* #f(2.0 -20.0 -5.0 -70.0 20.0 -20.0 -50.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -5.0 -5.0))
;;(setq *start-pose-rarm #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -40.0 20.0 20.0 40.0 0.0 0.0))
;;(setq *end-pose-rarm #f(2.0 20.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -20.0 20.0 50.0 40.0 5.0 5.0))
(defvar *pose5* (list #f(2.0 -2.0 -5.0 -70.0 40.0 -20.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -20.0 -5.0 -70.0 20.0 -20.0 -50.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -5.0 -5.0) #f(2.0 -2.0 -5.0 -70.0 40.0 -20.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -20.0 -5.0 -70.0 20.0 -20.0 -50.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -5.0 -5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -40.0 20.0 20.0 40.0 0.0 0.0) #f(2.0 20.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -20.0 20.0 50.0 40.0 5.0 5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -40.0 20.0 20.0 40.0 0.0 0.0) #f(2.0 20.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -70.0 -20.0 20.0 50.0 40.0 5.0 5.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time5* (list 4000 4000 4000 4000 3000 4000 3000 3000 3000 3000 3000))

;; (list *nodding-pose* *nodding-pose* *reset-pose*)
(defvar *pose6* (list #f(2.0 -2.0 -5.0 110.0 35.0 -40.0 -60.0 -40.0 110.0 -35.0 40.0 60.0 40.0 0.0 15.0) #f(2.0 -2.0 -5.0 110.0 35.0 -40.0 -60.0 -40.0 110.0 -35.0 40.0 60.0 40.0 0.0 15.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)))
(defvar *time6* (list 2000 3000 1000))

;;
(defvar *reset* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))
(defvar *a* #f(2.0 -2.0 -5.0 0.0 10.0 -100.0 -70.0 60.0 0.0 -10.0 100.0 70.0 -60.0 0.0 0.0))
(defvar *d* #f(2.0 -2.0 -5.0 0.0 10.0 -110.0 -70.0 60.0 0.0 -10.0 110.0 70.0 -60.0 0.0 -10.0))
(defvar *e* #f(2.0 -2.0 -5.0 0.0 10.0 -110.0 -70.0 60.0 0.0 -10.0 110.0 70.0 -60.0 0.0 -10.0))

(defvar *img-list*
  (list
   "golden-saying-by-pepper-1.png"
   "golden-saying-by-pepper-2.png"
   "golden-saying-by-pepper-3.png"
   "golden-saying-by-pepper-4.png"
   "golden-saying-by-pepper-5.png"
   "golden-saying-by-pepper-6.png"))
(defvar *pose-list* (list *pose1* *pose2* *pose3* *pose4* *pose5* *pose6*))
(defvar *time-list* (list *time1* *time2* *time3* *time4* *time5* *time6*))
(defvar *comment-list*
  (list
   "\\vol=40\\\\vct=120\\\\rspd=80\\研究会に出て，\\vct=150\\えらいッ！\\rspd=100\\"
   "\\vct=120\\\\rspd=80\\ご飯，食べた？\\rspd=100\\"
   "\\vct=120\\\\rspd=80\\今週も，\\vct=150\\頑張ろうッ！\\rspd=100\\"
   "\\vct=120\\\\rspd=80\\コメント少ないのは，順調ってことだよッ！\\rspd=100\\"
   "\\vct=120\\\\rspd=80\\座りっぱなし，\\vct=150\\気をつけてッ！\\rspd=100\\"
   "\\vct=120\\\\rspd=80\\報告して，\\vct=150\\えらいッ！\\rspd=100\\"
   ))

;; parameter
(defparameter *continue* t)
(defparameter *count* 0)
(defparameter *last-tm* (ros::time-now))

(ros::roseus "online-meeting-enhancement-main")

(ros::subscribe "joy" sensor_msgs::Joy
                #'(lambda (msg)
                    (let ((button (send msg :buttons)))
                      (ros::ros-info "received joy button ~A" button)
                      (when (eq (elt button 3) 1)
                        (send *ri* :speak "はい")
                        (setq *continue* nil)))))

(defun start-func (args)
  (let ()
    (setq *last-tm* (ros::time-now))
    ;;
    (send *ri* :servo-on)
    (send *ri* :set-language "Japanese")
    (send *ri* :stop-grasp)
    (send *ri* :set-basic-awareness-enabled nil)
    (send *ri* :set-background-movement-enabled nil)
    (send *ri* :show-image (elt *img-list* 0))
    (send *ri* :speak (elt *comment-list* 0))
    (send *ri* :angle-vector-sequence (elt *pose-list* 0) (elt *time-list* 0))
    (send *ri* :wait-interpolation)
    (send *ri* :set-basic-awareness-enabled t)
    (send *ri* :set-background-movement-enabled t)
    ;;
    (setq *count* 1)
    (set-alist 'description "" args)
    :started))

(defun end-func (args)
  (let ()
    (send *ri* :show-image "golden-saying-by-pepper-7.png")
    (send *ri* :angle-vector-sequence (list #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 30.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 -30.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)) (list 1000 1000 1000 1000))
    (send *ri* :speak "\\vct=120\\\\rspd=80\\みなさん")
    (send *ri* :wait-interpolation)

    (send *ri* :angle-vector-sequence (list *a* *d* *a* *d* *a* *d* *a* *d* *a* *e* *reset*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 3000))
    (send *ri* :speak "\\vct=120\\\\rspd=80\\おつかれさまでしたッ！\\rspd=100\\")
    (send *ri* :wait-interpolation)
    (send *ri* :hide-image)
    (send *ri* :set-language "English")
    ;;
    (set-alist 'description "End Meeting" args)
    :finished))

(defun comment-func (args)
  (let (count str (wait 600) tm)
    ;; cleanup description/image
    (set-alist 'description "" args)
    (set-alist 'image "" args)
    (ros::ros-info "comment func, *count* = ~A" *count*)
    (setq *last-tm* (ros::time-now))
    ;; wait for 600 sec OR if someone set *coutinue* nil, return with :finished
    (while (and (ros::ok)
                *continue*
                (< (setq tm (send (ros::time- (ros::time-now) *last-tm*) :to-sec)) wait))
      (ros::ros-info "wait for move .. continue = ~A, time ~A < ~A (wait)" *continue* tm wait)
      (ros::spin-once)
      (ros::duration-sleep 1))
    (unless *continue*
      (return-from comment-func :finished))
    (setq *last-tm* (ros::time-now))
    ;;
    (setq count (mod *count* (length *img-list*)))
    ;;
    (send *ri* :set-basic-awareness-enabled nil)
    (send *ri* :set-background-movement-enabled nil)
    (send *ri* :show-image (elt *img-list* count))
    (send *ri* :speak (elt *comment-list* count))
    (send *ri* :angle-vector-sequence (elt *pose-list* count) (elt *time-list* count))
    (send *ri* :wait-interpolation)
    ;;(send *ri* :angle-vector (send *pepper* :spread-hands-smally))
    (send *ri* :set-basic-awareness-enabled t)
    (send *ri* :set-background-movement-enabled t)
    ;;
    (incf *count*)
    (setq str (elt *comment-list* count))
    ;; trim // ... //
    (while (setq term1 (position #\\ str :count 1) term2 (position #\\ str :count 2))
      (setq str (concatenate string (subseq str 0 term1) (subseq str (+ term2 1)))))
    (ros::ros-info "comment func,  count  = ~A, str =  ~A" count str)
    (set-alist 'description str args)
    (when *image*
         ;; not to send \n ???
        (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) args)) ;; should be compressed??
    :next))


(load "package://roseus_smach/src/state-machine-ros.l")
(defun meeting-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:start :started :comment)    ;; transitions (node transition node)
             (:comment :next :comment)
             (:comment :finished :end)
             (:end :finished :goal)
             )
           '((:start    'start-func)       ;; node-to-function maps
             (:end      'end-func)
             (:comment  'comment-func)
             )
           '(:start)                    ;; initial node
           '(:goal)                     ;; goal node
           ))
    (send sm :arg-keys 'description)
    sm))

;; create robot interface
(unless (boundp '*ri*) (pepper-init))
(objects (list *pepper*))

;;
(setq *image* nil)
(defun image_cb (msg)
  (setq *image* msg))
(ros::subscribe "/pepper_robot/camera/front/image_rect_color/compressed" sensor_msgs::CompressedImage #'image_cb)

;; this does not work...
;; (unix:signal unix::sigint '(lambda-closure nil 0 0 (sig code) (setq *continue* nil)))

;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/717
;; did not work, when we subscribe image data ???

(defun ros::roseus-sigint-handler (sig code)
  (ros::ros-warn (format nil "ros::roseus-sigint-handler ~A" sig))
  (setq *continue* nil))
(unix:signal unix::sigint 'ros::roseus-sigint-handler)

;; state machine
(exec-state-machine (meeting-sm) '((description . "研究会に参加しました！")(image . "")))
(exit)
