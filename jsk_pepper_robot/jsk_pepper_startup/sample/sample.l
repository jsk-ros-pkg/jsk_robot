#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_pepper_startup")
(ros::roseus "pepper_client_sample")

;; 
(ros::advertise "/speech" std_msgs::String 1)
(defun speak-jp (str &rest args)
  (let ((msg (instance std_msgs::String :init)))
    (send msg :data (funcall #'format nil str args))
    (ros::publish "/speech" msg))
  )

(defun get-installed-behaviors ()
  (let ((ret))
    (setq ret (ros::service-call "get_installed_behaviors" (instance nao_msgs::GetInstalledBehaviorsRequest :init)))
    (send ret :behaviors)
  ))

;; (run-behavior "pepper_tongue_twister_sample")
;; (run-behavior "pepper_dialog_sample")
(setq *run-behavior* (instance ros::simple-action-client :init "run_behavior" nao_msgs::RunBehaviorAction))
(defun run-behavior (behavior)
  (let (goal)
    (setq goal (instance nao_msgs::RunBehaviorActionGoal :init))
    (send goal :goal :behavior behavior)  
    (send *run-behavior* :send-goal goal)
    ))

(defun stop-behavior ()
  ;;(send *run-behavior* :cancel-all-goals) ;; this may break something???? not sure
  )


;; (recognize-word (list "Konnichiwa" "Sayounara" "Pepper"))
(defun recognize-word (vocabulary &optional (threshold 0.3) (wait 5))
  (let (tm req str)
    (setq str (car vocabulary))
    (dolist (v (cdr vocabulary))
      (setq str (format nil "~A / ~A" str v)))
    (setq req (instance dynamic_reconfigure::ReconfigureRequest :init))
    (send req :config :strs (list (instance dynamic_reconfigure::StrParameter :init :name "vocabulary" :value str)))
    (ros::service-call "nao_speech/set_parameters" req)
    ;;
    (setq *word-recognized* nil)
    (ros::subscribe "word_recognized" nao_msgs::WordRecognized
		    #'(lambda (msg)
			(if (> (elt (send msg :confidence_values) 0) threshold)
			    (setq *word-recognized* (send msg :words)))))
    (call-empty-service "start_recognition")
    (setq tm (ros::time-now))
    (while (and (< (send (ros::time- (ros::time-now) tm) :to-sec) wait)
		(null *word-recognized*))
      (ros::spin-once)
      (ros::ros-info "Recognizing.... ~A" str)
      (ros::sleep))
    (call-empty-service "stop_recognition")
    (car *word-recognized*)
    ))


;(ros::rate 10)
;(do-until-key
; (ros::spin-once)
; (ros::sleep)

(ros::advertise "/pepper_tweet" std_msgs::String 1)
(defun tweet (str)
  (ros::publish "/pepper_tweet" (instance std_msgs::String :init :data str)))

(defun demo1 ()
  (let (ret)
    (speak-jp "こんにちは")
    (setq ret (recognize-word (list "Konnichiwa" "Sayounara" "Pepper" "Shashin")))
    (unless ret
      (speak-jp "おーい")
      (setq ret (recognize-word (list "Konnichiwa" "Sayounara" "Pepper" "Shashin") :wait 3)))
    (ros::ros-info "Recognized ~A~%" ret)
    (cond ((equal ret "Konnichiwa")
	   (speak-jp "ははは")
	   )
	  ((equal ret "Sayounara")
	   (speak-jp "そんなこといわないでよー")
	   )
	  ((equal ret "Pepper")
	   (speak-jp "はーい")
	   )
	  ((equal ret "Shashin")
	   (tweet "こんにちわ")
	   )
	  ((null ret)
	   (speak-jp "だれもいないのかなぁ"))
	  (t
	   (speak-jp "なんかバグった")))
    ))
