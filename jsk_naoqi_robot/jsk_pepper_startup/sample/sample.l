#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_pepper_startup")
(ros::roseus "pepper_client_sample")

;; 
(ros::advertise "/speech" std_msgs::String 1)
(defun speak-jp (str &rest args)
  (let ((msg (instance std_msgs::String :init)))
    (send msg :data (apply #'format nil str args))
    (ros::ros-info "speech ~A" (send msg :data))
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
(defun run-behavior (behavior &key (wait nil))
  (let (goal)
    (ros::ros-info "running ~A" behavior)
    (setq goal (instance nao_msgs::RunBehaviorActionGoal :init))
    (send goal :goal :behavior behavior)  
    (send *run-behavior* :send-goal goal)
    (if wait (send *run-behavior* :wait-for-result))
    ))

(defun stop-behavior ()
  ;;(send *run-behavior* :cancel-all-goals) ;; this may break something???? not sure
  )


;; (recognize-word (list "Konnichiwa" "Sayounara" "Pepper"))
;; (recognize-word (list "こんにちは" "さようなら" "Pepper" "あれ" "ぼく" "げんき" "ろぼっと" "しんぼう"))
(defun recognize-word (vocabulary &key (threshold 0.3) (wait 5))
  (let (tm req str)
    (ros::ros-info "wait for start_recognition")
    (ros::wait-for-service "start_recognition")
    (call-empty-service "start_recognition")

    (setq str (car vocabulary))
    (dolist (v (cdr vocabulary))
      (setq str (format nil "~A / ~A" str v)))
    (setq req (instance dynamic_reconfigure::ReconfigureRequest :init))
    (send req :config :strs (list (instance dynamic_reconfigure::StrParameter :init :name "vocabulary" :value str)))
    (ros::ros-info "wait for nao_speech/set_parameters")
    (ros::wait-for-service "nao_speech/set_parameters")
    (ros::service-call "nao_speech/set_parameters" req)
    ;;
    (setq *word-recognized* nil)
    (ros::subscribe "word_recognized" nao_msgs::WordRecognized
		    #'(lambda (msg)
			(if (> (elt (send msg :confidence_values) 0) threshold)
			    (setq *word-recognized* (send msg :words)))))

    (setq tm (ros::time-now))
    (ros::rate 2)
    (while (and (< (send (ros::time- (ros::time-now) tm) :to-sec) wait)
		(null *word-recognized*))
      (ros::spin-once)
      (ros::ros-info "Recognizing.... ~A" str)
      (ros::sleep))
    (ros::ros-info "wait for stop_recognition")
    (ros::wait-for-service "stop_recognition")
    (call-empty-service "stop_recognition")
    (car *word-recognized*)
    ))


;(ros::rate 10)
;(do-until-key
; (ros::spin-once)
; (ros::sleep)

(ros::advertise "/pepper_tweet" std_msgs::String 1)
(defun tweet (str &rest args)
  (ros::publish "/pepper_tweet" (instance std_msgs::String :init :data (apply #'format nil str args))))

(defun demo1 ()
  (let ((ret) (voc (list "こんにちは" "さようなら" "Pepper" "あれ" "ぼく" "げんき" "ろぼっと" "しんぼう")))
    ;;(setq voc (list "Konnichiwa" "Sayounara" "Pepper"))
    (do-until-key 
     (speak-jp "こんにちは")
     (setq ret (recognize-word voc))
     (unless ret
       (speak-jp "おーい")
       (setq ret (recognize-word voc :wait 3)))
     (ros::ros-info "Recognized ~A~%" ret)
     (cond ((equal ret "こんにちは")
	    (speak-jp "よろしくー")
	    )
	   ((equal ret "さようなら")
	    (speak-jp "そんなこといわないでよー")
	    )
	   ((equal ret "Pepper")
	    (speak-jp "はーい")
	    )
	   ((null ret)
	    (speak-jp "だれもいないのかなぁ")
	    )
	   (t
	    (tweet "~A ですって" ret)
	    (unix:sleep 10)
	    ))
     (unix:sleep 10)
     (speak-jp "うふふ")
     (run-behavior (elt (get-installed-behaviors) (+ (random 300) 15)) :wait t)
     (unix:sleep 30)
     ) ;; do-until key
    ))
