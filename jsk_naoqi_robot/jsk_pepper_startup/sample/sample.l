#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_pepper_startup")
(ros::roseus "pepper_client_sample")

;;  see http://doc.aldebaran.com/2-1/naoqi/audio/altexttospeech-tuto.html#using-tags-for-voice-tuning for tag
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
    (setq str (car vocabulary))
    (dolist (v (cdr vocabulary))
      (setq str (format nil "~A / ~A" str v)))
    (setq req (instance dynamic_reconfigure::ReconfigureRequest :init))
    (send req :config :strs (list (instance dynamic_reconfigure::StrParameter :init :name "vocabulary" :value str)))
    (ros::ros-info "wait for nao_speech/set_parameters")
    (ros::wait-for-service "nao_speech/set_parameters")
    (ros::service-call "nao_speech/set_parameters" req)
    ;;
    (ros::ros-info "wait for start_recognition")
    (ros::wait-for-service "start_recognition")
    (call-empty-service "start_recognition")

    (setq *word-recognized* nil)
    (ros::subscribe "word_recognized" nao_msgs::WordRecognized
		    #'(lambda (msg)
			(ros::ros-info "Recognized ~A (~A)" (send msg :words) (send msg :confidence_values))
			(if (> (elt (send msg :confidence_values) 0) threshold)
			    (setq *word-recognized* (send msg :words)))))

    (setq tm (ros::time-now))
    (ros::rate 2)
    (while (and (< (send (ros::time- (ros::time-now) tm) :to-sec) wait)
		(null *word-recognized*))
      (ros::spin-once)
      (ros::ros-info "Recognizing.... ~A" str)
      (ros::sleep))

    (ros::unsubscribe "word_recognized")

    (ros::ros-info "wait for stop_recognition")
    (ros::wait-for-service "stop_recognition")
    (call-empty-service "stop_recognition")
    (car *word-recognized*)
    ))

(defun learn-face
  (name)
  (let ((req (instance nao_interaction_msgs::LearnFaceRequest :init))
	res)
    (send req :name :data name)
    (speak-jp "こんにちは")
    (speak-jp "ぼく　ペッパー")
    (speak-jp "あなたのおなまえ覚えます")
    (setq res (ros::service-call "/nao_vision/face_detection/learn_face" req))
    (ros::ros-warn "learn face ~A~%" (if (send res :result :data) "success" "fail"))
    (speak-jp "~Aさん　よろしくね" name)
    )
  )

(defun forget-person
  (name)
  (let ((req (instance nao_interaction_msgs::LearnFaceRequest :init))
	res)
    (send req :name :data name)
    ;;(speak-jp "~Aさん" name)
    (speak-jp "残念だけど~Aさんの顔を忘れるよ" name)
    (setq res (ros::service-call "/nao_vision/face_detection/forget_person" req))
    (ros::ros-warn "forget person ~A~%" (if (send res :result :data) "success" "fail"))
    (speak-jp "また遊んでね"))
  )


(defun detect-face (&key (threshold 0.3) (wait 5))
  (let (tm req str)
    (ros::ros-info "wait for face_detection/enable")
    (ros::wait-for-service "/nao_vision/face_detection/enable")
    (call-empty-service "/nao_vision/face_detection/enable")

    (setq *face-recognized* nil)
    (setq *face-label* nil)


    (ros::subscribe "/nao_vision/faces_detected" nao_interaction_msgs::FaceDetected
		    #'(lambda (msg)
			(ros::ros-info "Detected ~A (~A)" (send msg :face_id :data) (send msg :score_reco :data))
			(setq *face-recognized* (send msg :face_id :data))
			(setq *face-label* (if (< threshold (send msg :score_reco :data)) (send msg :face_label :data) nil))
			))

    (setq tm (ros::time-now))
    (ros::rate 2)
    (while (and (< (send (ros::time- (ros::time-now) tm) :to-sec) wait)
		(null *face-recognized*))
      (ros::spin-once)
      (ros::ros-info "Detecting.... ~A" str)
      (ros::sleep))
    (ros::unsubscribe "/nao_vision/faces_detected")


    (ros::ros-info "wait for face_detection/disable")
    (ros::wait-for-service "/nao_vision/face_detection/disable")
    (call-empty-service "/nao_vision/face_detection/disable")
    ;;(when *face-label* (speak-jp "~Aさん" *face-label*))
    (when *face-label* (tweet "~Aさんに会ったよ" *face-label*))
    *face-recognized*
    ))

(ros::advertise "/pepper_tweet" std_msgs::String 1)
(defun tweet (str &rest args)
  (ros::publish "/pepper_tweet" (instance std_msgs::String :init :data (apply #'format nil str args))))

;(ros::rate 10)
;(do-until-key
; (ros::spin-once)
; (ros::sleep)


(ros::advertise "/tweet" std_msgs::String 1)
(defun fast_tweet (str &rest args)
  (let ((text)
	(timestamp0 0) (timestamp1 0)
	(filename "/tmp/pepper_camera.png")
	tm)
    (setq text (apply #'format nil str args))
    (ros::service-call "/image_saver/save" (instance std_srvs::EmptyRequest :init))
    (setq tm (ros::time-now))
    (ros::ros-info "~A timestamp ~A ~A" filename timestamp0 timestamp1)
    (ros::rate 2)
    (while (and (< (send (ros::time- (ros::time-now) tm) :to-sec) 10)
		(= timestamp0 timestamp1))
      (if (probe-file filename) (setq timestamp1 (file-write-date filename)))
      (ros::ros-info "~A timestamp ~A ~A" filename timestamp0 timestamp1)
      (ros::sleep))
    (ros::ros-info "save image")
    ;; tweet
    (ros::publish "/tweet" (instance std_msgs::String :init :data (format nil "~A ~A" str filename)))
    ))

(defun start-imu ()
  (ros::subscribe "imu" sensor_msgs::Imu
		  #'(lambda (msg)
		      (setq *imu-rotation* (ros::tf-quaternion->rot (send msg :orientation)))
		      (setq *imu-orientation* (v. (matrix-column *imu-rotation* 2) #f(0 0 1)))
		      (ros::ros-info "imu ~A ~A" (matrix-column *imu-rotation* 2) *imu-orientation*)
		      (cond ((< *imu-orientation* 0.99)
			     (speak-jp "やめてくださいよー")
			     (fast_tweet "やめてくださいよー")
			     (unix:sleep 2)
			     )
			    ((< *imu-orientation* 0.98)
			     (tweet "だれかたすけてー")
			     (fast_tweet "だれかたすけてー")
			     (unix:sleep 5)
			     )))
		  1))
(defun stop-imu ()
  (ros::unsubscribe "imu"))


(defun demo1 ()
  (let ((ret) (voc (list "こんにちは" "さようなら" "Pepper" "あれ" "ぼく" "げんき" "ろぼっと" "しんぼう")))
    ;;(setq voc (list "Konnichiwa" "Sayounara" "Pepper"))
    (do-until-key 
     (start-imu)
     (when (detect-face :wait 30)
       (speak-jp "こんにちは")
       (setq ret (recognize-word voc :wait 15 :threshold 0.1))
       (unless ret
	 (speak-jp "おーい")
	 (setq ret (recognize-word voc :wait 10 :threshold 0.1)))
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
       (unix:sleep 10))
     (speak-jp "うふふ")
     (stop-imu)
     (run-behavior (elt (get-installed-behaviors) (+ (random 300) 15)) :wait t)
     ) ;; do-until key
    ))

(defun detect-face-demo
  ()
  (do-until-key
   (detect-face :wait 30)
   (if *face-label*
     (speak-jp "~Aさん　ひさしぶり" *face-label*)
     (progn
       (speak-jp "はじめまして")
       (speak-jp "僕はペッパー")
       (speak-jp "名前を教えて")
       )
     )
   (unix:sleep 30)
   )
  )
   


(ros::ros-warn "usage")
(ros::ros-warn "(demo1)")
(ros::ros-warn "(detect-face-demo)")
(ros::ros-warn "(learn-face \"name\")")
(ros::ros-warn "(forget-person \"name\")")
