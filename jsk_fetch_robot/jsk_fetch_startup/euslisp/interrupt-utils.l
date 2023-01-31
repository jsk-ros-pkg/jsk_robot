#!/usr/bin/env roseus

(ros::roseus-add-msgs "speech_recognition_msgs")
(ros::roseus-add-srvs "speech_recognition_msgs")


(defclass speech-recognition-client
    :slots (sub speech-msg))

(defmethod speech-recognition-client
  (:init (topic-name)
   (setq sub (ros::subscribe
              topic-name speech_recognition_msgs::SpeechRecognitionCandidates
              #'send self :callback)))
  (:callback (msg)
             (setq speech-msg msg))
  (:result ()
           (when (and (not (null speech-msg)) (> (length (send speech-msg :transcript)) 0))
             (car (send speech-msg :transcript)))))


(defun check-reaction-words (text)
  (let ((affirmative-words (list
                            "はい"
                            "いいよ"
                            "そうです"))
        (stop-words (list "おつかれ"
                          "終了"
                          "終わり"
                          "もういいよ"
                          "ストップ"))
        (interrupt-words (list "どいて"
                               "ストップ"
                               "やめて"
                               "あっちいって"))
        (skip-words (list "スキップ"
                          "飛ばして")))
    (when (find text stop-words :test #'string=)
      (return-from check-reaction-words :stop))
    (when (find text interrupt-words :test #'string=)
      (return-from check-reaction-words :interrupt))
    (when (find text skip-words :test #'string=)
      (return-from check-reaction-words :skip))
    (when (find text affirmative-words :test #'string=)
      (return-from check-reaction-words :yes))
          ))

(defun wait-reaction (&key (timeout 10000))
  (let ((speech-recognition (instance speech-recognition-client :init "/speech_to_text_google"))
        speech-recognition-result
        (finishtm (if timeout (ros::time-now))))
    (ros::ros-info "timeout: ~A" timeout)
    (when finishtm
      (setq finishtm (ros::time+ finishtm (ros::time (/ timeout 1000.0)))))
    (while (not (and finishtm
                     (< (send (ros::time- finishtm (ros::time-now)) :to-Sec) 0)))
      ;; (unix::usleep (* 50 1000))
      (ros::spin-once)
      (unix::usleep 500)
      (setq speech-recognition-result (send speech-recognition :result))
      (cond
        ((eql (check-reaction-words speech-recognition-result) :yes)
         (return-from wait-reaction :yes)
         (setq interrupt nil))
        ((eql (check-reaction-words speech-recognition-result) :stop)
         (ros::ros-warn "exiting...")
         (return-from wait-reaction :stop))
        ((eql (check-reaction-words speech-recognition-result) :skip)
         (ros::ros-warn "skipping...")
         (return-from wait-reaction :skip))
        ((eql (check-reaction-words speech-recognition-result) :interrupt)
         (ros::ros-warn "interrupting...")
         (return-from wait-reaction :interrupt))
        ))
    (ros::ros-info "timeout")
    )
  )

(defun after-state-interrupt ()
  (ros::ros-info "after-state")
  (let* ((reaction (wait-reaction :timeout 10000)))
    (unix::sleep 1)
    (ros::ros-info "wait")
    (ros::ros-info "reaction: ~A" reaction)
    (cond
      ((eql reaction :yes) (return-from after-state-interrupt t))
      ((eql reaction :stop) (sys::exit 0))
      ((eql reaction :interrupt)
       (unix::sleep 1)
       (if (eql (wait-reaction :timeout 1000000) :yes) (return-from after-state-interrupt t)))
      ((eql reaction :skip) (return-from after-state :skip)))
    )
  )
