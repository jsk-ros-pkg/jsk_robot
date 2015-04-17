#!/usr/bin/env roseus

(ros::roseus "greeting")
(ros::load-ros-manifest "peppereus")
(load "package://peppereus/pepper-interface.l")
(load "package://jsk_201504_miraikan/euslisp/pepper-motions.l")
(setq *pepper* (pepper))
(setq *ri* (instance pepper-interface :init))


;; 音声ファイルを再生し、動く関数
(defun playAudioFile_move (file angle-list time-list)
  (ros::ros-info "speak")
  (send *ri* :angle-vector-sequence angle-list time-list)
  (send *ri* :play-audio-file file)
  )

(defun demo()
  (send *ri* :set-maste-volume 100)
  (send *ri* :stop-grasp)
  (playAudioFile_move "/audio_file/M1.mp3" *lift_arm_and_bow* (list 1000 100 100 100 100 100 1000))
  (playAudioFile_move "/audio_file/M2.mp3" *pepper_can_cannot* (list 100 100 100 100 100 100 100 100 100  100 150 100 150 100 100 100 100 1000))
  (playAudioFile_move "/audio_file/M3.mp3" *pepper_run* (list 200 200 200  200 200 200 200 100 100 100 100 100 100))
  (playAudioFile_move "/audio_file/M4.mp3" *show_the_camera* (list 100 100 100 100 100 100 100 100 ))
  (playAudioFile_move "/audio_file/M5.mp3" *speaker_touch_sensor* (list 100 100 100 100 100 100 100 100 100 100))
  (playAudioFile_move "/audio_file/M6.mp3" (list  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *touch_CPU* *touch_CPU* *touch_CPU* *reset_pose*) (list 100 100 100 100 100 100 100 100))
  (playAudioFile_move "/audio_file/M7.mp3" *look_around_hope* (list 100 100 100 100 100 100  100 100 100))
  (playAudioFile_move "/audio_file/M8.mp3" *lift_arm_and_bow* (list 1000 100 100 100 100 100 1000))
  )

(demo)

