#!/usr/bin/env roseus

(ros::roseus "greeting")
(ros::load-ros-manifest "peppereus")
(load "package://peppereus/pepper-interface.l")
(load "package://jsk_201504_miraikan/euslisp/pepper-motions.l")
(ros::advertise "/speech" std_msgs::String 1)
(setq *pepper* (pepper))
(setq *ri* (instance pepper-interface :init))


;; 音声ファイルを再生し、動く関数
(defun playAudioFile_move (file angle-list time-list)
  (ros::ros-info "speak")
  (send *ri* :angle-vector-sequence angle-list time-list)
  (send *ri* :play-audio-file file)
  )

;; 発話し、動く関数
(defun speak_move (speech_msg angle-list time-list)
  (ros::ros-info "speak")
  (send *ri* :speak speech_msg)
  (send *ri* :angle-vector-sequence angle-list time-list)
  (send *ri* :wait-interpolation)
  )

(defun demo(&optional (language :ja))
  (send *ri* :set-master-volume 50)
  (send *ri* :stop-grasp)
  (case language
	(:ja
	 (speak_move "稲葉岡田研究室にようこそ\\pau=1000\\私はペッパー\\pau=500\\人と一緒に暮らすために作られたロボットです" *lift_arm_and_bow* (list 1000 100 100 100 100 100 1000))
	 (speak_move "家事が苦手で\\pau=500\\産業ロボットのような器用なアームはないですが\\pau=500\\言葉を通して人と交流することができますし\\pau=500\\人の声を認識したり\\pau=500\\センサーによって触覚を感じられたりします\\pau=1000\\そして\\pau=500\\人の顔を見て覚えられますよ。" *pepper_can_cannot* (list 200 100 100 100 100 100 100 100 100  100 150 100 150 100 100 100 100 1000))
	 (speak_move "目で世界を見ることができます\\pau=1000\\私の二つの目はスリーディーカメラで\\pau=500\\実世界の三次元画像をとれて\\pau=500\\移動するとき距離を判断できます。" *pepper_run* (list 200 200 200 200 200 100 100 100 100 100 100 100 100))
	 (speak_move "二次元画像を取得するために\\pau=500\\あと二つのカメラを持っています\\pau=1000\\ひたい\\pau=100\\と\\pau=100\\くちの中にそれぞれあって\\pau=500\\このように設計されているのはもっと広い範囲を見るためです。" (list *reset_pose* *reset_pose* *reset_pose* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_in_mouth* *show_the_camera_in_mouth*  *open_two_hands_widely*) (list 100 100 100 1000 100 100 100 100))
	 (speak_move "私の耳は実はスピーカーで\\pau=500\\ここから声を出します\\pau=1000\\私の後頭部\\pau=500\\アームと足にはタッチセンサーがあります\\pau=1000\\人が触るのを感知したり\\pau=500\\障害物を避けたりするためです。" *speaker_touch_sensor* (list 100 100 100 100 100 100 100 100 100 100))
	 (speak_move "そして最後に\\pau=500\\私の一番重要な部分\\pau=500\\つまり私のシーピーユーは\\pau=500\\頭の中にあります\\pau=1000\\全てのプログラムはその中に保存されていて\\pau=500\\それらによって思考します。" (list  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *touch_CPU* *touch_CPU* *touch_CPU* *reset_pose*) (list 100 100 100 100 100 100 100 100 100))
	 (speak_move "今は人の表情と声の調子の変化を識別することによって\\pau=500\\人の感情を読み取る勉強をしています\\pau=1000\\そうすることで\\pau=500\\私はみなさん\\pau=50\\ともっとうまく交流できます\\pau=1000\\私は\\pau=500\\できるだけ早く人の生活に入って\\pau=500\\みなさん\\pau=50\\と一緒に暮らしたいです!\\emph=2\\"  (list *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *bow_slightly_to_left* *bow_slightly_to_left* *bow_slightly_to_left* *reset_pose* *reset_pose* *reset_pose* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *put_two_hands_in_front_of_the_body* *bow_slightly_to_left* *bow_slightly_to_left* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *look_right* *look_right* *look_left* *look_left* *reset_pose* *reset_pose*) (list 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100))
	 (speak_move "今日はここで\\pau=100\\みなさんとお会いし\\pau=500\\自分のことを知ってもらえて\\pau=500\\とても嬉しいです\\pau=1000\\本当にありがとうございます。" (list *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *left_right_arm* *left_right_arm* *left_right_arm* *bow* *bow* *bow* *reset_pose*) (list 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100))
	 )
	(:chi
	 (playAudioFile_move "/audio_file/M1.mp3" *lift_arm_and_bow* (list 1000 100 100 100 100 100 1000))
	 (playAudioFile_move "/audio_file/M2.mp3" *pepper_can_cannot* (list 100 100 100 100 100 100 100 100 100  100 150 100 150 100 100 100 100 1000))
	 (playAudioFile_move "/audio_file/M3.mp3" *pepper_run* (list 200 200 200  200 200 200 200 100 100 100 100 100 100))
	 (playAudioFile_move "/audio_file/M4.mp3" *show_the_camera* (list 100 100 100 100 100 100 100 100 ))
	 (playAudioFile_move "/audio_file/M5.mp3" *speaker_touch_sensor* (list 100 100 100 100 100 100 100 100 100 100))
	 (playAudioFile_move "/audio_file/M6.mp3" (list  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *touch_CPU* *touch_CPU* *touch_CPU* *reset_pose*) (list 100 100 100 100 100 100 100 100))
	 (playAudioFile_move "/audio_file/M7.mp3" *look_around_hope* (list 100 100 100 100 100 100  100 100 100 100 100 100))
	 (playAudioFile_move "/audio_file/M8.mp3" *lift_arm_and_bow* (list 1000 100 100 100 100 100 1000))
	 )
	))
(ros::ros-warn "usage")
(ros::ros-warn "if Japanese,")
(ros::ros-warn "demo")
(ros::ros-warn "if Chinese,")
(ros::ros-warn "demo :chi")


;;(demo)

