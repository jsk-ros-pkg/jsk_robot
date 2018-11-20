#!/usr/bin/env roseus

(ros::roseus "greeting")
(require :pepper-interface "package://peppereus/pepper-interface.l")
(load "package://jsk_201504_miraikan/euslisp/pepper-motions.l")

;; If autonomous life is on, kill it and servo on again
(defun check-autonomous-life-is-on ()
  (unless (boundp '*ri*)
    (pepper-init))
  (if (not (string= (send *ri* :get-life) "disabled"))
      (progn
	(send *ri* :disable-life)
	(send *ri* :servo-on)
	))
  (send *ri* :angle-vector (send *pepper* :reset-pose)))

;; change speaing language if needed
(defun change-speaking-language (language)
  (if (not (string= (send *ri* :get-language) language))
      (send *ri* :set-language language)))

;; play audio file and make a gesture
(defun play_audio_file_move (file angle-list time-list)
  (ros::ros-info "speak")
  (send *ri* :angle-vector-sequence angle-list time-list)
  (send *ri* :play-audio-file file))

;; speak and make a gesture
(defun speak_move (speech_msg angle-list time-list)
  (ros::ros-info "speak")
  (send *ri* :speak speech_msg)
  (send *ri* :angle-vector-sequence angle-list time-list)
  (send *ri* :wait-interpolation))

;; main program
(defun demo(&optional (language :ja))
  (check-autonomous-life-is-on)
  (send *ri* :stop-grasp)
  (case language
	(:ja
	 (change-speaking-language "Japanese")
	 (speak_move "\\vct=180\\稲葉岡田研究室にようこそ\\pau=1000\\私はペッパー\\pau=500\\人と一緒に暮らすために作られたロボットです\\vct=130\\" *lift_arm_and_bow* (list 1000 1000 1000 1000 1000 1000 1000))
	 (speak_move "\\vct=180\\家事が苦手で\\pau=500\\産業ロボットのような器用なアームはないですが\\pau=500\\言葉を通して人と交流することができますし\\pau=500\\人の声を認識したり\\pau=500\\センサーによって触覚を感じられたりします\\pau=1000\\そして\\pau=500\\人の顔を見て覚えられますよ。\\vct=130\\" *pepper_can_cannot* (list 2000 1000 1000 1000 1000 1000 1000 1000 1000  1000 1500 1000 1500 1000 1000 1000 1000 1000))
	 (speak_move "\\vct=180\\目で世界を見ることができます\\pau=1000\\私の二つの目はスリーディーカメラで\\pau=500\\実世界の三次元画像をとれて\\pau=500\\移動するとき距離を判断できます。\\vct=130\\" *pepper_run* (list 1000 1000 1000 1000 1000 1000 100 100 100 1000 1000 1000 1000))
	 (speak_move "\\vct=180\\二次元画像を取得するために\\pau=500\\あと二つのカメラを持っています\\pau=1000\\ひたい\\pau=100\\と\\pau=100\\くちの中にそれぞれあって\\pau=500\\このように設計されているのはもっと広い範囲を見るためです。\\vct=130\\" (list *reset_pose* *reset_pose* *reset_pose* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_in_mouth* *show_the_camera_in_mouth*  *open_two_hands_widely*) (list 1000 1000 1000 1000 1000 1000 1000 1000))
	 (unix:sleep 2)
	 (speak_move "\\vct=180\\私の耳は実はスピーカーで\\pau=500\\ここから声を出します\\pau=1000\\私の後頭部\\pau=500\\アームと足にはタッチセンサーがあります\\pau=1000\\人が触るのを感知したり\\pau=500\\障害物を避けたりするためです。\\vct=130\\" *speaker_touch_sensor* (list 2000 1000 1000 2000 2000 1000 1000 1000 1000 1000))
	 (unix:sleep 2)
	 (speak_move "\\vct=180\\そして最後に\\pau=500\\私の一番重要な部分\\pau=500\\つまり私のシーピーユーは\\pau=500\\頭の中にあります\\pau=1000\\全てのプログラムはその中に保存されていて\\pau=500\\それらによって思考します。\\vct=130\\" (list  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *touch_CPU* *touch_CPU* *touch_CPU* *reset_pose*) (list 3000 2000 2000 1000 1000 1000 2000 1000 1000))
	 (speak_move "\\vct=180\\今は人の表情と声の調子の変化を識別することによって\\pau=500\\人の感情を読み取る勉強をしています\\pau=1000\\そうすることで\\pau=500\\私はみなさん\\pau=50\\ともっとうまく交流できます\\pau=1000\\私は\\pau=500\\できるだけ早く人の生活に入って\\pau=500\\みなさん\\pau=50\\と一緒に暮らしたいです!\\emph=1\\\\vct=130\\"  (list *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *bow_slightly_to_left* *bow_slightly_to_left* *bow_slightly_to_left* *reset_pose* *reset_pose*  *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *put_two_hands_in_front_of_the_body* *bow_slightly_to_left* *bow_slightly_to_left* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *look_right* *look_left* *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000))
	 (unix:sleep 2)
	 (speak_move "\\vct=180\\今日はここで\\pau=100\\みなさんとお会いし\\pau=500\\自分のことを知ってもらえて\\pau=500\\とても嬉しいです\\pau=1000\\本当にありがとうございます。\\vct=130\\" (list *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *left_right_arm* *left_right_arm* *left_right_arm* *bow* *bow* *bow* *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000))
	 )
	(:chi
	 (play_audio_file_move "/audio_file/M1.mp3" *lift_arm_and_bow* (list 1000 1000 1000 1000 1000 1000 1000))
	 (play_audio_file_move "/audio_file/M2.mp3" *pepper_can_cannot* (list 2000 1000 1000 1000 1000 1000 1000 1000 1000  1000 1500 1000 1500 1000 1000 1000 1000 1000))
	 (play_audio_file_move "/audio_file/M3.mp3" *pepper_run* (list 1000 1000 1000 1000 1000 1000 100 100 100 1000 1000 1000 1000))
	 (play_audio_file_move "/audio_file/M4.mp3" (list *reset_pose* *reset_pose* *reset_pose* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_in_mouth* *show_the_camera_in_mouth*  *open_two_hands_widely*) (list 1000 1000 1000 1000 1000 1000 1000 1000))
	 (play_audio_file_move "/audio_file/M5.mp3" *speaker_touch_sensor* (list 2000 1000 1000 2000 2000 1000 1000 1000 1000 1000))
	 (play_audio_file_move "/audio_file/M6.mp3" (list  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *touch_CPU* *touch_CPU* *touch_CPU* *reset_pose*) (list 3000 2000 2000 1000 1000 1000 2000 1000 1000))
	 (play_audio_file_move "/audio_file/M7.mp3" (list *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *bow_slightly_to_left* *bow_slightly_to_left* *bow_slightly_to_left* *reset_pose* *reset_pose*  *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *put_two_hands_in_front_of_the_body* *bow_slightly_to_left* *bow_slightly_to_left* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *look_right* *look_left* *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000))
	 (play_audio_file_move "/audio_file/M8.mp3" (list *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *left_right_arm* *left_right_arm* *left_right_arm* *bow* *bow* *bow* *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000))
	 )
	(:en
	 (change-speaking-language "English")
	 (speak_move "Welcome to JSK laboratory. I am pepper, \\pau=500\\a robot which is designed to live with human beings." *lift_arm_and_bow* (list 1000 1000 1000 1000 1000 1000 1000))
	 (speak_move "Although I'm not good at doing housework,\\pau500\\ nor do I have dexterous arms like industrial robots do, \\pau=500\\I'm able to communicate with humans through language, \\pau=500\\recognize human' voices,\\pau=500\\ feel touch and remember the faces of humans whom I have seen." *pepper_can_cannot* (list 2000 1000 1000 1000 1000 1000 1000 1000 1000  1000 1500 1000 1500 1000 1000 1000 1000 1000))
	 (speak_move "I can see the world with my eyes as well. My two eyes are 3D cameras, \\pau=500\\with which I get the point cloud data of the real world, \\pau=100\\so I can judge distance when I'm moving." *pepper_run*  (list 1000 1000 1000 1000 1000 1000 100 100 100 1000 1000 1000 1000))
	 (speak_move "I also have another two cameras for obtaining 2D pictures; \\pau=500\\they are located on my forehead and in my mouth. It was designed this way to enable me to see a broader range." (list *reset_pose* *reset_pose* *reset_pose* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_in_mouth* *show_the_camera_in_mouth*  *open_two_hands_widely*) (list 1000 1000 1000 1000 1000 1000 1000 1000))
	 (speak_move "My ears are actually speakers, \\pau=500\\and I speak through them. On my head, \\pau=500\\arms and feet there are touch sensors, \\pau=500\\using which I can feel humans' touch or avoid obstacle." *speaker_touch_sensor* (list 2000 1000 1000 2000 2000 1000 1000 1000 1000 1000))
	(speak_move "Finally, my most important part, my CPU, is in my head. All of my program is saved there, and I use it to think." (list  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *touch_CPU* *touch_CPU* *touch_CPU* *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000))
	(speak_move "Now, I'm learning how to understand human' emotion by recognizing changes of expression and voice \\pau=500\\ so that I can communicate with humans better. \\pau=1000\\I hope I can enter humans' lives as soon as possible and live together with everyone!" (list *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *lift_two_hands_beside_face_and_bow_slightly_to_right* *bow_slightly_to_left* *bow_slightly_to_left* *bow_slightly_to_left* *reset_pose* *reset_pose*  *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky* *put_two_hands_in_front_of_the_body*  *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 ))
	(speak_move "It was an honor to meet you here today and introduce myself to you. Thank you very much again for coming!" (list *reset_pose*  *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *left_right_arm* *left_right_arm* *left_right_arm* *bow* *bow* *bow* *reset_pose*) (list 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000))
	 )
	))
(ros::ros-warn "usage")
(ros::ros-warn "if Japanese, type (demo)")
(ros::ros-warn "if Chinese, type (demo :chi)")
(ros::ros-warn "if English, type (demo :en)")
