(load "package://baxtereus/baxter-interface.l")
(ros::roseus "sample_real_baxter")
(baxter-init)

(defun main()
  ;;reset baxter
  (send *baxter* :reset-pose)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;;reset manip baxter
  (send *baxter* :reset-manip-pose)
  (send *ri* :angle-vector (send *baxter* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;;Camera Control
  (ros::ros-info "camera-controls")
  (send *ri* :close-left-hand-camera)
  (send *ri* :close-right-hand-camera)
  (send *ri* :close-head-camera)
  (send *ri* :open-left-hand-camera)
  (send *ri* :open-right-hand-camera)

  ;;Gripper Control
  (ros::ros-info "start-grasp")
  (send *ri* :start-grasp :rarm)
  (send *ri* :start-grasp :arms :effort 40)

  (ros::ros-info "stop-grasp")
  (send *ri* :stop-grasp)

  (ros::ros-info "go-grasp")
  (send *ri* :go-grasp :rarm :pos 50 :effort 30)


  (send *ri* :nod)
  ;;Set jpg file for baxter face
  ;;replace FILE_PATH with real jpeg file path
  ;;(send *ri* :set-baxter-face FILE_PATH)
  )

(ros::ros-info "(main) : start demo")
