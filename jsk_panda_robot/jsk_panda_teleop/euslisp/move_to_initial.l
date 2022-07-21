#!/usr/bin/env roseus
(load "package://panda_eus/euslisp/dual_panda-interface.l")

(defun main ()
  (ros::roseus "move_to_initial_pose")

  (dual_panda-init)

  (send *robot* :angle-vector (send *robot* :reset-manip-pose))
  (when (send *ri* :check-error)
    (send *ri* :recover-error))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  (send *ri* :stop-grasp :rarm))

(main)
(exit)
