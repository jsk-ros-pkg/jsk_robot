#!/usr/bin/env roseus
(defun main ()
  (require :dual_panda-interface "package://panda_eus/euslisp/dual_panda-interface.l")

  (ros::roseus "move_to_initial_pose")

  (dual_panda-init)

  (setq *robot* *dual_panda*)

  (send *robot* :angle-vector reset-pose)
  (when (send *ri* :check-error)
    (send *ri* :recover-error))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  (send *ri* :stop-grasp :rarm))

(main)
(exit)
