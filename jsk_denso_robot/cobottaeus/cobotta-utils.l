(require :cobotta "package://cobottaeus/cobotta.l")

(defmethod cobotta-robot
  (:arm (&rest args) (send* self :rarm args))      ;; enable to call send *cobotta* :arm :angle-vector
  (:start-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0 args)
    t)
  (:stop-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.015 args))
  (:go-grasp
    (&key (pos 0))
    "Move gripper fingers to target `pos`.
Arguments:
- pos : target distance between each finger and the gripper center (0.0-0.015) [m]
Details: https://densorobot.github.io/docs/denso_cobotta_ros/command_reference/rostopic.html#parallel-gripper
"
    (send self :joint_gripper :joint-angle (* pos 1000)) ;; m -> mm
    (send self :joint_gripper_mimic :joint-angle (* pos -1000))
    t)
  )
