;; joint-state-compressor-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::roseus "joint_state_compressor_test")

(ros::roseus-add-msgs "sensor_msgs")

(setq *joint-states* nil)

(setq *diff-threshold-per-joint* (ros::get-param "~diff_threshold" 0.1))

(ros::subscribe "joint_states1" sensor_msgs::JointState
                #'(lambda (msg)
                    (push msg *joint-states*)
                    (ros::unsubscribe "joint_states1")))
(ros::subscribe "joint_states2" sensor_msgs::JointState
                #'(lambda (msg)
                    (push msg *joint-states*)
                    (ros::unsubscribe "joint_states2")))

(deftest test-joint-states-error ()
  (while (not (eq (length *joint-states*) 2))
    (ros::spin-once)
    (ros::sleep))
  (let ((msg1 (car *joint-states*))
        (msg2 (cadr *joint-states*))
        diff debug-diff-joint-name)
    (dolist (jt-name (flatten (send msg1 :name)
                              (send msg2 :name)))
      (let ((idx1 (position jt-name (send msg1 :name) :test #'string=))
            (idx2 (position jt-name (send msg2 :name) :test #'string=)))
        (unless (and idx1 idx2)
          (return))
        (let ((pos1 (elt (send msg1 :position) idx1))
              (pos2 (elt (send msg2 :position) idx2)))
          (push jt-name debug-diff-joint-name)
          (push (- pos1 pos2) diff))))
    (setq diff (apply #'float-vector (reverse diff)))
    (warning-message 2 "diff:~%")
    (dotimes (i (length diff))
      (warning-message 2 "~A: ~A~%" (elt debug-diff-joint-name i) (elt diff i)))
    (warning-message 2 "(norm diff): ~A~%" (norm diff))
    (warning-message 2 "threshold: ~A~%" (* (length diff) *diff-threshold-per-joint*))
    (assert (< (norm diff) (* (length diff) *diff-threshold-per-joint*)))
    ))

(ros::rate 1)
(run-all-tests)
(exit)
