;; angle-vector-compress-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require :pr2 "package://pr2eus/pr2.l")
(pr2)

(require :angle-vector-compress "package://jsk_robot_utils/euslisp/angle-vector-compress.l")

(ros::roseus "angle_vector_compress_test")

(setq *error-threshold-per-joint* (ros::get-param "~error_threshold" 0.1))

(deftest just-compress-and-decompress ()
  (send *pr2* :reset-pose)
  (setq av1 (send *pr2* :angle-vector))
  (setq av1-c (compress-angle-vector *pr2* av1))
  (setq av1-dec (decompress-angle-vector *pr2* av1-c))
  (setq diff (apply #'float-vector
                    (mapcar #'(lambda (e)
                                (mod e 360)) (coerce (v- av1 av1-dec) cons))))
  (setq error-threshold (* (length av1) *error-threshold-per-joint*))
  (warning-message 3 "angle-vector: ~A~%" av1)
  (warning-message 3 "decompressed angle-vector: ~A~%" av1-dec)
  (warning-message 3 "diff: ~A~%"  diff)
  (warning-message 2 "error: ~A~%" (norm diff))
  (warning-message 2 "error-threshold: ~A~%" error-threshold)
  (assert (< (norm diff) error-threshold)))

(run-all-tests)
(exit)
