;; test_pr2_base_trajectory_action.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(init-unit-test)

;; pr2 on gazebo does not keep velocity commanded from controller.
;; so we instead had to write test case with wide threshold
#|
(deftest test-go-pos-unsafe ()
  (pr2-init)
  (setq *start-cds* (send *ri* :state :worldcoords))

  (send *ri* :go-pos-unsafe 1.0 0)
  (setq *go-cds* (send *ri* :state :worldcoords))
  (warn "cds: ~A~%" *go-cds*)
  (assert (eps-v= (send *go-cds* :worldpos) (float-vector 1000 0 0) 50)
          "go-pos 1 0 0")
  (setq *prev-pos* (send *start-cds* :copy-worldcoords))
  (send *ri* :go-pos-unsafe 0 1.0)
  (setq *go-cds* (send *ri* :state :worldcoords))
  (warn "cds: ~A~%" *go-cds*)
  (assert (eps-v= (send *go-cds* :worldpos) (float-vector 1000 1000 0) 50)
          "go-pos 0 1 0")

  (send *ri* :go-pos-unsafe 0 0 90)
  (setq *go-cds* (send *ri* :state :worldcoords))
  (warn "cds: ~A~%" *go-cds*)
  (assert (< (abs (- (caar (send *go-cds* :rpy-angle)) pi/2)) (deg2rad 10))
          "go-pos 0 0 pi/2"))
|#

(deftest test-go-pos-unsafe ()
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  ;; wait for action controller
  (setq *base-traj-action* (pr2-interface-move-base-trajectory-action *ri*))
  (setq *cnt* 0)
  (while (not (send *base-traj-action* :wait-for-server 1))
    (ros::ros-warn "waiting for server ~A" (send *base-traj-action* :name))
    (unix:sleep 3)
    (when (> (inc *cnt*) 100)
      (assert nil "server not started")))

  (setq *start-cds* (send *ri* :state :worldcoords))

  (send *ri* :go-pos-unsafe 1.0 0)
  (setq *go-cds* (send *ri* :state :worldcoords))
  (warn "cds: ~A~%" *go-cds*)
  (assert (> (norm (send *start-cds* :difference-position *go-cds*)) 50)
          "go-pos 1 0 0")
  (setq *prev-cds* (send *start-cds* :copy-worldcoords))
  (send *ri* :go-pos-unsafe 0 1.0)
  (setq *go-cds* (send *ri* :state :worldcoords))
  (warn "cds: ~A~%" *go-cds*)
  (assert (> (norm (send *prev-cds* :difference-position *go-cds*)) 50)
          "go-pos 0 1 0")
  (setq *prev-cds* (send *go-cds* :copy-worldcoords))

  (send *ri* :go-pos-unsafe 0 0 90)
  (setq *go-cds* (send *ri* :state :worldcoords))
  (warn "cds: ~A~%" *go-cds*)
  (assert (> (norm (send *go-cds* :difference-rotation *prev-cds*)) (deg2rad 10))
          "go-pos 0 0 pi/2")
)

(run-all-tests)
(exit)
