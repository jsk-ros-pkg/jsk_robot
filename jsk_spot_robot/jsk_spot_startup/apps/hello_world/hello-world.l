#!/usr/bin/env roseus

(setq *continue* t)
(load "package://spoteus/spot-interface.l")
(spot-init)
(if (eq (send *ri* :state :power-state-shore-power-state) 'on-shore-power)
    (send *ri* :undock))

(defun init-pose ()
  (send *spot* :reset-pose)
  (send *spot* :arm :move-end-pos #f(0 0 250) :world)
  )

(defun start-func (args)
  (init-pose)
  (send *ri* :angle-vector (send *spot* :angle-vector) 500 :default-controller)
  (send *ri* :wait-interpolation)
  :started)

(defun end-func (args)
  (send *ri* :stow-arm)
  :finished)

(defun hello-func (args)
  (send *ri* :speak "Hello" :volume 1.0)
  (init-pose)
  (send *spot* :arm :move-end-rot -20 :z :parent)
  (send *spot* :arm :move-end-rot -10 :x :parent) 
  (send *ri* :angle-vector (send *spot* :angle-vector) 150 :default-controller)
  (send *ri* :wait-interpolation)
  ;;
  (send *ri* :speak "World" :volume 1.0)
  (init-pose)
  (send *spot* :arm :move-end-rot  20 :z :parent)
  (send *spot* :arm :move-end-rot -10 :x :parent) 
  (send *ri* :angle-vector (send *spot* :angle-vector) 150 :default-controller)
  (send *ri* :wait-interpolation)
  :finished)
;;
(load "package://roseus_smach/src/state-machine-ros.l")
(defun hello-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:start :started :hello)    ;; transitions (node transition node)
             (:hello :finished :end)
             (:end :finished :goal)
             )
           '((:start    'start-func)       ;; node-to-function maps
             (:end      'end-func)
             (:hello     'hello-func)
             )
           '(:start)                    ;; initial node
           '(:goal)                     ;; goal node
           ))
    (send sm :arg-keys 'description)
    sm))

;; create robot interface
(unless (boundp '*ri*) (spot-init))
(objects (list *spot*))

;;

;; this does not work...
;; (unix:signal unix::sigint '(lambda-closure nil 0 0 (sig code) (setq *continue* nil)))

;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/717
;; did not work, when we subscribe image data ???

(defun ros::roseus-sigint-handler (sig code)
  (ros::ros-warn (format nil "ros::roseus-sigint-handler ~A" sig))
  (setq *continue* nil))
(unix:signal unix::sigint 'ros::roseus-sigint-handler)

;; state machine
(exec-state-machine (hello-sm) '((description . "Hello World")(image . "")))
(exit)
