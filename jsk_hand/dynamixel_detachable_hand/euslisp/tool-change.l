#!/usr/bin/env roseus

(require "package://dynamixel_detachable_hand/euslisp/rhand-interface.l")
(require "package://dynamixel_detachable_hand/euslisp/lhand-interface.l")

(defvar *detachable-tool-robot* nil)
(defvar *detachable-tool-robot-kind* nil)
(defvar *detachable-tool-arm-map* nil)
(defvar *detachable-tool-state* nil)
(defvar *detachable-tool-launch-wait* 3.0)
(defvar *detachable-tool-update-irtviewer* t)
(defvar *detachable-tool-irtviewer-objects* nil)
(defvar *detachable-tool-draw-tip-visual* t)
(defvar *detachable-tool-viewer-visuals* nil)
(defparameter *detachable-tool-needle-holder-grasp-tip-ops*
  (list (list :rotate (deg2rad -90) :y)
        (list :rotate (deg2rad 90) :x :local)
        (list :translate #f(96 -12 -20))))

(defun detachable-tool-key-string (x)
  (string-upcase (string x)))

(defun detachable-tool-normalize-arm (arm)
  (let ((s (detachable-tool-key-string arm)))
    (cond
     ((string= s "RARM") :rarm)
     ((string= s "LARM") :larm)
     (t (error "unsupported arm ~A" arm)))))

(defun detachable-tool-normalize-side (side)
  (let ((s (detachable-tool-key-string side)))
    (cond
     ((string= s "RHAND") :rhand)
     ((string= s "LHAND") :lhand)
     (t (error "unsupported detachable hand side ~A" side)))))

(defun detachable-tool-normalize-tool (tool)
  (let ((s (detachable-tool-key-string tool)))
    (cond
     ((or (string= s "GENERIC") (string= s "DUMMY")) :generic)
     ((or (string= s "NEEDLE_HOLDER")
          (string= s "NEEDLE-HOLDER")
          (string= s "HOLDER")) :needle_holder)
     ((string= s "GRIPPER") :gripper)
     ((or (string= s "FORCEPS")
          (string= s "SCISSORS")
          (string= s "SCISSOR")) :forceps)
     (t (error "unsupported detachable tool ~A" tool)))))

(defun detachable-tool-normalize-robot-kind (kind)
  (let ((s (detachable-tool-key-string kind)))
    (cond
     ((or (string= s "MYCOBOT") (string= s "SINGLE-ARM")
          (string= s "SINGLE_ARM")) :mycobot)
     ((or (string= s "NEXTAGE") (string= s "DUAL-ARM")
          (string= s "DUAL_ARM")) :nextage)
     (t (error "unsupported robot kind ~A" kind)))))

(defun detachable-tool-tool-name (tool)
  (case (detachable-tool-normalize-tool tool)
    (:generic "generic")
    (:needle_holder "needle_holder")
    (:gripper "gripper")
    (:forceps "forceps")))

(defun detachable-tool-side-name (side)
  (case (detachable-tool-normalize-side side)
    (:rhand "rhand")
    (:lhand "lhand")))

(defun detachable-tool-default-arm-map (robot-kind)
  (case (detachable-tool-normalize-robot-kind robot-kind)
    (:mycobot '((:rarm . :lhand)))
    (:nextage '((:larm . :lhand) (:rarm . :rhand)))))

(defun detachable-tool-guess-robot-kind (robot)
  (cond
   ((and (boundp '*mycobot*) (eq robot *mycobot*)) :mycobot)
   ((and (boundp '*nextage*) (eq robot *nextage*)) :nextage)
   (t :mycobot)))

(defun tool-setup-robot (robot &key robot-kind arm-map)
  (setq *detachable-tool-robot* robot)
  (setq *detachable-tool-robot-kind*
        (detachable-tool-normalize-robot-kind
         (or robot-kind (detachable-tool-guess-robot-kind robot))))
  (setq *detachable-tool-arm-map*
        (or arm-map (detachable-tool-default-arm-map *detachable-tool-robot-kind*)))
  *detachable-tool-arm-map*)

(defun detachable-tool-side-for-arm (robot arm)
  (unless *detachable-tool-arm-map*
    (tool-setup-robot robot))
  (let* ((a (detachable-tool-normalize-arm arm))
         (pair (assoc a *detachable-tool-arm-map*)))
    (unless pair
      (error "arm ~A is not available for detachable tools; arm map is ~A"
             a *detachable-tool-arm-map*))
    (detachable-tool-normalize-side (cdr pair))))

(defun detachable-tool-arm-state (arm)
  (cdr (assoc (detachable-tool-normalize-arm arm) *detachable-tool-state*)))

(defun detachable-tool-set-arm-state (arm state)
  (let* ((a (detachable-tool-normalize-arm arm))
         (entry (assoc a *detachable-tool-state*)))
    (if entry
        (rplacd entry state)
      (push (cons a state) *detachable-tool-state*))
    state))

(defun detachable-tool-state-get (arm key)
  (cdr (assoc key (detachable-tool-arm-state arm))))

(defun detachable-tool-state-set (arm key value)
  (let* ((state (detachable-tool-arm-state arm))
         (entry (assoc key state)))
    (if entry
        (rplacd entry value)
      (setq state (acons key value state)))
    (detachable-tool-set-arm-state arm state)
    value))

(defun detachable-tool-clear-arm-state (arm)
  (let ((entry (assoc (detachable-tool-normalize-arm arm)
                      *detachable-tool-state*)))
    (when entry
      (setq *detachable-tool-state*
            (remove entry *detachable-tool-state* :test #'eq)))))

(defun detachable-tool-hand-interface (side)
  (case (detachable-tool-normalize-side side)
    (:rhand (rhand-init))
    (:lhand (lhand-init))))

(defun detachable-tool-visual-box (x y z pos color)
  (let ((body (make-cube x y z :pos pos)))
    (send body :set-color color)
    body))

(defun detachable-tool-tip-visual-bodies (tool)
  (case (detachable-tool-normalize-tool tool)
    (:needle_holder
     (list
      (detachable-tool-visual-box 35 35 80 #f(0 0 40) #f(1 0 0))
      (detachable-tool-visual-box 18 18 150 #f(0 0 155) #f(1 0 0))))
    (:forceps
     (list
      (detachable-tool-visual-box 40 40 70 #f(0 0 35) #f(1 0.82 0))
      (detachable-tool-visual-box 18 18 190 #f(0 0 165) #f(1 0.82 0))))
    (:gripper
     (list
      (detachable-tool-visual-box 55 55 70 #f(0 0 35) #f(0 0.75 0.25))
      (detachable-tool-visual-box 60 20 160 #f(0 0 150) #f(0 0.75 0.25))))
    (t
     (list
      (detachable-tool-visual-box 45 45 70 #f(0 0 35) #f(0 0.8 1))
      (detachable-tool-visual-box 30 30 120 #f(0 0 130) #f(0 0.8 1))))))

(defun detachable-tool-make-tip-visual (arm tool)
  (when *detachable-tool-draw-tip-visual*
    (let ((visual
           (instance bodyset-link :init (make-cascoords)
                     :bodies (detachable-tool-tip-visual-bodies tool)
                     :name (format nil "~A-~A-visual"
                                   (string-downcase (string arm))
                                   (detachable-tool-tool-name tool)))))
      (pushnew visual *detachable-tool-viewer-visuals* :test #'eq)
      visual)))

(defun detachable-tool-active-viewer-visuals ()
  (let (visuals)
    (dolist (entry *detachable-tool-state* (nreverse visuals))
      (let ((visual (cdr (assoc :tip-visual (cdr entry)))))
        (when visual
          (push visual visuals))))))

(defun detachable-tool-remove-viewer-visuals (objects)
  (remove-if #'(lambda (obj)
                 (member obj *detachable-tool-viewer-visuals* :test #'eq))
             objects))

(defun detachable-tool-objects-with-visuals (robot objects)
  (let* ((base (detachable-tool-remove-viewer-visuals (or objects nil)))
         (with-robot (if (member robot base :test #'eq)
                         base
                       (cons robot base))))
    (append with-robot (detachable-tool-active-viewer-visuals))))

(defun detachable-tool-irtviewer-available-p ()
  (and (boundp '*irtviewer*) *irtviewer*))

(defun detachable-tool-irtviewer-object-list (robot)
  (detachable-tool-objects-with-visuals
   robot
   (or *detachable-tool-irtviewer-objects*
       (send *irtviewer* :objects))))

(defun detachable-tool-redraw-irtviewer (robot)
  (when (and *detachable-tool-update-irtviewer*
             (detachable-tool-irtviewer-available-p))
    (send *irtviewer* :objects (detachable-tool-irtviewer-object-list robot))
    (send *irtviewer* :draw-objects)
    t))

(defun detachable-tool-robot-interface-available-p (robot)
  (and (boundp '*ri*) *ri*
       (find-method *ri* :draw-objects)
       (or (not (find-method *ri* :robot))
           (eq (send *ri* :robot) robot))))

(defun detachable-tool-redraw-robot-interface (robot)
  (when (and *detachable-tool-update-irtviewer*
             (detachable-tool-robot-interface-available-p robot))
    (when (find-method *ri* :objects)
      (let ((objects (detachable-tool-remove-viewer-visuals
                      (send *ri* :objects))))
        (send *ri* :objects
              (append objects (detachable-tool-active-viewer-visuals)))))
    (send *ri* :draw-objects)
    t))

(defun detachable-tool-redraw-viewers (robot)
  (let ((irtviewer-updated (detachable-tool-redraw-irtviewer robot))
        (robot-interface-updated
         (detachable-tool-redraw-robot-interface robot)))
    (or irtviewer-updated robot-interface-updated)))

(defun detachable-tool-shell (cmd)
  (ros::ros-info "~A" cmd)
  (unix:system cmd))

(defun detachable-tool-bool-string (value)
  (if value "true" "false"))

(defun detachable-tool-launch-static-tf-args
    (publish-static-tf parent-frame root-frame static-xyz static-rpy)
  (if (or publish-static-tf parent-frame)
      (format nil " publish_static_tf:=~A parent_frame:=~A root_frame:=~A static_xyz:='~A' static_rpy:='~A'"
              (detachable-tool-bool-string (or publish-static-tf parent-frame))
              (or parent-frame "")
              (or root-frame "")
              (or static-xyz "0 0 0")
              (or static-rpy "0 0 0"))
    ""))

(defun detachable-tool-kill-model-nodes (namespace)
  (detachable-tool-shell
   (format nil "pkill -f \"ros2 launch dynamixel_detachable_hand .*namespace:=~A\" >/dev/null 2>&1 || true"
           namespace)))

(defun reload-tool-model (side tool &key namespace
                                    publish-static-tf parent-frame root-frame
                                    (static-xyz "0 0 0")
                                    (static-rpy "0 0 0")
                                    (wait *detachable-tool-launch-wait*))
  (let* ((side-name (detachable-tool-side-name side))
         (ns (or namespace side-name))
         (tool-name (detachable-tool-tool-name tool))
         (root (or root-frame (format nil "~A_base_link" side-name)))
         (static-tf-args
          (detachable-tool-launch-static-tf-args
           publish-static-tf parent-frame root static-xyz static-rpy)))
    (detachable-tool-kill-model-nodes ns)
    (detachable-tool-shell
     (format nil "ros2 launch dynamixel_detachable_hand hand_model.launch.xml side:=~A namespace:=~A tool:=~A attached:=true~A &"
             side-name ns tool-name static-tf-args))
    (when wait (ros::duration-sleep wait))
    tool-name))

(defun start-tool-control (side tool &key namespace port-name
                                      (baud-rate "57600")
                                      publish-static-tf parent-frame root-frame
                                      (static-xyz "0 0 0")
                                      (static-rpy "0 0 0")
                                      (wait *detachable-tool-launch-wait*))
  (let* ((side-name (detachable-tool-side-name side))
         (ns (or namespace side-name))
         (tool-name (detachable-tool-tool-name tool))
         (port (or port-name (format nil "/dev/~A" side-name)))
         (root (or root-frame (format nil "~A_base_link" side-name)))
         (static-tf-args
          (detachable-tool-launch-static-tf-args
           publish-static-tf parent-frame root static-xyz static-rpy)))
    (detachable-tool-kill-model-nodes ns)
    (detachable-tool-shell
     (format nil "ros2 launch dynamixel_detachable_hand hand_control.launch.xml side:=~A namespace:=~A port_name:=~A baud_rate:=~A tool:=~A attached:=true~A &"
             side-name ns port baud-rate tool-name static-tf-args))
    (when wait (ros::duration-sleep wait))
    tool-name))

(defun start-detacher-control (side &key namespace port-name
                                     (baud-rate "57600")
                                     (wait *detachable-tool-launch-wait*))
  (let* ((side-name (detachable-tool-side-name side))
         (ns (or namespace side-name))
         (port (or port-name (format nil "/dev/~A" side-name))))
    (detachable-tool-kill-model-nodes ns)
    (detachable-tool-shell
     (format nil "ros2 launch dynamixel_detachable_hand hand_detacher.launch.xml side:=~A namespace:=~A port_name:=~A baud_rate:=~A &"
             side-name ns port baud-rate))
    (when wait (ros::duration-sleep wait))
    ns))

(defun detachable-tool-default-tip-ops (robot-kind arm tool)
  (let ((kind (detachable-tool-normalize-robot-kind robot-kind))
        (a (detachable-tool-normalize-arm arm))
        (tname (detachable-tool-normalize-tool tool)))
    (cond
     ((and (eq kind :nextage) (eq a :rarm) (eq tname :needle_holder))
      (list (list :translate #f(0 0 190))
            (list :translate #f(0 0 -102))
            (list :rotate (deg2rad 33) :z)
            (list :rotate (deg2rad 88.7) :y)
            (list :translate #f(0 -23 90))))
     ((and (eq kind :nextage) (eq a :larm) (eq tname :forceps))
      (list (list :rotate (deg2rad -90) :y)
            (list :translate #f(190 0 0))
            (list :translate #f(-170 -34 -45))
            (list :rotate (deg2rad -45) :x)
            (list :rotate (deg2rad -20.534) :z :local)
            (list :rotate (deg2rad 24.683) :y :local)
            (list :rotate (deg2rad 138.110) :x :local)
            (list :rotate (deg2rad 180) :z :local)
            (list :translate #f(0 0 5))))
     ((eq tname :needle_holder)
      *detachable-tool-needle-holder-grasp-tip-ops*)
     ((eq tname :forceps)
      (list (list :translate #f(0 0 90))))
     ((eq tname :gripper)
      (list (list :translate #f(0 0 80))))
     (t
      (list (list :translate #f(0 0 60)))))))

(defun detachable-tool-tip-ops-from-pos-rpy (pos rpy)
  (append
   (if pos (list (list :translate pos)) nil)
   (if rpy
       (list (list :rotate (elt rpy 0) :x :local)
             (list :rotate (elt rpy 1) :y :local)
             (list :rotate (elt rpy 2) :z :local))
     nil)))

(defun detachable-tool-apply-tip-op (coords op)
  (case (car op)
    (:translate
     (if (cddr op)
         (send coords :translate (cadr op) (caddr op))
       (send coords :translate (cadr op))))
    (:rotate
     (if (cdddr op)
         (send coords :rotate (cadr op) (caddr op) (cadddr op))
       (send coords :rotate (cadr op) (caddr op))))
    (t (error "unsupported tool tip op ~A" op)))
  coords)

(defun detachable-tool-install-tip (robot arm tool &key tip-ops tip-pos tip-rpy)
  (let* ((a (detachable-tool-normalize-arm arm))
         (tname (detachable-tool-normalize-tool tool))
         (end-coords (send robot a :end-coords))
         (tip (make-cascoords
               :name (format nil "~A-~A-tip" (string-downcase (string a))
                             (detachable-tool-tool-name tname))))
         (visual (detachable-tool-make-tip-visual a tname))
         (ops (or tip-ops
                  (detachable-tool-tip-ops-from-pos-rpy tip-pos tip-rpy)
                  (detachable-tool-default-tip-ops *detachable-tool-robot-kind*
                                                   a tname))))
    (detach-tool-tip robot a)
    (send end-coords :assoc tip)
    (send tip :newcoords (make-coords))
    (when visual
      (send end-coords :assoc visual)
      (send visual :newcoords (make-coords)))
    (dolist (op ops) (detachable-tool-apply-tip-op tip op))
    (detachable-tool-state-set a :tool tname)
    (detachable-tool-state-set a :tip tip)
    (detachable-tool-state-set a :tip-visual visual)
    (detachable-tool-state-set a :tip-ops ops)
    tip))

(defun detach-tool-tip (robot arm)
  (let* ((a (detachable-tool-normalize-arm arm))
         (tip (detachable-tool-state-get a :tip))
         (visual (detachable-tool-state-get a :tip-visual)))
    (when (and tip (send tip :parent))
      (send (send tip :parent) :dissoc tip))
    (when (and visual (send visual :parent))
      (send (send visual :parent) :dissoc visual))
    (detachable-tool-state-set a :tip nil)
    (detachable-tool-state-set a :tip-visual nil)
    nil))

(defun attach-tool (robot arm tool &rest args
                          &key (physical nil)
                               (reload-model t)
                               (restart-control nil)
                               namespace port-name
                               (baud-rate "57600")
                               (tm 6.0)
                               (wait *detachable-tool-launch-wait*)
                               tip-ops tip-pos tip-rpy
                               publish-static-tf parent-frame root-frame
                               (static-xyz "0 0 0")
                               (static-rpy "0 0 0")
                          &allow-other-keys)
  (declare (ignore args))
  (unless *detachable-tool-arm-map*
    (tool-setup-robot robot))
  (let* ((a (detachable-tool-normalize-arm arm))
         (side (detachable-tool-side-for-arm robot a))
         (tname (detachable-tool-normalize-tool tool)))
    (when physical
      (send (detachable-tool-hand-interface side) :attach :tm tm))
    (cond
     (restart-control
      (start-tool-control side tname :namespace namespace :port-name port-name
                          :baud-rate baud-rate :wait wait
                          :publish-static-tf publish-static-tf
                          :parent-frame parent-frame
                          :root-frame root-frame
                          :static-xyz static-xyz
                          :static-rpy static-rpy))
     (reload-model
      (reload-tool-model side tname :namespace namespace :wait wait
                         :publish-static-tf publish-static-tf
                         :parent-frame parent-frame
                         :root-frame root-frame
                         :static-xyz static-xyz
                         :static-rpy static-rpy)))
    (detachable-tool-state-set a :side side)
    (detachable-tool-state-set a :publish-static-tf publish-static-tf)
    (detachable-tool-state-set a :parent-frame parent-frame)
    (detachable-tool-state-set a :root-frame root-frame)
    (detachable-tool-state-set a :static-xyz static-xyz)
    (detachable-tool-state-set a :static-rpy static-rpy)
    (let ((tip (detachable-tool-install-tip robot a tname
                                            :tip-ops tip-ops
                                            :tip-pos tip-pos
                                            :tip-rpy tip-rpy)))
      (detachable-tool-redraw-viewers robot)
      tip)))

(defun detach-tool (robot arm &rest args
                          &key (physical nil)
                               (reload-model t)
                               (restart-control nil)
                               namespace port-name
                               (baud-rate "57600")
                               (tm 6.0)
                               (wait *detachable-tool-launch-wait*)
                               publish-static-tf parent-frame root-frame
                               static-xyz static-rpy
                          &allow-other-keys)
  (declare (ignore args))
  (unless *detachable-tool-arm-map*
    (tool-setup-robot robot))
  (let* ((a (detachable-tool-normalize-arm arm))
         (side (detachable-tool-side-for-arm robot a))
         (static-p (or publish-static-tf
                       (detachable-tool-state-get a :publish-static-tf)
                       parent-frame
                       (detachable-tool-state-get a :parent-frame)))
         (parent (or parent-frame
                     (detachable-tool-state-get a :parent-frame)))
         (root (or root-frame
                   (detachable-tool-state-get a :root-frame)))
         (xyz (or static-xyz
                  (detachable-tool-state-get a :static-xyz)
                  "0 0 0"))
         (rpy (or static-rpy
                  (detachable-tool-state-get a :static-rpy)
                  "0 0 0")))
    (when physical
      (send (detachable-tool-hand-interface side) :detach :tm tm))
    (detach-tool-tip robot a)
    (detachable-tool-redraw-viewers robot)
    (detachable-tool-state-set a :tool :generic)
    (when restart-control
      (start-detacher-control side :namespace namespace :port-name port-name
                              :baud-rate baud-rate :wait wait))
    (when reload-model
      (reload-tool-model side :generic :namespace namespace :wait wait
                         :publish-static-tf static-p
                         :parent-frame parent
                         :root-frame root
                         :static-xyz xyz
                         :static-rpy rpy))
    (detachable-tool-clear-arm-state a)
    t))

(defun tool-end-coords (robot arm)
  (or (detachable-tool-state-get (detachable-tool-normalize-arm arm) :tip)
      (send robot (detachable-tool-normalize-arm arm) :end-coords)))

(defun tool-inverse-kinematics (robot arm target-coords &rest args
                                      &key &allow-other-keys)
  (send* robot (detachable-tool-normalize-arm arm) :inverse-kinematics
         target-coords
         :move-target (tool-end-coords robot arm)
         args))

(defun tool-robot-inverse-kinematics (robot arm target-coords &rest args
                                            &key &allow-other-keys)
  (let ((a (detachable-tool-normalize-arm arm)))
    (send* robot :inverse-kinematics
           target-coords
           :move-target (tool-end-coords robot a)
           :link-list (send robot :link-list
                            (send robot a :end-coords :parent)
                            (send robot a :root-link))
           args)))

(defun tool-ik (robot arm target-coords &rest args &key &allow-other-keys)
  (send* robot (detachable-tool-normalize-arm arm) :inverse-kinematics
         target-coords
         :move-target (tool-end-coords robot arm)
         args))

;; Keep the detachable-hand dependency out of mycoboteus itself while allowing
;; the robot model to provide the usual chained message API after mycoboteus is
;; loaded first:
;;   (send *mycobot* :rhand :open-holder :wait t)
(when (boundp 'mycobot-robot)
  (defmethod mycobot-robot
    (:rhand (&rest args)
      (if args
          (send* (rhand-init) args)
        (rhand-init)))))

(provide :dynamixel-detachable-hand-tool-change
         "package://dynamixel_detachable_hand/euslisp/tool-change.l")
