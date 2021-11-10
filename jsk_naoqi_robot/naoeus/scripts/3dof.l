(load "rotational-dependent-joint.l")

(defclass 3dof-robot
  :super cascaded-link
  :slots (end-coords l1 l2 l3 l4 j1 j2 j3))
(defmethod 3dof-robot
  (:init
   ()
   (let (b)
     (send-super :init)
     ;;
     (setq b (make-cube 10 10 20))
     (send b :locate #f(0 0 10))
     (send b :set-color :red)
     (setq l4 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l4))
     (setq end-coords (make-cascoords :pos #f(0 0 20)))
     (send l4 :assoc end-coords)
     (send l4 :locate #f(0 0 100))
     ;;
     (setq b (make-cube 10 10 100))
     (send b :locate #f(0 0 50))
     (send b :set-color :green)
     (setq l3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l3))
     (send l3 :assoc l4)
     (send l3 :locate #f(0 0 100))
     ;;
     (setq b (make-cube 10 10 100))
     (send b :locate #f(0 0 50))
     (send b :set-color :blue)
     (setq l2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l2))
     (send l2 :assoc l3)
     (send l2 :locate #f(0 0 20))
     ;;
     (setq b (body+ (make-cube 10 10 20 :pos #f(0 0 10)) (make-cube 300 300 2)))
     (send b :set-color :white)
     (setq l1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l1))
     (send l1 :assoc l2)
     ;;
     (setq j1 (instance rotational-dependent-joint :init :name 'j1
			:parent-link l1 :child-link l2 :axis :y :min -100 :max 100)
	   j2 (instance rotational-dependent-joint :init :name 'j2
			:parent-link l2 :child-link l3 :axis :y :min -100 :max 100)
	   j3 (instance rotational-joint :init :name 'J3
			:parent-link l3 :child-link l4 :axis :y :min -100 :max 100))
     (send j1 :set-dependent-joint j2)
     (send j2 :set-dependent-joint j1)
     (setq links (list l1 l2 l3 l4))
     (setq joint-list (list j1 j2 j3))
     ;;
     (send self :init-ending)
     self))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  )

(setq r (instance 3dof-robot :init))
(objects (list r (send r :end-coords)))
;;(send r :angle-vector #f(30 30 30))

#|
(dotimes (i 100)
  (setq j (send r :calc-jacobian-from-link-list
		(send r :link-list (send r :end-coords :parent))
		:move-target (send r :end-coords) :rotation-axis t :translation-axis t))
  (setq j# (sr-inverse j))
  (setq da (transform j# #f(1 0 0 0 0 0)))
  (send r :angle-vector (v+ (send r :angle-vector) da))
  (send *irtviewer* :draw-objects))
|#

(defun ik-depend ()
  (setq c (make-cascoords :pos #f(153 0 166)))
  (send r :inverse-kinematics c
	:link-list (send r :link-list (send r :end-coords :parent))
	:move-target (send r :end-coords)
	:translation-axis t
	:rotation-axis nil
	:thre 10
	:debug-view t)
  (objects (list r c (send r :end-coords))))

#|
(setq translation-axis t rotation-axis nil)
(dotimes (i 2000)
  (setq j
	(send r :calc-jacobian-from-link-list (send r :link-list (send r :end-coords :parent))
	      :move-target (send r :end-coords)
	      :rotation-axis rotation-axis :translation-axis translation-axis))
  (setq j# (sr-inverse j))
  (setq c (make-cascoords :pos (float-vector (* 100 (sin (/ i 100.0))) 0 200)))
  (setq dif-pos (send (send r :end-coords) :difference-position c))
  (setq da (transform j# dif-pos))
  (send r :angle-vector (v+ (send r :angle-vector) da))
  (send *irtviewer* :draw-objects :flush nil)
  (send c :draw-on :flush t)) 
|#

#|
(send r :angle-vector #f(0 0 0))
(setq c (make-cascoords :pos #f(150 0 0) :rpy (float-vector 0 pi 0)))
(send r :inverse-kinematics c
      :link-list (send r :link-list (send r :end-coords :parent))
      :move-target (send r :end-coords)
      :translation-axis t
      :rotation-axis t
      :debug-view t)
|#

#|
(send r :angle-vector #f(0 0 0))
(dotimes (i 100)
  (setq c (make-cascoords :pos (float-vector (+ 150 i) 0 0)
			  :rpy (float-vector 0 pi 0)))
  (send r :inverse-kinematics c
	:link-list (send r :link-list (send r :end-coords :parent))
	:move-target (send r :end-coords)
	:translation-axis t
	:rotation-axis nil
	:revert-if-fail nil)
  (x::window-main-one)
  (send *irtviewer* :draw-objects :flush nil)
  (send c :draw-on :flush t)
  ) 
|#
