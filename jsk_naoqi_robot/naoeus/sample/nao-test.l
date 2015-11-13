(ros::roseus "nao-test")
(load "package://naoeus/euslisp/nao-interface.l")

(setq *nao* (nao))
(objects (list *nao*))
(setq *ni* (instance nao-interface :init))

(unix:sleep 1)
(send *ni* :servo-on)
(dotimes (i 3)
  (send *nao* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
  (send *ni* :angle-vector (send *nao* :angle-vector) 2000)
  (send *ni* :wait-interpolation)
  )

;;(send *ni* :go-pos 1 0 0)
(send *nao* :fix-leg-to-coords (make-coords) '(:rleg :lleg))

(send *nao* :move-centroid-on-foot :both '(:rleg :lleg)
      :target-centroid-pos (send *nao* :lleg :end-coords :worldpos))
(send *ni* :angle-vector (send *nao* :angle-vector) 2000)
(send *ni* :wait-interpolation)

(send *nao* :rleg :move-end-pos #f(0 0 50))
(send *ni* :angle-vector (send *nao* :angle-vector) 2000)
(send *ni* :wait-interpolation)


(send *nao* :reset-pose)
(send *nao* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
(let* ((tc (list nil
		 (send (send *nao* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 25) :world)
		 nil
		 nil)))
  (with-move-target-link-list
   (mt ll *nao* '(:rleg :lleg :rarm :larm))
   (send *nao* :fullbody-inverse-kinematics
	 (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	 :link-list ll :move-target mt
	 :collision-avoidance-link-pair nil
	 :revert-if-fail nil    :stop 1000
	 :translation-axis '(t t nil nil)
	 :rotation-axis '(t :z t t)
	 :thre '(5 5 10 10)
	 :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
	 :target-centroid-pos (send (send *nao* :rleg :end-coords :copy-worldcoords) :worldpos)
	 :centroid-thre 20)))

(send *ni* :angle-vector (send *nao* :angle-vector) 2000)
(send *ni* :wait-interpolation)



(send *nao* :rarm :inverse-kinematics 
      (send  (send (send *nao* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 100 0 100) :world)
      :revert-if-fail nil
      :look-at-target t
      :rotation-axis nil
      :debug-view nil
      )
(send *ni* :angle-vector (send *nao* :angle-vector) 2000)
(send *ni* :wait-interpolation)
