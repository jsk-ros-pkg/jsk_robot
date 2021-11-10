(load "package://naoeus/naoplus.l") 
(setq *naop* (NaoH25V50plus))
(objects (list *naop*)) 
(send *naop* :reset-pose)

;; fullbody-ik
;; (setq rleg-coords (send *naop* :rleg :end-coords :copy-worldcoords))
(setq rleg-coords (make-coords :pos (float-vector 120 -150 -250)))
(setq lleg-coords (send *naop* :lleg :end-coords :copy-worldcoords))
(setq dest-full (make-coords :pos (float-vector 150 -40 50)))
(setq dest-full-l (make-coords :pos (float-vector 150 40 100)))
(defun fullik ()
  (send *naop* :fullbody-inverse-kinematics
	(list rleg-coords
	    lleg-coords)
	:move-target
	(list (send *naop* :rleg :end-coords)
	      (send *naop* :lleg :end-coords))
	:link-list
	(list (send *naop* :link-list (send *naop* :rleg :end-coords :parent))
	      (send *naop* :link-list (send *naop* :lleg :end-coords :parent)))
	:translation-axis (list t t)
	:rotation-axis (list nil t)
	#|
	:target-centroid-pos (midpoint 0.5
				       (send *naop* :rleg :end-coords :worldpos)
				       (send *naop* :lleg :end-coords :worldpos))
	|#
	:target-centroid-pos (send *naop* :lleg :end-coords :worldpos)
	:cog-translation-axis :z
	:thre (list 30 30)
	:centroid-thre 5
	)
  )

(defun fullik-l ()
  (send *naop* :fullbody-inverse-kinematics
	(list rleg-coords
	    lleg-coords
	    dest-full-l)
	:move-target
	(list (send *naop* :rleg :end-coords)
	      (send *naop* :lleg :end-coords)
	      (send *naop* :larm :end-coords))
	:link-list
	(list (send *naop* :link-list (send *naop* :rleg :end-coords :parent))
	      (send *naop* :link-list (send *naop* :lleg :end-coords :parent))
	      (send *naop* :link-list (send *naop* :larm :end-coords :parent)))
	:translation-axis (list t t t)
	:rotation-axis (list t t nil)
	:target-centroid-pos (midpoint 0.5
				       (send *naop* :rleg :end-coords :worldpos)
				       (send *naop* :lleg :end-coords :worldpos))
	:cog-translation-axis :z
	:thre (list 30 30 30)
	:centroid-thre 5
	)
  )

(objects (list *naop* dest-full dest-full-l rleg-coords))
#|
    (send *ri* :state)
    (setq pv (send *ri* :potentio-vector)) ;; 実機の角度取得

 成功（右腕を前に, sim）
#f(110.0 10.0 -90.0 -10.0 0.0 65.5962 6.62933 92.5897 62.4828 0.0 0.0 0.0 -30.0 60.0 -30.0 0.0 0.0 0.0 -30.0 60.0 -30.0 0.0 0.0 0.0) ;; sim
#f(108.808 11.3356 -89.0367 -10.193 0.612832 67.3275 4.74374 92.0202 60.9114 1.40386 0.002404 0.090295 -29.9687 60.0277 -29.9735 0.002404 0.002404 0.002404 -29.9735 60.0324 -29.9687 0.002404 0.788627 -1.145)

失敗
#f(110.0 10.0 -90.0 -40.0 0.0 16.2513 10.2294 53.8906 42.4242 -0.056222 9.94046 7.97013 -33.811 27.1275 -5.62529 -3.4198 -12.8398 6.378 -9.13465 19.1195 -6.10682 -2.95268 0.0 0.0) 
#f(109.247 13.1813 -90.3551 -39.1973 0.700723 19.8659 10.7204 54.4905 41.1357 1.93121 9.93417 7.91266 -33.8359 27.0682 -5.53958 -3.51326 9.93417 6.41851 -9.23104 18.987 -6.15001 -2.89802 0.788627 -0.969218)
|#
