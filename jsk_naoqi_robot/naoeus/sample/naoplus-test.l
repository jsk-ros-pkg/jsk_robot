(load "package://naoeus/naoplus.l") 
(setq *naop* (NaoH25V50plus))
(objects (list *naop*)) 
(send *naop* :reset-pose)

;; fullbody-ik
(setq rleg-coords (make-coords :pos (float-vector 120 -150 -250)))
(setq lleg-coords (send *naop* :lleg :end-coords :copy-worldcoords))
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
      :target-centroid-pos (send *naop* :lleg :end-coords :worldpos)
      :cog-translation-axis :z
      :thre (list 30 30)
      :centroid-thre 5
      )

(objects (list *naop* lleg-coords rleg-coords))
