(load "package://naoeus/nao.l") 
(setq *nao* (NaoH25V50))
(objects (list *nao*)) 
(send *nao* :reset-pose)

;; fullbody-ikで姿勢を決める
(setq rleg-coords (send *nao* :rleg :end-coords :copy-worldcoords))
(setq lleg-coords (send *nao* :lleg :end-coords :copy-worldcoords))
(setq dest-full (make-coords :pos (float-vector 150 -40 100)))
(send *nao* :fullbody-inverse-kinematics
      (list rleg-coords
	    lleg-coords
	    dest-full)
      :move-target
      (list (send *nao* :rleg :end-coords)
	    (send *nao* :lleg :end-coords)
	    (send *nao* :rarm :end-coords))
      :link-list
      (list (send *nao* :link-list (send *nao* :rleg :end-coords :parent))
	    (send *nao* :link-list (send *nao* :lleg :end-coords :parent))
	    (send *nao* :link-list (send *nao* :rarm :end-coords :parent)))
      :translation-axis (list t t t)
      :rotation-axis (list t t nil)
      :target-centroid-pos (midpoint 0.5
				     (send *nao* :rleg :end-coords :worldpos)
				     (send *nao* :lleg :end-coords :worldpos))
      :cog-translation-axis :z
      :thre (list 30 30 30)
      :centroid-thre 5
      )


;; 実機に姿勢を送る
(send *ri* :servo-on)
(send *ri* :angle-vector (send *nao* :angle-vector))
(send *ri* :wait-interpolation)

;; 実機の角度を取得、表示する
(send *ri* :state)
(send *nao* :angle-vector (send *ri* :potentio-vector))


#|

#f(110.0 10.0 -90.0 -40.0 0.0 16.2513 10.2294 53.8906 42.4242 -0.056222 9.94046 7.97013 -33.811 27.1275 -5.62529 -3.4198 -12.8398 6.378 -9.13465 19.1195 -6.10682 -2.95268 0.0 0.0)

#f(108.104 13.1813 -90.2672 -39.0215 0.524941 20.1296 10.4567 53.9631 40.9599 1.0523 9.75839 8.08845 -33.748 27.244 -5.45169 -3.24959 9.75839 6.15482 -8.79157 19.1628 -6.32579 -2.81013 0.261268 -0.793436)

|#
