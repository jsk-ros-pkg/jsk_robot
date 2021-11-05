;; roslaunch jsk_nao_startup jsk_nao_startup.launch network_interface:=wlp5s0

(load "package://naoeus/nao.l") ;; simの場合
(setq *nao* (NaoH25V50))
(objects (list *nao*))
(send *nao* :reset-pose)

;; 腕の角度を変えてみる
(send *nao* :larm :elbow-p :joint-angle -10)
(send *nao* :rarm :elbow-y :joint-angle 110)

;; 関節のbodyset-linkを獲得 
(setq cc (send *nao* :rarm :elbow-y :child-link))

;; 関節のcoordinatesをワールド座標系として取得
(setq dd (send cc :worldcoords))

(objects (list *nao* dd)) ;; 関節の座標系も表示

;; ロボットの座標をもとに目標座標を作成
(setq dest (send *nao* :copy-worldcoords))

;; 目標座標を指定
(send dest :translate #f(100 -100 0))

;; 右手の手先の座標も表示する
(setq rend (send *nao* :rarm :end-coords))

(objects (list *nao* dest rend))

;; ikで関節角度計算、動かす
(send *nao* :rarm :inverse-kinematics
      dest
      :link-list (send *nao* :link-list (send *nao* :rarm :end-coords :parent))
      :move-target (send *nao* :rarm :end-coords)
      :translation-axis :xyz
      :rotation-axis nil
      :debug-view t
      )

#|
(send *ri* :state)
(send *ri* :potentio-vector) ;; ロボットの姿勢を入手
|#
