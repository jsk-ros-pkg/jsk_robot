;; カメラ
;; $ roslaunch opencv_apps camshift.launch image:=/nao_robot/naoqi_driver/camera/front/image_raw
(ros::roseus-add-msgs "opencv_apps")
(setq x 0) ;; 対象物位置

;; camshiftのコールバック関数
(defun cams-cb (rect)
  (setq x (send rect :rect :center :x)))
(ros::subscribe "camshift/track_box" opencv_apps::RotatedRectStamped #'cams-cb)
;; (ros::rate 3)

;; 対象物の方向に体を向ける
(defun cams ()
  (send *ri* :servo-on)
  (ros::rate 5)
  (dotimes (i 150)
    (ros::ros-info "box at ~A" x)
    (if (< x 120) ;; 320
	(send *ri* :go-pos 0 0 8) ())
    (if (> x 220) ;; 320
	(send *ri* :go-pos 0 0 -8) ())
    (ros::sleep)
    (ros::spin-once))
  (send *ri* :servo-off))

;; 対象物の方向に頭を回す
(defun cams-head ()
  (send *ri* :servo-on)
  (send *nao* :reset-pose)
  (send *ri* :angle-vector (send *nao* :angle-vector))
  (send *ri* :wait-interpolation)
  (ros::rate 3)
  (dotimes (i 30)
    (send *ri* :state)
    (setq pv (send *ri* :potentio-vector)) ;; 実機の角度取得
    (setq hy (elt pv 22)) ;; 22: head neck-y
    (ros::ros-info "box at ~A" x)
    (if (< x 120) (setq hy (+ hy 10)) ())
    (if (> x 220) (setq hy (- hy 10)) ())
    (send *nao* :head :neck-y :joint-angle hy)
    (send *ri* :angle-vector (send *nao* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (ros::sleep)
    (ros::spin-once))
  (send *ri* :servo-off))

;; 対象物の方向を見る
(setq tgt 0)
(setq ctheta 40) ;; カメラの中央から外側までの角度
(defun cams-look ()
  (send *ri* :servo-on)
  (send *nao* :reset-pose)
  (send *ri* :angle-vector (send *nao* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :speak "p")
  (ros::rate 5)
  (dotimes (i 45)
    (cond
     ((= i 10) (send *ri* :speak "peko"))
     ((= i 20)
      (progn (send *ri* :speak "pekopeko")
	     (setq tgt (- ctheta (/ (* ctheta x) 160)))
	     (ros::ros-info "target at ~A" tgt)
	     (send *nao* :head :neck-y :joint-angle tgt)
	     (send *ri* :angle-vector (send *nao* :angle-vector) 200)
	     (send *ri* :wait-interpolation)))
     ((= i 30)
      (progn (send *ri* :speak "peekopeko")
	     (head2arm)
	     (send *ri* :angle-vector (send *nao* :angle-vector) 500)
	     (send *ri* :wait-interpolation)
	     (send *ri* :stop-grasp)))
     )
    (ros::sleep)
    (ros::spin-once))
  ;; (send *ri* :servo-off)
  )

;; 頭の向きをもとに目標座標を作成しik
(defun head2arm ()
  (setq headcoords (send *nao* :head :neck-p :child-link)) ;; 頭末端の座標
  (setq headwc (send headcoords :worldcoords)) ;; 頭末端のworld座標
  (setq dest (send headwc :copy-worldcoords)) ;; 目標のworld座標
  (send dest :translate #f(200 0 0))
  ;; どちらの腕を使うか決める
  ;; (setq pv (send *ri* :potentio-vector)) ;; 実機の角度取得
  (setq pv (send *nao* :angle-vector)) ;; sim上の角度取得
  (setq hy (elt pv 22)) ;; 22: head neck-y   
  (setq naoarm (if (< hy 0) 0 1))
  ;; sim
  (objects (list *nao* headwc dest))
  ;; ik
  (if (= naoarm 0)
      (progn (print "rarm ik")
	     (send *nao* :rarm :inverse-kinematics
		   dest
		   :link-list (send *nao* :link-list (send *nao* :rarm :end-coords :parent))
		   :move-target (send *nao* :rarm :end-coords)
		   :rotation-axis nil
		   :thre 30
		   :debug-view t))
    (progn (print "larm ik")
	   (send *nao* :larm :inverse-kinematics
		 dest
		 :link-list (send *nao* :link-list (send *nao* :larm :end-coords :parent))
		 :move-target (send *nao* :larm :end-coords)
		 :rotation-axis nil
		 :thre 30
		 :debug-view t)))
  )

#|
(setq cc (send *nao* :head :neck-p :child-link)) ;; neck-pが末端
(setq dd (send cc :worldcoords)) ;; 頭world
(setq ee (send dd :copy-worldcoords)) ;;対象物world
(send ee :translate #f(200 0 0))
(objects (list *nao* dd ee))
|#

(defun sequence-test ()
  (send *ri* :servo-on)
  (send *nao* :reset-pose)
  (send *ri* :angle-vector (send *nao* :angle-vector) 1000)
  (send *ri* :wait-interpolation) ;; angle-vectorの後はwait-interpolation
  (send *ri* :speak "hello")
  (send *ri* :servo-off)
  )
