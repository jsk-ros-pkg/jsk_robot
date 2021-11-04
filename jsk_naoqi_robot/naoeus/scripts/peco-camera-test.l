;; カメラ
;; $ roslaunch opencv_apps camshift.launch image:=/nao_robot/naoqi_driver/camera/bottom/image_raw
(ros::roseus-add-msgs "opencv_apps")
(setq x 0) ;; 対象物位置

;; camshiftのコールバック関数
(defun cams-cb (rect)
  (setq x (send rect :rect :center :x)))
(ros::subscribe "camshift/track_box" opencv_apps::RotatedRectStamped #'cams-cb)
(ros::rate 3)

;; 対象物の方向に体を向ける
(defun cams ()
  (send *ri* :servo-on)
  (dotimes (i 150)
    (ros::ros-info "box at ~A" x)
    (if (< x 120) ;; 320
	(send *ri* :go-pos 0 0 8) ())
    (if (> x 220) ;; 320
	(send *ri* :go-pos 0 0 -8) ())
    (ros::sleep)
    (ros::spin-once))
  (send *ri* :servo-off))

;; 対象物の方向を見る
(setq hy 0)
(defun cams-head ()
  (send *ri* :servo-on)
  (dotimes (i 10)
    (ros::ros-info "box at ~A" x)
    (if (< x 120) (setq hy (+ hy 2)) ())
    (if (> x 220) (setq hy (- hy 2)) ())
    (send *nao* :head :neck-y :joint-angle hy)
    (send *ri* :angle-vector (send *nao* :angle-vector))
    (ros::sleep)
    (ros::spin-once))
  (send *ri* :servo-off))
