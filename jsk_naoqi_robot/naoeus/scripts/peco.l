;; roslaunch jsk_nao_startup jsk_nao_startup.launch network_interface:=wlp5s0


(load "../nao-interface.l")
(nao-init)
;; (send *ri* :speak "hello")

;; servo-on
(defun son ()
  (send *ri* :servo-on))

;; servo-off
(defun soff ()
  (send *ri* :servo-off))

;; *nao*の関節角度を*ri*に反映させる
(defun nao2ri ()
  (send *ri* :angle-vector (send *nao* :angle-vector)))

(defun ri2nao ()
  (send *ri* :state)
  (send *nao* :angle-vector (send *ri* :potentio-vector)))

;; *nao*をreset-poseし、原点に戻す
(defun naoreset ()
  (send *nao* :reset-pose)
  (send *nao* :move-to (make-coords) :world))
