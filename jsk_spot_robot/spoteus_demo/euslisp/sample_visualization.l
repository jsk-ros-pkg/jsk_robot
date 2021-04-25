#!/usr/bin/env roseus

;;
;; This script is for demonstration of visualization of spot's angle vector and global pose with irtviewer
;;

(load "package://spoteus/spot-interface.l")

(spot-init t) ;; create-viewer
(send *irtviewer* :draw-floor t)

;; if not power-on, power on
;; if not stand, stand

(do-until-key
 ;; update body posture
 (send *spot* :angle-vector (send *ri* :state :angle-vector))
 ;;
 (send *spot* :move-to (send *ri* :state :worldcoords) :world)
 (send *irtviewer* :look1 (send *spot* :worldpos))
 (send *irtviewer* :draw-objects)
 (x::window-main-one)
 )
