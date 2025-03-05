#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(spot-init)
(send *ri* :stand)
;; if dock is used
;; (setq dock-id (ros::get-param "~dock_id" 520))
;; (send *ri* :undock)

;;(send *ri* :go-to-spot "eng2_7FElevator")
(send *ri* :go-to-spot "eng2_2FElevator")
(send *ri* :speak-jp "こんにちは" :wait t)
(send *ri* :speak-en "hello" :wait t)
(send *ri* :go-to-spot "eng2_73B2")

(send *ri* :sit)
(send *ri* :stand)
;; (send *ri* :dock dock-id)
