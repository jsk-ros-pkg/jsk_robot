(load "package://fetcheus/fetch-interface.l")

;; avoid print violate max/min-angle that exceeds 4M log limit
(setf (symbol-function 'warning-message-org)
      (symbol-function 'warning-message))
(defun warning-message (color format &rest mesg)
  (unless (or (substringp "violate min-angle" format)
              (substringp "violate max-angle" format))
    (apply #'warning-message-org color format mesg)))

(defun shortest-angle (d0 d1)
  (atan2 (sin (- d0 d1)) (cos (- d0 d1))))

;; initialize *fetch*

(setq *fetch* (fetch))

(while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
  (setq *ri* (instance fetch-interface :init)))

(when (send *ri* :simulation-modep)
  (ros::ros-warn "*ri* is running with simulation mode, something goes wrong ....")
  (sys::exit 1))

