(require :mycobot "package://mycoboteus/mycobot.l")

(unless (assoc :init-orig (send mycobot-robot :methods))
  (rplaca (assoc :init (send mycobot-robot :methods)) :init-orig))

(defmethod mycobot-robot
  (:init
   (&rest args)
   (send* self :init-orig args)))

(provide :mycobot-utils "package://mycoboteus/mycobot-utils.l")
