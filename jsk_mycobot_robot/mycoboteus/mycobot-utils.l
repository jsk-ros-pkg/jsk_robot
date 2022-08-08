(require :mycobot "package://mycoboteus/mycobot.l")


(unless (assoc :init-orig (send mycobot-robot :methods))
  (rplaca (assoc :init (send mycobot-robot :methods)) :init-orig))


(defmethod mycobot-robot
  (:init
   (&rest args)
   ;; grey
   (dolist (b (list
                :_make_instance_link1_geom0
                :_make_instance_link7_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.5 0.5 0.5 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send  (class self) :methods))))))
   (send* self :init-orig args))


(provide :mycobot-utils "package://mycoboteus/mycobot-utils.l")
