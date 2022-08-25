(require "package://naoeus/nao.l")

(defmethod NaoH25V50-robot
  (:interlocking-joint-pairs
   ()
   "Interlocking joint pairs."
   (list (cons (send self :lhipyawpitch) (send self :rhipyawpitch))))
  (:inverse-kinematics
   (target-coords
    &rest args
    &key
    (additional-jacobi
     (list #'(lambda (link-list) (send self :calc-jacobian-for-interlocking-joints link-list))))
    (additional-vel
     (list #'(lambda (link-list) (send self :calc-vel-for-interlocking-joints link-list))))
    (move-joints-hook
     #'(lambda () (send self :set-midpoint-for-interlocking-joints) nil))
    &allow-other-keys)
   (send-super* :inverse-kinematics target-coords
		:additional-jacobi additional-jacobi
		:additional-vel additional-vel
		:move-joints-hook move-joints-hook
		args))
  )
