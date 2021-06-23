(load "package://spoteus/spot-interface.l")
(load "package://kinovaeus/kinova-interface.l")
(load "package://spotkinovaeus/spotkinova-utils.l")

(eval `(defclass spot-kinova-interface :super robot-interface
         :slots ((spot-inst :forward ,(mapcar #'car (send spot-interface :all-methods)))
                 (kinova-inst :forward ,(mapcar #'car (send kinova-interface :all-methods))))))

(defmethod spotkinova-interface
  (:init
   (&key type &rest args)
   (let ()
     (send-super* :init
                  :robot
                  ;; Create robot instance different from *kinova*
                  (case type
                        (:spot_gen3_robotiq_2f_85
                         (instance spot_gen3_robotiq_2f_85-robot :init))
                        (:spot_gen3_robotiq_2f_140
                         (instance spot_gen3_robotiq_2f_140-robot :init))
                        (:spot_gen3_lite_gen3_lite_2f
                         (instance spot_gen3_lite_gen3_lite_2f-robot :init))
                        (t
                         (warning-message 1 "unknown kinova robot type ~A~%" type)))
                  :joint-states-topic "/spot_kinova/joint_states"
                  :groupname "spot_kinova_interface" args))
   (send self :add-controller :arm-controller)
   self)
  (:angle-vector (av &rest args)
    (let* ((kinova-av (coerce (nthcdr 12 (coerce av cons)) float-vector))
           (spot-av-list (butlast (coerce av cons) 6)))
      (setq kinova-av (send kinova-inst :angle-vector kinova-av args))
      (coerce (append spot-av-list (coerce kinova-av cons)) float-vector)))
  (:angle-vector-raw (av &rest args)
    (let* ((kinova-av (coerce (nthcdr 12 (coerce av cons)) float-vector))
           (spot-av (coerce (butlast (coerce av cons) 6) float-vector)))
      (setq kinova-av (send kinova-inst :angle-vector-raw kinova-av args))
      (coerce (append (coerce spot-av cons) (coerce kinova-av cons)) float-vector)))
  (:angle-vector-sequence (avs &rest args)
    (let* ((kinova-avs
            (mapcar #'(lambda (av) (coerce (nthcdr 12 (coerce av cons)) float-vector)) avs))
           (spot-avs
            (mapcar #'(lambda (av) (coerce (butlast (coerce av cons) 6) float-vector)) avs)))
      (setq kinova-avs (send kinova-inst :angle-vector-sequence kinova-avs args))
      (mapcar #'(lambda (spot-av kinova-av)
                  (coerce (append (coerce spot-av cons) (coerce kinova-av cons)) float-vector))
              spot-avs kinova-avs)))
  (:angle-vector-sequence-raw (avs &rest args)
    (let* ((kinova-avs
            (mapcar #'(lambda (av) (coerce (nthcdr 12 (coerce av cons)) float-vector)) avs))
           (spot-avs
            (mapcar #'(lambda (av) (coerce (butlast (coerce av cons) 6) float-vector)) avs)))
      (setq kinova-avs (send kinova-inst :angle-vector-sequence-raw kinova-avs args))
      (mapcar #'(lambda (spot-av kinova-av)
                  (coerce (append (coerce spot-av cons) (coerce kinova-av cons)) float-vector))
              spot-avs kinova-avs)))
  )
