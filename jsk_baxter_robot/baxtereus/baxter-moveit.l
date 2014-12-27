(require :baxter-interface
         "package://baxtereus/baxter-interface.l")
(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(load "package://pr2eus_moveit/tutorials/collision-object-sample.l")

(defclass baxter-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod baxter-moveit-environment
  (:init
   (&key ((:robot rb) (baxter)))
   (send-super :init :robot rb
               :frame-id "WAIST_LINK0"
               :multi-dof-joint-name "virtual_joint"
               :multi-dof-frame-id   "/odom")
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :target-link
                     (send self :search-link-from-name "right_s0"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :target-link
                     (send self :search-link-from-name "left_s0"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         )
   )
  )

(defun sync-larm (&optional (tm 500))
  (let ((av (send *ri* :state :reference-vector)))
    (send *ri* :robot :angle-vector av)
    (send *ri* :angle-vector av tm)
    (send *ri* :wait-interpolation)
    (send *ri* :remove-joint-group "left_arm")
    (unix::usleep (* 100 1000))
    (send *ri* :add-joint-group "left_arm"
          (list "left_s0" "left_s1" "left_e0" "left_e1" "left_w0" "left_w1" "left_w2"))
    (unix::usleep (* 100 1000))
    ))
