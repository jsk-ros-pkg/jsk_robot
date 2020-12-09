#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defvar *look-at-human-enabled* nil)
;; (defvar *pr2*)
;; (defvar *ri*)


(defun people-pose-array-cb (msg)
  (unless (send msg :poses)
    (return-from people-pose-array-cb nil))

  (let (limb-names limb-scores limb-poses
        (target-limbs '("nose" "left eye" "left ear" "right eye" "right ear"))
        person-count limb-count (max-score 0)
        (look-person-idx nil) (look-limb-idx nil)
        look-person look-uv look-xyz)

    ;; Search highest score for each person
    (setq person-count 0)
    (dolist (person (send msg :poses))
      (setq limb-names (send person :limb_names))
      (setq limb-scores (send person :scores))
      (setq limb-poses (send person :poses))

      ;; For each limb
      (setq limb-count 0)
      (dolist (limb limb-names)
        (dolist (target target-limbs)
          (when (and (equal (string-downcase limb) target)
                     (> (elt limb-scores limb-count) max-score))
            (setq max-score (elt limb-scores limb-count))
            (setq look-person-idx person-count)
            (setq look-limb-idx limb-count)))
        (incf limb-count))
      (incf person-count))

    ;; Do nothing when no valid limb was found
    (when (or (null look-person-idx) (null look-limb-idx))
      (return-from people-pose-array-cb nil))
    (setq look-person (elt (send msg :poses) look-person-idx))
    (when (< (elt (send look-person :scores) look-limb-idx) 0.5)
      (return-from people-pose-array-cb nil))

    ;; Look at target
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq look-uv
          (send (elt (send look-person :poses) look-limb-idx) :position))
    (setq look-xyz (scale 1e+6 (send (send *pr2* :kinect_head-rgb)
                                     :ray (send look-uv :x) (send look-uv :y))))
    (send *pr2* :head :look-at look-xyz)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 700 :head-controller)))

(defun start-look-at-human (req)
  (setq *look-at-human-enabled* t)
  (ros::subscribe
   "~input/people_pose_array" jsk_recognition_msgs::PeoplePoseArray
   #'people-pose-array-cb)
  (instance std_srvs::EmptyResponse :init))

(defun stop-look-at-human (req)
  (setq *look-at-human-enabled* nil)
  (ros::unsubscribe "~input/people_pose_array")
  (instance std_srvs::EmptyResponse :init))


;; Main process
(ros::roseus "pr2_look_at_human")

(pr2-init)

(ros::advertise-service "~start" std_srvs::Empty #'start-look-at-human)
(ros::advertise-service "~stop" std_srvs::Empty #'stop-look-at-human)
(ros::advertise "~enabled" std_msgs::Bool 1)
(unix:usleep (* 100 1000))

(ros::rate 2)
(while (ros::ok)
  (ros::spin-once)
  (ros::publish "~enabled"
                (instance std_msgs::Bool :init :data *look-at-human-enabled*))
  (ros::sleep))
