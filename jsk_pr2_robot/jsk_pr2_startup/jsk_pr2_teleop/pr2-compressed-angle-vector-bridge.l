(ros::roseus-add-msgs "jsk_pr2_startup")

(defun compress-angle-vector (robot av &optional tms)
  (if tms
      (error "time is not impremented!"))
  (let ((initial-pose (send robot :angle-vector))
        msg)
    (send robot :angle-vector av)
    (setq msg (instance jsk_pr2_startup::AngleVectorCompressed :init
                        :angles (coerce
                                 (mapcar #'(lambda (jt)
                                             (ros::ros-info "~A ~A ~A"
                                                            (send jt :joint-angle)
                                                            (send jt :min-angle)
                                                            (send jt :max-angle))
                                             (if (infinite-joint-p jt)
                                                 (round (* (/
                                                            (abs (mod (send jt :joint-angle) 360))
                                                            360.0)
                                                           255))
                                               (round (* (/ (- (send jt :joint-angle)
                                                               (send jt :min-angle))
                                                            (- (send jt :max-angle)
                                                               (send jt :min-angle)))
                                                         255))))
                                         (send robot :joint-list))
                                 string)))
    (send robot :angle-vector initial-pose)
    msg))

(defun decompress-angle-vector (robot msg)
  (ros::ros-info "decompress ~A ~A" robot msg)
  (coerce
   (mapcar #'(lambda (jt-uint-ang)
               (let ((jt (car jt-uint-ang))
                     (uint-ang (cdr jt-uint-ang)))
                 (if (infinite-joint-p jt)
                     (* (/ uint-ang 255.0) 360.0)
                   (+ (send jt :min-angle)
                      (* (/ uint-ang 255.0)
                         (- (send jt :max-angle)
                            (send jt :min-angle)))))))
           (pairlis
            (send robot :joint-list)
            (coerce (send msg :angles) cons)))
   float-vector))

(defun infinite-joint-p (jt)
  (or (> (send jt :max-angle) 360)
      (< (send jt :min-angle) -360)))

#|
  (list (mapcar #'(lambda (av-uint-msg)
                    (let ((uint-av (send av-uint-msg :data)))
                      (mapcar #'(lambda (jt-ang)
                                  (+ (send (car jt-ang) :min-angle)
                                     (* (cdr jt-ang)
                                        (- (send (car jt-ang) :max-angle))
                                        (send (car jt-ang) :min-angle)))))
                      (pairlst (send robot :joint-list)
                               uint-av)))
                (send msg :angles))
        (send (send msg :times) :data)))
|#
(provide :pr2-compressed-angle-vector-bridge)
