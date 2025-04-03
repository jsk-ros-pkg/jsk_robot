(defun calc-distance-object-to-path (object-coords path-coords-list)
  (let (nearest-distance temp-distance)
    (dolist (path-coords path-coords-list)
      (setq temp-distance (distance (send path-coords :pos) (send object-coords :pos)))
      (if (not nearest-distance)
          (setq nearest-distance temp-distance)
        (progn
          (if (< temp-distance nearest-distance)
              (setq nearest-distance temp-distance)
            )
          )
        )
      )
    nearest-distance))

(defun check-object-on-path (object-coords path-coords-list &key (threshold 2000))
  (let (distance)
    (setq distance (calc-distance-object-to-path object-coords path-coords-list))
    (ros::ros-info "distance: ~A" distance)
    (< distance threshold)))

(defun get-nearest-object-to-path (objects-coords-list path-coords-list)
  (let (nearest-object-coords nearest-distance temp-distance)
    (dolist (object-coords objects-coords-list)
      (setq temp-distance
              (calc-distance-object-to-path
                object-coords
                path-coords-list))
      (if (not nearest-distance)
        (progn
          (setq nearest-distance temp-distance)
          (setq nearest-object-coords object-coords)
          ))
      (if (< temp-distance nearest-distance)
        (progn
          (setq nearest-distance temp-distance)
          (setq nearest-object-coords object-coords)
          ))
      )
    nearest-object-coords))
