#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :base64 "lib/llib/base64.l")

(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::advertise "/photo_taken" sensor_msgs::Image 1)

(defun image-cb (msg)
  (setq *image* msg))

(defun go-to-kitchen (&key (tweet t) (n-dock-trial 1) (n-kitchen-trial 1)
                           (control-switchbot :api))
  ;; go to kitchen
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (report-start-go-to-kitchen)
  ;; Check if the lights are on in the room
  (let ((initial-light-on (get-light-on))
        (success-go-to-kitchen nil)
        (success-go-to-trashcan nil)
        (success-auto-dock nil)
        (success-battery-charging nil))
    (store-params)
    ;; turn on light
    (if initial-light-on
      (report-light-on)
      (progn
        (report-light-off)
        (room-light-on :control-switchbot control-switchbot)))
    ;; change the inflation_radius
    (inflation-loose)
    ;; go to kitchen sink
    (setq success-go-to-kitchen (move-to-sink-front :n-trial n-kitchen-trial))
    ;; report result to go to kitchen
    (if success-go-to-kitchen
      (inspect-kitchen :tweet tweet)
      (report-move-to-sink-front-failure))
    ;; go to kitchen trash can
    (setq success-go-to-trashcan (move-to-trashcan-front :n-trial n-trashcan-trial))
    ;; report result to go to trash can
    (if success-go-to-trashcan
      (inspect-trashcan :tweet tweet)
      (report-move-to-trashcan-front-failure))
    ;; go back from dock
    (report-auto-dock)
    (setq success-auto-dock (auto-dock :n-trial n-dock-trial :clear-costmap nil))
    ;; turn off light
    (if (and success-auto-dock (not initial-light-on))
      (room-light-off :control-switchbot control-switchbot))
    ;; change the inflation_radius
    (restore-params)
    (send-kitchen-mail)
    (setq success-battery-charging (progn (wait-until-is-charging)
                                          (equal (get-battery-charging-state) :charging)))
    (and success-go-to-kitchen success-auto-dock success-battery-charging)))

(defun go-to-kitchen-state-machine ()
  (setq *sm*
        (make-state-machine
          '((:init -> :report-start-go-to-kitchen)
            (:report-start-go-to-kitchen -> :get-light-on)
            (:get-light-on -> :report-light-on)
            (:get-light-on !-> :room-light-on)
            (:report-light-on -> :move-to-dock-front)
            (:room-light-on -> :move-to-dock-front)

            (:move-to-dock-front -> :inspect-dock-front)
            (:move-to-dock-front !-> :report-move-to-dock-front-failure)
            (:inspect-dock-front -> :move-to-tv-front)
            (:report-move-to-dock-front-failure -> :move-to-tv-front)

            (:move-to-tv-front -> :inspect-tv-front)
            (:move-to-tv-front !-> :report-move-to-tv-front-failure)
            (:inspect-tv-front -> :move-to-tv-desk)
            (:report-move-to-tv-front-failure -> :move-to-tv-desk)

            (:move-to-tv-desk -> :inspect-tv-desk)
            (:move-to-tv-desk !-> :report-move-to-tv-desk-failure)
            (:inspect-tv-desk -> :move-to-desk-back)
            (:report-move-to-tv-desk-failure -> :move-to-desk-back)

            (:move-to-desk-back -> :inspect-desk-back)
            (:move-to-desk-back !-> :report-move-to-desk-back-failure)
            (:inspect-desk-back -> :move-to-desk-front)
            (:report-move-to-desk-back-failure -> :move-to-desk-front)

            (:move-to-desk-front -> :inspect-desk-front)
            (:move-to-desk-front !-> :report-move-to-desk-front-failure)
            (:inspect-desk-front -> :move-to-kitchen-door-front)
            (:report-move-to-desk-front-failure -> :move-to-kitchen-door-front)

            (:move-to-kitchen-door-front -> :inspect-kitchen-door-front)
            (:move-to-kitchen-door-front !-> :report-move-to-kitchen-door-front-failure)
            (:inspect-kitchen-door-front -> :move-to-sink-front)
            (:report-move-to-kitchen-door-front-failure -> :move-to-sink-front)

            (:move-to-sink-front -> :inspect-kitchen)
            (:move-to-sink-front !-> :report-move-to-sink-front-failure)
            ;;(:inspect-kitchen -> :auto-dock)
            (:report-move-to-sink-front-failure -> :auto-dock)
            (:inspect-kitchen -> :move-to-trashcan-front)
            (:move-to-trashcan-front -> :inspect-trashcan)
            (:move-to-trashcan-front !-> :report-move-to-trashcan-front-failure)
            (:inspect-trashcan -> :auto-dock)
            (:report-move-to-trashcan-front-failure -> :auto-dock)
            (:auto-dock -> :room-light-off)
            (:auto-dock !-> :auto-dock-failure)
            (:auto-dock-failure -> :room-light-off)
            (:room-light-off -> :finish)
            (:finish -> t)
            (:finish !-> nil))
          '((:init
              '(lambda (userdata)
                 (fetch-init)
                 (undock)
                 (send *ri* :clear-costmap)
                 (store-params)
                 (inflation-loose)
                 (clear-app-notification)
                 (ros::subscribe "/edgetpu_object_detector/output/image/compressed"
                                 sensor_msgs::CompressedImage #'image-cb)
                 t))
            (:report-start-go-to-kitchen
              '(lambda (userdata)
                 (report-start-go-to-kitchen)
                 (set-alist 'description "キッチンを見に行くよ" userdata)
                 (set-alist 'image "" userdata)
                 t))
            (:get-light-on
              '(lambda (userdata)
                 (let ((light-on (get-light-on)))
                   (setf (cdr (assoc 'initial-light-on userdata)) light-on)
                   (ros::spin-once)
                   (if light-on
                       (set-alist 'description "電気がついていたよ" userdata)
                       (set-alist 'description "電気がついていなかったよ" userdata))
                   (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                   light-on)))
            (:report-light-on
              '(lambda (userdata)
                (report-light-on)
                (set-alist 'description "" userdata)
                (set-alist 'image "" userdata)
                 t))
            (:room-light-on
              '(lambda (userdata)
                (let ((control-switchbot (cdr (assoc 'control-switchbot userdata)))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                   (report-light-off)
                   (room-light-on :control-switchbot control-switchbot)
                   (ros::spin-once)
                   (set-alist 'description "電気をつけたよ" userdata)
                   (if label-names
                       (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                       (set-alist 'image "" userdata))
                   t)))
            (:move-to-dock-front
             '(lambda (userdata)
                 (let* ((n-trial (cdr (assoc 'n-dock-front-trial userdata)))
                        (success (move-to-dock-front :n-trial n-trial))
                        (label-names (notify-recognition :location "on the way to kitchen")))
                   (setf (cdr (assoc 'success-go-to-dock-front userdata)) success)
                   (ros::spin-once)
                   (if success
                       (set-alist 'description "ドックの前に移動したよ" userdata)
                       (set-alist 'description "ドックの前に移動しようとしたけど，迷子になっちゃった" userdata))
                   (if label-names
                       (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                       (set-alist 'image "" userdata))
                   success)))
            (:inspect-dock-front
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (inspect-dock-front :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description "ドックの前の様子を見たよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:report-move-to-dock-front-failure
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (report-move-to-dock-front-failure)
                 (ros::spin-once)
                 (set-alist 'description "ドックの前に移動できなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:move-to-tv-front
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-tv-front-trial userdata)))
                      (success (move-to-tv-front :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                 (setf (cdr (assoc 'success-go-to-tv-front userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "テレビの前に移動したよ" userdata)
                     (set-alist 'description "テレビの前に移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 success)))
            (:inspect-tv-front
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (inspect-tv-front :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description "テレビの前の様子を見たよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:report-move-to-tv-front-failure
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (report-move-to-tv-front-failure)
                 (ros::spin-once)
                 (set-alist 'description "テレビの前に移動できなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:move-to-tv-desk
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-tv-desk-trial userdata)))
                      (success (move-to-tv-desk :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                 (setf (cdr (assoc 'success-go-to-tv-desk userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "机の前に移動したよ" userdata)
                     (set-alist 'description "机の前に移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 success)))
            (:inspect-tv-desk
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (inspect-tv-desk :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description "机の様子を確認したよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:report-move-to-tv-desk-failure
             '(lambda (userdata)
               (let* (label-names (notify-recognition :location "on the way to kitchen"))
                 (report-move-to-tv-desk-failure)
                 (ros::spin-once)
                 (set-alist 'description "机の前に移動できなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:move-to-desk-back
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-desk-back-trial userdata)))
                      (success (move-to-desk-back :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                 (setf (cdr (assoc 'success-go-to-desk-back userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "部屋の後ろに移動したよ" userdata)
                     (set-alist 'description "部屋の後ろに移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 success)))
            (:inspect-desk-back
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (inspect-desk-back :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description "部屋の後ろを確認したよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:report-move-to-desk-back-failure
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (report-move-to-desk-back-failure)
                 (ros::spin-once)
                 (set-alist 'description "部屋の後ろに移動できなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:move-to-desk-front
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-desk-front-trial userdata)))
                      (success (move-to-desk-front :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to desk-front")))
                 (setf (cdr (assoc 'success-go-to-desk-front userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "部屋の前に移動したよ" userdata)
                     (set-alist 'description "部屋の前に移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 success)))
            (:inspect-desk-front
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (inspect-desk-front :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description "部屋の前を確認したよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:report-move-to-desk-front-failure
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to kitchen")))
                 (report-move-to-desk-front-failure)
                 (ros::spin-once)
                 (set-alist 'description "部屋の前に移動できなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:move-to-kitchen-door-front
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-kitchen-door-front-trial userdata)))
                      (success (move-to-kitchen-door-front :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                 (setf (cdr (assoc 'success-go-to-tv-desk userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "ドアの前に移動したよ" userdata)
                     (set-alist 'description "ドアの前に移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                     success)))
            (:inspect-kitchen-door-front
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "kitchen-door-front")))
                 (inspect-kitchen-door-front :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description "ドアの前からキッチンの様子を見たよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 t)))
            (:report-move-to-kitchen-door-front-failure
             '(lambda (userdata)
               (let* (label-names (notify-recognition :location "on the way to kitchen"))
                 (report-move-to-kitchen-door-front-failure)
                 (ros::spin-once)
                 (set-alist 'description "ドアの前に移動できなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
                 t)))
            (:move-to-sink-front
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-kitchen-trial userdata)))
                      (success (move-to-sink-front :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                 (setf (cdr (assoc 'success-go-to-kitchen userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "キッチンに移動したよ" userdata)
                     (set-alist 'description "キッチンに移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                     (set-alist 'image "" userdata))
                 success)))
            (:inspect-kitchen
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "kitchen"))
                      (notify-text (if label-names (format nil "~Aがあったよ" label-names) "")))
                 (inspect-kitchen :tweet (cdr (assoc 'tweet userdata)))
                 (ros::spin-once)
                 (set-alist 'description (format nil "キッチンの様子を見たよ。~A" notify-text) userdata)
               (if label-names
                   (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                   (set-alist 'image "" userdata)))
               t))
            (:report-move-to-sink-front-failure
             '(lambda (userdata)
               (let* ((label-names (notify-recognition :location "on the way to sink-front")))
                 (report-move-to-sink-front-failure)
                 (ros::spin-once)
                 (set-alist 'description "キッチンに行けなかったよ" userdata)
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
                 t)))
            (:move-to-trashcan-front
             '(lambda (userdata)
               (let* ((n-trial (cdr (assoc 'n-trashcan-trial userdata)))
                      (success (move-to-trashcan-front :n-trial n-trial))
                      (label-names (notify-recognition :location "on the way to kitchen")))
                 (setf (cdr (assoc 'success-go-to-trashcan userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "ゴミ箱の前に移動したよ" userdata)
                   (set-alist 'description "ゴミ箱の前に移動しようとしたけど，迷子になっちゃった" userdata))
                 (if label-names
                     (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                   (set-alist 'image "" userdata))
                 success)))
            (:inspect-trashcan
             '(lambda (userdata)
               (inspect-trashcan :tweet (cdr (assoc 'tweet userdata)))
               (ros::spin-once)
               (set-alist 'description "ゴミ箱の様子を確認したよ" userdata)
               (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
               t))
            (:report-move-to-trashcan-front-failure
             '(lambda (userdata)
               (report-move-to-trashcan-front-failure)
               (ros::spin-once)
               (set-alist 'description "ゴミ箱の前に行けなかったよ" userdata)
               (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
               t))
            (:auto-dock
             '(lambda (userdata)
               (report-auto-dock)
               (let* ((n-trial (cdr (assoc 'n-dock-trial userdata)))
                      (success (auto-dock :n-trial n-trial :clear-costmap nil)))
                 (setf (cdr (assoc 'success-auto-dock userdata)) success)
                 (ros::spin-once)
                 (if success
                     (set-alist 'description "帰ってきたよ" userdata)
                     (set-alist 'description "帰ってこようとしたけど，迷子になっちゃった" userdata))
                 (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
                 success)))
            (:auto-dock-failure
             '(lambda (userdata)
               (report-auto-dock-failure)
               (ros::spin-once)
               (set-alist 'description "帰ってこられなかったよ" userdata)
               (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata)
               t))
            (:room-light-off
             '(lambda (userdata)
               (let ((success-auto-dock (cdr (assoc 'success-auto-dock userdata)))
                     (initial-light-on (cdr (assoc 'initial-light-on userdata)))
                     (control-switchbot (cdr (assoc 'control-switchbot userdata))))
                 (if (and success-auto-dock (not initial-light-on))
                     (progn
                       (room-light-off :control-switchbot control-switchbot)
                       (ros::spin-once)
                       (set-alist 'description "電気を消したよ" userdata)
                       (set-alist 'image "" userdata))))
               t))
            (:finish
             '(lambda (userdata)
               (let ((success-battery-charging
                      (progn (wait-until-is-charging)
                             (equal (get-battery-charging-state) :charging)))
                     (success-auto-dock (cdr (assoc 'success-auto-dock userdata)))
                     (success-go-to-kitchen
                      (cdr (assoc 'success-go-to-kitchen userdata))))
                 (restore-params)
                 (ros::spin-once)
                 (set-alist 'description "キッチンデモを終えるよ" userdata)
                 (set-alist 'image "" userdata)
                 (and success-go-to-kitchen success-auto-dock success-battery-charging)))))
          '(:init)
          '(t nil))))

(defun main (&key (tweet t) (n-dock-trial 3) (n-kitchen-trial 3) (n-trashcan-trial 3) (n-dock-front-trial 3) (n-tv-front-trial 3) (n-tv-desk-trial 3) (n-desk-back-trial 3) (n-desk-front-trial 3) (n-kitchen-door-front-trial 3) (control-switchbot :api))
  (when (not (boundp '*sm*))
    (go-to-kitchen-state-machine))
  (let ((result-state
          (exec-state-machine *sm* `((tweet . ,tweet)
                                     (n-kitchen-trial . ,n-kitchen-trial)
                                     (n-trashcan-trial . ,n-trashcan-trial)
                                     (n-dock-trial . ,n-dock-trial)
                                     (n-dock-front-trial . ,n-dock-front-trial)
                                     (n-tv-front-trial . ,n-tv-front-trial)
                                     (n-tv-desk-trial . ,n-tv-desk-trial)
                                     (n-desk-back-trial . ,n-desk-back-trial)
                                     (n-desk-front-trial . ,n-desk-front-trial)
                                     (n-kitchen-door-front-trial . ,n-kitchen-door-front-trial)
                                     (control-switchbot . ,control-switchbot)
                                     (initial-light-on . nil)
                                     (success-go-to-dock-front . nil)
                                     (success-go-to-tv-front . nil)
                                     (success-go-to-tv-desk . nil)
                                     (success-go-to-desk-back . nil)
                                     (success-go-to-desk-front . nil)
                                     (success-go-to-tv-desk . nil)
                                     (success-go-to-kitchen . nil)
                                     (success-go-to-trashcan . nil)
                                     (success-auto-dock . nil))
                              :hz 2.0)))
    (send result-state :name)))


(ros::roseus "go_to_kitchen")
(if (main) (unix::exit 0) (unix::exit 1))
