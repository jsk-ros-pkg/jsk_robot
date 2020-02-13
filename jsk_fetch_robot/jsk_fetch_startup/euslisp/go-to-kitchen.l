#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus-add-srvs "gdrive_ros")


(defun gdrive-upload-video ()
  (let ((req (instance gdrive_ros::MultipleUploadRequest :init)))
    (send req :file_paths *file-paths*)
    (send req :file_titles *file-titles*)
    (send req :parents_path *parents-path*)
    (send req :use_timestamp_folder t)
    (send req :use_timestamp_file_title t)
    (ros::service-call "/gdrive_server/upload_multi" req)))

(defun send-mail (text)
  (let ((cmd (format nil "LC_CTYPE=ja_JP.UTF-8 /bin/echo -e \"~A\" | /usr/bin/mail -s \"Fetch キッチン見回りデモ\" -r ~A ~A" text *from-address* *to-address*)))
    (eq (unix::system cmd) 0)))

(defun main ()
  (fetch-init)
  (send *ri* :clear-costmap)
  (let ((success (go-to-kitchen :tweet t :n-dock-trial 3 :n-kitchen-trial 3)))
    (unix::system "rosnode kill /go_to_kitchen_head_camera_video_recorder")
    (unix::system "rosnode kill /go_to_kitchen_object_detection_video_recorder")
    (unix::system "rosnode kill /go_to_kitchen_rosbag_recorder")
    (when *mail-notification*
      (let* ((upload-res (gdrive-upload-video))
             (file-urls
               (if (and upload-res (some #'identity (send upload-res :successes)))
                 (send upload-res :file_urls) nil)))
        (send-mail
          (concatenate string
                       (if success
                         "こんにちは、Fetchです。\\nキッチンの見回りをしてきました。\\n"
                         "こんにちは、Fetchです。\\nキッチンの見回りに失敗しました。\\n")
                       (if file-urls
                         (apply #'concatenate string
                               (append (list "見回りの動画をアップロードしました。\\n")
                                       (mapcar #'(lambda (x) (format nil "URL: ~A\\n" x)) file-urls)))
                         "見回りの動画のアップロードにも失敗しました。\\n")))))))


(ros::roseus "go_to_kitchen")
(setq *mail-notification* (ros::get-param "~mail_notification" nil))
(when *mail-notification*
  (ros::roseus-add-srvs "gdrive_ros")
  (setq *from-address* (ros::get-param "~from_address"))
  (setq *to-address* (ros::get-param "~to_address"))
  (setq *file-paths* (ros::get-param "~file_paths"))
  (setq *file-titles* (ros::get-param "~file_titles"))
  (setq *parents-path* "fetch_morning_go_to_kitchen"))
(main)
(exit)
