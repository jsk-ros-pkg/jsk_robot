(ros::load-ros-manifest "app_notification_saver")

(defun notify-app (title stamp location message)
  "
This method sends service call to app_notification_saver to save app notification to json file.

Args:
  stamp(ros::time): UNIX time when the event occurred
  title (str)  : Notification title (e.g. object detection, navigation faliure ...) # NOQA
  message (str): Notification message

Returns:
  Result of whether the json was saved. (bool)
"
  (let ((req (instance app_notification_saver::SaveAppNotificationRequest :init)))
    (send req :title title)
    (send req :stamp stamp)
    (send req :location location)
    (send req :message message)
    (ros::service-call "/service_notification_saver/save_app_notification" req)))
