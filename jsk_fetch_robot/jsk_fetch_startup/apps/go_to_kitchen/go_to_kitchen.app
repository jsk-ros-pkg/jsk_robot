display: Go to kitchen
platform: fetch
launch: jsk_fetch_startup/go_to_kitchen.xml
interface: jsk_fetch_startup/go_to_kitchen.interface
icon: jsk_fetch_startup/go_to_kitchen.png
plugins:
  - name: head_camera_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: go_to_kitchen_head_camera.avi
      video_topic_name: /head_camera/rgb/throttled/image_rect_color
      video_fps: 5.0
  - name: object_detection_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: go_to_kitchen_object_detection.avi
      video_topic_name: /edgetpu_object_detector/output/image
      video_fps: 15.0
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      rosbag_topic_names: /tf /joint_states /odom /odom_combined /cmd_vel
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/go_to_kitchen_head_camera.avi
        - /tmp/go_to_kitchen_object_detection.avi
        - /tmp/go_to_kitchen_rosbag.bag
      upload_file_titles:
        - go_to_kitchen_head_camera.avi
        - go_to_kitchen_object_detection.avi
        - go_to_kitchen_rosbag.bag
      upload_parents_path: fetch_morning_go_to_kitchen
      upload_server_name: /gdrive_server
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Fetch kitchen patrol demo
      sender_address: fetch15@jsk.imi.i.u-tokyo.ac.jp
      receiver_address: fetch@jsk.imi.i.u-tokyo.ac.jp
