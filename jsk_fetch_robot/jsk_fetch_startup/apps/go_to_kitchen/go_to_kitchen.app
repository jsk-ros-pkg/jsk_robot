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
      compress: true
      rosbag_topic_names:
        - /rosout
        - /tf
        - /tf_static
        - /joint_states
        - /map
        - /odom
        - /odom_combined
        - /cmd_vel
        - /move_base/NavFnROS/plan
        - /move_base/TrajectoryPlannerROS/global_plan
        - /move_base/TrajectoryPlannerROS/local_plan
        - /move_base/global_costmap/footprint
        - /spots_marker_array
        - /spots_pictogram
        - /safe_teleop_base/local_costmap/costmap
        - /move_base/local_costmap/costmap
        - /move_base/global_costmap/costmap
        - /particlecloud
        - /base_scan/throttled
        - /head_camera/rgb/throttled/camera_info
        - /head_camera/depth_registered/throttled/camera_info
        - /head_camera/rgb/throttled/image_rect_color/compressed
        - /head_camera/depth_registered/throttled/image_rect/compressedDepth
        - /audio
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
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Fetch kitchen patrol demo
      use_timestamp_title: true
    plugin_arg_yaml: /var/lib/robot/fetch_mail_notifier_plugin.yaml
plugin_order:
  start_plugin_order:
    - head_camera_video_recorder_plugin
    - object_detection_video_recorder_plugin
    - rosbag_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - head_camera_video_recorder_plugin
    - object_detection_video_recorder_plugin
    - rosbag_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
