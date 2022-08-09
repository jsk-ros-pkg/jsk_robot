display: Go to kitchen
platform: fetch
launch: jsk_fetch_startup/go_to_kitchen.xml
interface: jsk_fetch_startup/go_to_kitchen.interface
icon: jsk_fetch_startup/go_to_kitchen.png
plugins:
  - name: service_notification_saver_plugin
    type: app_notification_saver/service_notification_saver
  - name: smach_notification_saver_plugin
    type: app_notification_saver/smach_notification_saver
  - name: head_camera_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: go_to_kitchen_head_camera.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /head_camera/rgb/image_rect_color
      video_height: 480
      video_width: 640
      video_framerate: 30
      video_encoding: RGB
  - name: object_detection_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: go_to_kitchen_object_detection.avi
      video_topic_name: /edgetpu_object_detector_visualization/output
      video_fps: 5.0
  - name: panorama_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: go_to_kitchen_panorama.avi
      video_topic_name: /dual_fisheye_to_panorama/output
      video_fps: 1.0
  - name: respeaker_audio_recorder_plugin
    type: app_recorder/audio_recorder_plugin
    launch_args:
      audio_path: /tmp
      audio_title: go_to_kitchen_audio.wav
      audio_topic_name: /audio
      audio_format: wave
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
        - /move_base/navigation_plan_viz
        - /move_base/global_plan_viz
        - /move_base/local_plan_viz
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
        - /rviz/throttled/image/compressed
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: go_to_kitchen_result.yaml
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/go_to_kitchen_result.yaml
        - /tmp/go_to_kitchen_head_camera.avi
        - /tmp/go_to_kitchen_object_detection.avi
        - /tmp/go_to_kitchen_panorama.avi
        - /tmp/go_to_kitchen_audio.wav
        - /tmp/go_to_kitchen_rosbag.bag
      upload_file_titles:
        - go_to_kitchen_result.yaml
        - go_to_kitchen_head_camera.avi
        - go_to_kitchen_object_detection.avi
        - go_to_kitchen_panorama.avi
        - go_to_kitchen_audio.wav
        - go_to_kitchen_rosbag.bag
      upload_parents_path: fetch_go_to_kitchen
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
  - name: move_base_cancel_plugin
    type: app_publisher/rostopic_publisher_plugin
    plugin_args:
      stop_topics:
        - name: "/move_base/cancel"
          pkg: actionlib_msgs
          type: GoalID
  - name: shutdown_plugin
    type: app_publisher/rostopic_publisher_plugin
    plugin_args:
      stop_topics:
        - name: "/shutdown"
          pkg: std_msgs
          type: Empty
          cond:
            - timeout
            - failure
plugin_order:
  start_plugin_order:
    - move_base_cancel_plugin
    - service_notification_saver_plugin
    - smach_notification_saver_plugin
    - head_camera_video_recorder_plugin
    - object_detection_video_recorder_plugin
    - panorama_video_recorder_plugin
    - respeaker_audio_recorder_plugin
    - rosbag_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
    - shutdown_plugin
  stop_plugin_order:
    - move_base_cancel_plugin
    - service_notification_saver_plugin
    - smach_notification_saver_plugin
    - head_camera_video_recorder_plugin
    - object_detection_video_recorder_plugin
    - panorama_video_recorder_plugin
    - respeaker_audio_recorder_plugin
    - rosbag_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
    - shutdown_plugin
