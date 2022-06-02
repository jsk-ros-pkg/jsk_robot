display: Go to kitchen
platform: fetch
launch: jsk_fetch_startup/go_to_kitchen.xml
interface: jsk_fetch_startup/go_to_kitchen.interface
icon: jsk_fetch_startup/go_to_kitchen.png
timeout: 1200
plugins:
  - name: service_notification_saver_plugin
    type: app_notification_saver/service_notification_saver
  - name: smach_notification_saver_plugin
    type: app_notification_saver/smach_notification_saver
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
        - /dual_fisheye_to_panorama/quater/output/compressed
        - /edgetpu_object_detector/output/image/compressed
        - /head_camera/rgb/throttled/camera_info
        - /head_camera/depth_registered/throttled/camera_info
        - /head_camera/rgb/throttled/image_rect_color/compressed
        - /head_camera/depth_registered/throttled/image_rect/compressedDepth
        - /server_name/smach/container_init
        - /server_name/smach/container_status
        - /server_name/smach/container_structure
        - /audio
        - /rviz/throttled/image/compressed
  - name: head_camera_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      image_topic_name: /head_camera/rgb/throttled/image_rect_color/compressed
      image_fps: 5
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
      video_path: /tmp/go_to_kitchen_head_camera.mp4
  - name: object_detection_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      image_topic_name: /edgetpu_object_detector/output/image/compressed
      image_fps: 30
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
      video_path: /tmp/go_to_kitchen_object_detection.mp4
  - name: panorama_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      image_topic_name: /dual_fisheye_to_panorama/quater/output/compressed
      image_fps: 1
      video_path: /tmp/go_to_kitchen_panorama.mp4
  - name: rviz_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      image_topic_name: /rviz/throttled/image/compressed
      image_fps: 5
      video_path: /tmp/go_to_kitchen_rviz.mp4
  - name: respeaker_audio_converter_plugin
    type: app_recorder/rosbag_audio_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
      audio_path: /tmp/go_to_kitchen_audio.wav
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
        - /tmp/go_to_kitchen_head_camera.mp4
        - /tmp/go_to_kitchen_object_detection.mp4
        - /tmp/go_to_kitchen_panorama.mp4
        - /tmp/go_to_kitchen_rviz.mp4
        - /tmp/go_to_kitchen_audio.wav
        - /tmp/go_to_kitchen_rosbag.bag
        - /tmp/trashcan_inside.jpg
      upload_file_titles:
        - go_to_kitchen_result.yaml
        - go_to_kitchen_head_camera.mp4
        - go_to_kitchen_object_detection.mp4
        - go_to_kitchen_panorama.mp4
        - go_to_kitchen_rviz.mp4
        - go_to_kitchen_audio.wav
        - go_to_kitchen_rosbag.bag
        - trashcan_inside.jpg
      upload_parents_path: fetch_go_to_kitchen
      upload_server_name: /gdrive_server
  - name: tweet_notifier_plugin
    type: app_notifier/tweet_notifier_plugin
    plugin_args:
      client_name: /tweet_image_server/tweet
      image: true
      image_topic_name: /edgetpu_object_detector/output/image
      warning: false
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
    - rosbag_recorder_plugin
    - head_camera_converter_plugin
    - object_detection_converter_plugin
    - panorama_converter_plugin
    - rviz_converter_plugin
    - respeaker_audio_converter_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - tweet_notifier_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
    - shutdown_plugin
  stop_plugin_order:
    - move_base_cancel_plugin
    - service_notification_saver_plugin
    - smach_notification_saver_plugin
    - rosbag_recorder_plugin
    - head_camera_converter_plugin
    - object_detection_converter_plugin
    - panorama_converter_plugin
    - rviz_converter_plugin
    - respeaker_audio_converter_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - tweet_notifier_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
    - shutdown_plugin
