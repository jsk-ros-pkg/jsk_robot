display: Welcome to jsk
platform: fetch
launch: jsk_fetch_startup/welcome_to_jsk.xml
interface: jsk_fetch_startup/welcome_to_jsk.interface
icon: jsk_fetch_startup/welcome_to_jsk.png
plugins:
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: welcome_to_jsk_rosbag.bag
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
        - /rviz/image/compressed
        - /audio
  - name: head_camera_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: welcome_to_jsk_rosbag.bag
      image_topic_name: /head_camera/rgb/throttled/image_rect_color/compressed
      image_fps: 5
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
      video_path: /tmp/welcome_to_jsk_head_camera.mp4
  - name: object_detection_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: welcome_to_jsk_rosbag.bag
      image_topic_name: /edgetpu_object_detector/output/image/compressed
      image_fps: 30
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
      video_path: /tmp/welcome_to_jsk_object_detection.mp4
  - name: panorama_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: welcome_to_jsk_rosbag.bag
      image_topic_name: /dual_fisheye_to_panorama/quater/output/compressed
      image_fps: 1
      video_path: /tmp/welcome_to_jsk_panorama.mp4
  - name: rviz_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: welcome_to_jsk_rosbag.bag
      image_topic_name: /rviz/image/compressed
      image_fps: 30
      video_path: /tmp/welcome_to_jsk_rviz.mp4
  - name: respeaker_audio_converter_plugin
    type: app_recorder/rosbag_audio_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: welcome_to_jsk_rosbag.bag
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
      audio_path: /tmp/welcome_to_jsk_audio.wav
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: welcome_to_jsk_result.yaml
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/welcome_to_jsk_result.yaml
        - /tmp/welcome_to_jsk_head_camera.mp4
        - /tmp/welcome_to_jsk_object_detection.mp4
        - /tmp/welcome_to_jsk_panorama.mp4
        - /tmp/welcome_to_jsk_rviz.mp4
        - /tmp/welcome_to_jsk_audio.wav
        - /tmp/welcome_to_jsk_rosbag.bag
      upload_file_titles:
        - welcome_to_jsk_result.yaml
        - welcome_to_jsk_head_camera.mp4
        - welcome_to_jsk_object_detection.mp4
        - welcome_to_jsk_panorama.mp4
        - welcome_to_jsk_rviz.mp4
        - welcome_to_jsk_audio.wav
        - welcome_to_jsk_rosbag.bag
      upload_parents_path: fetch_welcome_to_jsk
      upload_server_name: /gdrive_server
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Welcome to JSK demo
      use_timestamp_title: true
    plugin_arg_yaml: /var/lib/robot/fetch_mail_notifier_plugin.yaml
  - name: move_base_cancel_plugin
    type: app_publisher/rostopic_publisher_plugin
    plugin_args:
      stop_topics:
        - name: "/move_base/cancel"
          pkg: actionlib_msgs
          type: GoalID
plugin_order:
  start_plugin_order:
    - move_base_cancel_plugin
    - rosbag_recorder_plugin
    - head_camera_converter_plugin
    - object_detection_converter_plugin
    - panorama_converter_plugin
    - rviz_converter_plugin
    - respeaker_audio_converter_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - move_base_cancel_plugin
    - rosbag_recorder_plugin
    - head_camera_converter_plugin
    - object_detection_converter_plugin
    - panorama_converter_plugin
    - rviz_converter_plugin
    - respeaker_audio_converter_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
