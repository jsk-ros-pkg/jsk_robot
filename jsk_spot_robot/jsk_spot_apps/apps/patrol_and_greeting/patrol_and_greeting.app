display: Patrol And Greeting
platform: spot
launch: jsk_spot_apps/patrol_and_greeting.xml
interface: jsk_spot_apps/patrol_and_greeting.interface
plugins:
  - name: insta_360_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: patrol_and_greeting_insta_360_video.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /dual_fisheye_to_panorama/output
      video_height: 921
      video_width: 1846
      video_framerate: 8
      video_encoding: RGB
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: patrol_and_greeting.bag
      compress: true
      rosbag_topic_names:
        - /rosout
        - /tf
        - /tf_static
        - /arm_gen3/joint_states
        - /audio
        - /audio_info
        - /dual_fisheye_to_panorama/output/quater/compressed
        - /dual_fisheye_to_panorama/panorama_info
        - /joint_states
        - /laptop_charge
        - /spinal/baro
        - /spinal/imu
        - /spot/camera/back/camera_info
        - /spot/camera/back/image/compressed
        - /spot/camera/frontleft/camera_info
        - /spot/camera/frontleft/image/compressed
        - /spot/camera/frontright/camera_info
        - /spot/camera/frontright/image/compressed
        - /spot/camera/left/camera_info
        - /spot/camera/left/image/compressed
        - /spot/camera/right/camera_info
        - /spot/camera/right/image/compressed
        - /spot/depth/back/camera_info
        - /spot/depth/back/image/compressed
        - /spot/depth/frontleft/camera_info
        - /spot/depth/frontleft/image/compressed
        - /spot/depth/frontright/camera_info
        - /spot/depth/frontright/image/compressed
        - /spot/depth/left/camera_info
        - /spot/depth/left/image/compressed
        - /spot/depth/right/camera_info
        - /spot/depth/right/image/compressed
        - /spot_behavior_manager_server/current_node_id
        - /spot_kinova/joint_states
        - /spot_recognition/object_detection_image/compressed
        - /spot_recognition/elevator_door_points
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: patrol_and_greeting.yaml
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/patrol_and_greeting.yaml
        - /tmp/patrol_and_greeting.bag
        - /tmp/patrol_and_greeting_insta_360_video.avi
      upload_file_titles:
        - patrol_and_greeting.yaml
        - patrol_and_greeting.bag
        - patrol_and_greeting_insta_360_video.avi
      upload_parents_path: patrol_and_greeting
      upload_server_name: /gdrive_server
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Result of Patrol and Greeting
      sender_address: spot-jsk@jsk.imi.i.u-tokyo.ac.jp
      receiver_address: spot@jsk.imi.i.u-tokyo.ac.jp
      use_timestamp_title: true
plugin_order:
  start_plugin_order:
    - insta_360_video_recorder_plugin
    - rosbag_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - insta_360_video_recorder_plugin
    - rosbag_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
