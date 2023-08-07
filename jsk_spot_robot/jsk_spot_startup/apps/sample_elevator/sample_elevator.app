display: Sample Elevator
platform: spot
launch: jsk_spot_startup/sample_elevator.xml
interface: jsk_spot_startup/sample_elevator.interface
plugins:
  - name: panorama_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: sample_elevator_panorama.avi
      video_topic_name: /dual_fisheye_to_panorama/output
      video_fps: 1.0
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: sample_elevator_rosbag.bag
      compress: true
      rosbag_topic_names:
        - /rosout
        - /tf
        - /tf_static
        - /joint_states
        - /odom
        - /elevator_accel \
        - /elevator_accel_filtered \
        - /elevator_altitude \
        - /elevator_state_publisher/current_floor \
        - /elevator_state_publisher/elevator_movement \
        - /elevator_state_publisher/rest_elevator \
        - /m5stack_core2_driver/imu \
        - /m5stack_core2_driver/pressure \
        - /m5stack_core2_driver/temperature
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: sample_elevator_result.yaml
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/sample_elevator_result.yaml
        - /tmp/sample_elevator_panorama.avi
        - /tmp/sample_elevator_rosbag.bag
      upload_file_titles:
        - sample_elevator_result.yaml
        - sample_elevator_panorama.avi
        - sample_elevator_rosbag.bag
      upload_parents_path: spot_sample_elevator
      upload_server_name: /gdrive_server
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Spot Sample Elevator demo
      use_timestamp_title: true
      sender_address: belka@jsk.imi.i.u-tokyo.ac.jp
      receiver_address: spot@jsk.imi.i.u-tokyo.ac.jp
plugin_order:
  start_plugin_order:
    - panorama_video_recorder_plugin
    - rosbag_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - panorama_video_recorder_plugin
    - rosbag_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
