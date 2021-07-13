display: Take Envelop To Mech Office
platform: spot
launch: jsk_spot_apps/take_envelop_to_mech_office.xml
interface: jsk_spot_apps/take_envelop_to_mech_office.interface
plugins:
  - name: insta_360_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: take_envelop_mech_office_insta_360_video.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /dual_fisheye_to_panorama/output/quater
      video_height: 885
      video_width: 1772
      video_framerate: 5
      video_encoding: RGB
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: take_envelop_mech_office.bag
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
        - /spot_behavior_manager_demo/current_node_id
        - /spot_kinova/joint_states
        - /spot_recognition/object_detection_image/compressed
        - /spot_recognition/elevator_door_points
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: take_envelop_mech_office.yaml
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/take_envelop_mech_office.yaml
        - /tmp/take_envelop_mech_office.bag
        - /tmp/take_envelop_mech_office_insta_360_video.avi
      upload_file_titles:
        - take_envelop_mech_office.yaml
        - take_envelop_mech_office.bag
        - take_envelop_mech_office_insta_360_video.avi
      upload_parents_path: spot_take_envelop_mech_office
      upload_server_name: /gdrive_server
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Result of Mech Office Demo
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
