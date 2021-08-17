display: Plug Spot Power Connector
platform: fetch
launch: jsk_fetch_startup/plug_spot_power_connector.xml
interface: jsk_fetch_startup/plug_spot_power_connector.interface
plugins:
  - name: service_notification_saver_plugin
    type: app_notification_saver/service_notification_saver
  - name: head_camera_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: plug_spot_power_connector_head_camera.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /tag_detections_image
      video_height: 1080
      video_width: 1920
      video_framerate: 30
      video_encoding: RGB
  - name: panorama_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: plug_spot_power_connector_panorama.avi
      video_topic_name: /dual_fisheye_to_panorama/output
      video_fps: 1.0
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: plug_spot_power_connector_result.yaml
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/plug_spot_power_connector_result.yaml
        - /tmp/plug_spot_power_connector_head_camera.avi
        - /tmp/plug_spot_power_connector_panorama.avi
      upload_file_titles:
        - plug_spot_power_connector_result.yaml
        - plug_spot_power_connector_head_camera.avi
        - plug_spot_power_connector_panorama.avi
      upload_parents_path: fetch_plug_spot_power_connector
      upload_server_name: /gdrive_server
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Fetch Plug Spot Power Connector Demo
      use_timestamp_title: true
    plugin_arg_yaml: /var/lib/robot/fetch_mail_notifier_plugin.yaml
plugin_order:
  start_plugin_order:
    - service_notification_saver_plugin
    - head_camera_video_recorder_plugin
    - panorama_video_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - service_notification_saver_plugin
    - head_camera_video_recorder_plugin
    - panorama_video_recorder_plugin
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
