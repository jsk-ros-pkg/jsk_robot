display: Patrol And Greeting
platform: spot
launch: jsk_spot_apps/patrol_and_greeting.xml
interface: jsk_spot_apps/patrol_and_greeting.interface
plugins:
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: patrol_and_greeting.yaml
  - name: audio_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: patrol_and_greeting_audio_video.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /dual_fisheye_to_panorama/output/throttled
      video_height: 921
      video_width: 1846
      video_framerate: 10
      video_encoding: RGB
  - name: video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: patrol_and_greeting_video.avi
      video_topic_name: /dual_fisheye_to_panorama/output/throttled
      video_fps: 10
  - name: audio_recorder_plugin
    type: app_recorder/audio_recorder_plugin
    launch_args:
      audio_path: /tmp
      audio_title: patrol_and_greeting_audio.wav
      audio_topic_name: /audio
      audio_format: wave
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/patrol_and_greeting.yaml
        - /tmp/patrol_and_greeting.avi
      upload_file_titles:
        - patrol_and_greeting.yaml
        - patrol_and_greeting.avi
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
    - result_recorder_plugin
    - audio_video_recorder_plugin
    - video_recorder_plugin
    - audio_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - result_recorder_plugin
    - audio_video_recorder_plugin
    - video_recorder_plugin
    - audio_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
