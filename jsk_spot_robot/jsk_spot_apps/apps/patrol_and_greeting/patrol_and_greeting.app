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
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/patrol_and_greeting.yaml
      upload_file_titles:
        - patrol_and_greeting.yaml
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
    - gdrive_uploader_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
