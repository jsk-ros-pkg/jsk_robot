display: Take Envelop To Mech Office
platform: spot
launch: jsk_spot_apps/take_envelop_to_mech_office.xml
interface: jsk_spot_apps/take_envelop_to_mech_office.interface
plugins:
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
      upload_file_titles:
        - take_envelop_mech_office.yaml
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
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - result_recorder_plugin
    - gdrive_uploader_plugin
    - mail_notifier_plugin
