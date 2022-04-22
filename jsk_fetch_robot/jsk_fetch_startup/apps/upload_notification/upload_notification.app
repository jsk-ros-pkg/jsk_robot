display: Upload notification
platform: fetch
launch: jsk_fetch_startup/upload_notification.xml
interface: jsk_fetch_startup/upload_notification.interface
icon: jsk_fetch_startup/upload_notification.png
timeout: 120
plugins:
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/app_notification.json
      upload_file_titles:
        - app_notification.json
      upload_parents_path: fetch_upload_notification
      upload_server_name: /gdrive_server
plugin_order:
  start_plugin_order:
    - gdrive_uploader_plugin
  stop_plugin_order:
    - gdrive_uploader_plugin
