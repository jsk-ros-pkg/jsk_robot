display: Fetch light on
platform: fetch
launch: jsk_fetch_startup/light_on.xml
interface: jsk_fetch_startup/light_on.interface
icon: jsk_fetch_startup/light_on.png
plugins:
  - name: user_speech_notifier_plugin
    type: app_notifier/user_speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
      warning: true
plugin_order:
  start_plugin_order:
    - user_speech_notifier_plugin
  stop_plugin_order:
    - user_speech_notifier_plugin
