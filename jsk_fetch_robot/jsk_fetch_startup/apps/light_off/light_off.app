display: Fetch light off
platform: fetch
launch: jsk_fetch_startup/light_off.xml
interface: jsk_fetch_startup/light_off.interface
icon: jsk_fetch_startup/light_off.png
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
