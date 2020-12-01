display: Speak battery
platform: fetch
launch: jsk_fetch_startup/speak_battery.xml
interface: jsk_fetch_startup/speak_battery.interface
icon: jsk_fetch_startup/speak_battery.png
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
