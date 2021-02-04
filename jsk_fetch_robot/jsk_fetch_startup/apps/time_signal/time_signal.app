display: Speak time signal
platform: fetch
launch: jsk_fetch_startup/time_signal.xml
interface: jsk_fetch_startup/time_signal.interface
icon: jsk_fetch_startup/time_signal.png
plugins:
  - name: user_speech_notifier_plugin
    type: app_notifier/user_speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
plugin_order:
  start_plugin_order:
    - user_speech_notifier_plugin
  stop_plugin_order:
    - user_speech_notifier_plugin
