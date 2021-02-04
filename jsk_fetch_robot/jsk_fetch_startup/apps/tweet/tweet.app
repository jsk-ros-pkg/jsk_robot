display: Fetch tweet
platform: fetch
launch: jsk_fetch_startup/tweet.xml
interface: jsk_fetch_startup/tweet.interface
icon: jsk_fetch_startup/tweet.png
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
