display: Speak time signal
platform: fetch
run: jsk_fetch_startup/time_signal.py
run_name: "time_signal"
# run_name needs https://github.com/PR2/app_manager/pull/64
interface: jsk_fetch_startup/time_signal.interface
icon: jsk_fetch_startup/time_signal.png
timeout: 120
plugins:
  - name: tweet_notifier_plugin
    type: app_notifier/tweet_notifier_plugin
    plugin_args:
      client_name: /tweet_image_server/tweet
      image: true
      image_topic_name: /edgetpu_object_detector/output/image
      warning: false
  - name: user_speech_notifier_plugin
    type: app_notifier/user_speech_notifier_plugin
    plugin_args:
      client_name: /sound_play
      warning: false
plugin_order:
  start_plugin_order:
    - tweet_notifier_plugin
    - user_speech_notifier_plugin
  stop_plugin_order:
    - tweet_notifier_plugin
    - user_speech_notifier_plugin
