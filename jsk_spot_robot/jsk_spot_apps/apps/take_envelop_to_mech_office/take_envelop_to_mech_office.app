display: Take Envelop To Mech Office
platform: spot
launch: jsk_spot_apps/take_envelop_to_mech_office.xml
interface: jsk_spot_apps/take_envelop_to_mech_office.interface
plugins:
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Result of Mech Office Demo
      sender_address: spot-jsk@jsk.imi.i.u-tokyo.ac.jp
      receiver_address: spot@jsk.imi.i.u-tokyo.ac.jp
      use_timestamp_title: true
plugin_order:
  start_plugin_order:
    - mail_notifier_plugin
  stop_plugin_order:
    - mail_notifier_plugin
