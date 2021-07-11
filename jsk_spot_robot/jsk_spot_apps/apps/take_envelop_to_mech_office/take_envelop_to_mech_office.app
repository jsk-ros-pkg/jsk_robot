display: Take Envelop To Mech Office
platform: spot
launch: jsk_spot_apps/take_envelop_to_mech_office.xml
interface: jsk_spot_apps/take_envelop_to_mech_office.interface
plugins:
    - name: mail_notifier_plugin
      type: app_notifier/mail_notifier_plugin
      sender_address: spot-jsk@jsk.imi.i.u-tokyo.ac.jp
      receiver_address: spot@jsk.imi.i.u-tokyo.ac.jp
