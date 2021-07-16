#!/usr/bin/env roseus

(ros::load-ros-manifest "power_msgs")

(require :speak "package://pr2eus/speak.l")


(defun speak-battery ()
  (setq msg (one-shot-subscribe "/battery_state" power_msgs::BatteryState :timeout (* 10 1000)))
  (setq sentence
    (if msg
      (format nil
              (if (send msg :is_charging)
                  "バッテリー残り~Aパーセントです。ただいま充電中です。"
                  "バッテリー残り~Aパーセントです。充電中ではありません。")
              (floor (* 100 (send msg :charge_level))))
      "バッテリー情報が得られませんでした。"))
  (speak-jp sentence :wait t))

(ros::roseus "speak_battery")
(speak-battery)
