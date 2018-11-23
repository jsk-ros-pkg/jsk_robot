(require :unittest "lib/llib/unittest.l")
(load "package://naoqieus/naoqi-interface.l")

(init-unit-test)

(load "irteus/demo/sample-robot-model.l")
(setq *robot* (instance sample-robot :init))
(deftest instantiate-naoqi-interface
  (setq *ri* (instance naoqi-interface :init :robot *robot*)))

;; test functions for methods which don't require a real robot
(deftest test-speak
  (assert (send *ri* :speak "hello") "test-speak error")
  (assert (send *ri* :speak "こんにちは") "test-speak error"))

(deftest test-animated-speak
  (assert (send *ri* :animated-speak "hello") "test-animated-speak error")
  (assert (send *ri* :animated-speak "こんにちは") "test-animated-speak error"))

(deftest test-go-pos
  (assert (send *ri* :go-pos 0.5 0.5 30) "test-go-pos error"))

(deftest test-go-velocity
  (assert (send *ri* :go-velocity 0.5 0.5 0.1 1000) "test-go-velocity error")
  (assert (not (send *ri* :go-velocity 1.1 0.5 0.1 1000)) "test-go-velocity error")
  (assert (not (send *ri* :go-velocity -1.1 0.5 0.1 1000)) "test-go-velocity error")
  (assert (not (send *ri* :go-velocity 0.5 1.1 0.1 1000)) "test-go-velocity error")
  (assert (not (send *ri* :go-velocity 0.5 -1.1 0.1 1000)) "test-go-velocity error")
  (assert (not (send *ri* :go-velocity 0.5 0.5 1.1 1000)) "test-go-velocity error")
  (assert (not (send *ri* :go-velocity 0.5 0.5 -1.1 1000)) "test-go-velocity error")
  (assert (send *ri* :go-velocity 0.5 0.5 0.1 1000 :stop nil) "test-go-velocity error"))

(deftest test-go-stop
  (assert (send *ri* :go-stop) "test-go-stop error"))

;; test functions for methods which require a real robot
;; naoqi_bridge_msgs

;; :set-external-collision-protection-status
;; https://github.com/ros-naoqi/naoqi_bridge_msgs/pull/19 was not merged, but is included in https://github.com/kochigami/naoqi_bridge_msgs/tree/kochigami-develop/

;; :get-external-collision-protection-status
;; https://github.com/ros-naoqi/naoqi_bridge_msgs/pull/19 was not merged, but is included in https://github.com/kochigami/naoqi_bridge_msgs/tree/kochigami-develop/

(deftest test-set-move-arms-enabled
  (assert (instance naoqi_bridge_msgs::SetArmsEnabledRequest :init) "test-set-move-arms-enabled"))

;; :get-move-arms-enabled
;; https://github.com/ros-naoqi/naoqi_bridge_msgs/pull/20 was not merged.
;; Please use https://github.com/kochigami/naoqi_bridge_msgs/tree/kochigami-develop

(deftest test-set-language
  (assert (instance naoqi_bridge_msgs::SetStringRequest :init) "test-set-string"))

(deftest test-get-language
  (assert (instance naoqi_bridge_msgs::GetStringRequest :init) "test-get-string"))

;; nao_interaction_msgs
(deftest test-play-audio-file
  (assert (instance nao_interaction_msgs::AudioPlaybackRequest :init) "test-play-audio-file"))

;; :set-master-volume, :get-master-volume
;; not released yet: https://github.com/ros-naoqi/nao_interaction/pull/10

;; std_srvs
(deftest test-servo-on
  (assert  (instance std_srvs::EmptyRequest :init) "test-servo-on"))

(deftest test-servo-off
  (assert  (instance std_srvs::EmptyRequest :init) "test-servo-off"))

(deftest test-servo-off
  (assert  (instance std_srvs::EmptyRequest :init) "test-servo-off"))

(deftest test-enable-life
  (assert  (instance std_srvs::EmptyRequest :init) "test-enable-life"))

(deftest test-disable-life
  (assert  (instance std_srvs::EmptyRequest :init) "test-disable-life"))

(deftest test-get-life
  (assert  (instance std_srvs::TriggerRequest :init) "test-get-life"))

(deftest test-set-background-movement-enabled
  (assert  (instance std_srvs::SetBoolRequest :init) "test-set-background-movement-enabled"))

(deftest test-get-background-movement-enabled
  (assert  (instance std_srvs::TriggerRequest :init) "test-get-background-movement-enabled"))

(run-all-tests)
(exit)

