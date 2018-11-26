(require :unittest "lib/llib/unittest.l")
(load "package://peppereus/pepper-interface.l")

(init-unit-test)

(deftest instantiate-pepper
  (setq *pepper* (pepper)))

(deftest instantiate-pepper-interface
  (setq *ri* (instance pepper-interface :init)))

;; naoqi_bridge_msgs
(deftest test-show-image
  (assert  (instance naoqi_bridge_msgs::SetStringRequest :init) "test-show-image"))

(deftest test-set-show-image-folder-path
  (assert  (instance naoqi_bridge_msgs::SetStringRequest :init) "test-set-show-image-folder-path"))

(deftest test-get-show-image-folder-path
  (assert  (instance naoqi_bridge_msgs::GetStringRequest :init) "test-get-show-image-folder-path"))

(deftest test-show-app
  (assert  (instance naoqi_bridge_msgs::SetStringRequest :init) "test-show-app"))

(deftest test-show-webview
  (assert  (instance naoqi_bridge_msgs::SetStringRequest :init) "test-show-webview"))

;; std_srvs
(deftest test-hide-image
  (assert  (instance std_srvs::EmptyRequest :init) "test-hide-image"))

(run-all-tests)
(exit)

