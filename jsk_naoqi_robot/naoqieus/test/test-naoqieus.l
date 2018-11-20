(require :unittest "lib/llib/unittest.l")
(load "package://naoqieus/naoqi-interface.l")

(init-unit-test)

(load "irteus/demo/sample-robot-model.l")
(setq *robot* (instance sample-robot :init))
(deftest instantiate-naoqi-interface
  (setq *ri* (instance naoqi-interface :init :robot *robot*)))

(deftest test-speak
  (assert (send *ri* :speak "hello") "test-speak error")
  (assert (send *ri* :speak "こんにちは") "test-speak error"))

(deftest test-animated-speak
  (assert (send *ri* :animated-speak "hello") "test-animated-speak error")
  (assert (send *ri* :animated-speak "こんにちは") "test-speak error"))

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

(run-all-tests)
(exit)

