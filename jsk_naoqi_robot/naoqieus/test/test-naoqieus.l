(require :unittest "lib/llib/unittest.l")
(load "package://naoqieus/naoqi-interface.l")

(init-unit-test)

(load "irteus/demo/sample-robot-model.l")
(setq *robot* (instance sample-robot :init))
(deftest instantiate-naoqi-interface
  (setq *ri* (instance naoqi-interface :init :robot *robot*)))

(run-all-tests)
(exit)

