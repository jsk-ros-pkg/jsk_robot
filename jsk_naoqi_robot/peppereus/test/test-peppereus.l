(require :unittest "lib/llib/unittest.l")
(load "package://peppereus/pepper-interface.l")

(init-unit-test)

(deftest instantiate-pepper
  (setq *pepper* (pepper)))

(deftest instantiate-pepper-interface
  (setq *ri* (instance pepper-interface :init)))

(run-all-tests)
(exit)

