(require :unittest "lib/llib/unittest.l")
(load "package://naoeus/nao-interface.l")

(init-unit-test)

(deftest instantiate-nao
  (setq *nao* (nao)))

;(deftest instantiate-nao-interface
;  (setq *ri* (instance nao-interface :init)))

(run-all-tests)
(exit)

