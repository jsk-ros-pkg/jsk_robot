(require :unittest "lib/llib/unittest.l")
(load "package://peppereus/pepper-interface.l")

(init-unit-test)

(deftest instantiate-pepper
  (setq *pepper* (pepper)))

(deftest instantiate-pepper-interface
  (setq *ri* (instance pepper-interface :init)))

;; std_srvs
(deftest test-hide-image
  (assert  (instance std_srvs::EmptyRequest :init) "test-hide-image"))

(run-all-tests)
(exit)

