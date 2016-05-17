#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "package://fetcheus/fetch-interface.l")

(init-unit-test)

(deftest instantiate-fetch
  (setq *fetch* (fetch)))

(deftest instantiate-fetch-interface
  (setq *ri* (instance fetch-interface :init)))

(run-all-tests)
(exit)

