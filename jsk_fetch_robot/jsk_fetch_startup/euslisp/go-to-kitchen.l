#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(defun main (&key (tweet t) (n-dock-trial 3) (n-kitchen-trial 3) (control-switchbot :api))
  (when (not (boundp '*sm*))
    (go-to-kitchen-state-machine))
  (exec-state-machine
    *sm* `((tweet . ,tweet)
           (n-kitchen-trial . ,n-kitchen-trial)
           (n-dock-trial . ,n-dock-trial)
           (control-switchbot . ,control-switchbot)
           (initial-light-on . nil)
           (success-go-to-kitchen . nil)
           (success-auto-dock . nil))
    :hz 2.0))

(ros::roseus "go_to_kitchen")
(if (main) (unix::exit 0) (unix::exit 1))
