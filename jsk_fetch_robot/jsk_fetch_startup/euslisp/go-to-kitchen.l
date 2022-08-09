#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")


(defun main (&key (tweet t) (n-dock-trial 3) (n-kitchen-trial 3) (n-trashcan-trial 3) (n-dock-front-trial 3) (n-tv-front-trial 3) (n-tv-desk-trial 3) (n-desk-back-trial 3) (n-desk-front-trial 3) (n-kitchen-door-front-trial 3) (control-switchbot :api))
  (when (not (boundp '*sm*))
    (go-to-kitchen-state-machine))
  (let ((result-state
          (exec-state-machine *sm* `((tweet . ,tweet)
                                     (n-kitchen-trial . ,n-kitchen-trial)
                                     (n-trashcan-trial . ,n-trashcan-trial)
                                     (n-dock-trial . ,n-dock-trial)
                                     (n-dock-front-trial . ,n-dock-front-trial)
                                     (n-tv-front-trial . ,n-tv-front-trial)
                                     (n-tv-desk-trial . ,n-tv-desk-trial)
                                     (n-desk-back-trial . ,n-desk-back-trial)
                                     (n-desk-front-trial . ,n-desk-front-trial)
                                     (n-kitchen-door-front-trial . ,n-kitchen-door-front-trial)
                                     (control-switchbot . ,control-switchbot)
                                     (initial-light-on . nil)
                                     (success-go-to-kitchen . nil)
                                     (success-go-to-trashcan . nil)
                                     (success-auto-dock . nil))
                              :hz 2.0)))
    (send result-state :name)))


(ros::roseus "go_to_kitchen")
(if (main) (unix::exit 0) (unix::exit 1))
