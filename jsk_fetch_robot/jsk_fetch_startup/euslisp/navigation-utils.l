(load "package://jsk_robot_startup/lifelog/tweet_client.l")
(load "package://jsk_fetch_startup/euslisp/notify-app.l")
(load "package://switchbot_ros/scripts/switchbot.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(ros::load-ros-manifest "fetch_auto_dock_msgs")
(ros::load-ros-manifest "jsk_robot_startup")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "power_msgs")

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *spots* nil)
(defparameter *tfl* (instance ros::transform-listener :init))


(let ((robot-name (ros::get-param "/robot/name")))
  (defparameter *dock-spot*
    (cond
      ((equal robot-name "fetch15")
       "/eng2/7f/room73B2-fetch-dock-front")
      ((equal robot-name "fetch1075")
       "/eng2/7f/room73B2-fetch-dock2-front")
      (t nil))))


(defun store-params ()
  (defparameter *global-inflation-radius*
    (ros::get-param "/move_base/global_costmap/inflater/inflation_radius"))
  (defparameter *local-inflation-radius*
    (ros::get-param "/move_base/local_costmap/inflater/inflation_radius"))
  t)


(defun restore-params ()
  (if (boundp '*global-inflation-radius*)
    (ros::set-dynamic-reconfigure-param
      "/move_base/global_costmap/inflater" "inflation_radius"
      :double *global-inflation-radius*))
  (if (boundp '*local-inflation-radius*)
    (ros::set-dynamic-reconfigure-param
      "/move_base/local_costmap/inflater" "inflation_radius"
      :double *local-inflation-radius*))
  t)


(defun inflation-loose (&key (local-costmap-inflation-radius 0.35)
                             (global-costmap-inflation-radius 0.7))
  ;; Increase the global inflation to generate a path
  ;; that is less sensitive to differences between the map shape and the actual object placement.
  (ros::set-dynamic-reconfigure-param
    "/move_base/global_costmap/inflater" "inflation_radius" :double global-costmap-inflation-radius)
  ;; Decrease the local inflation to generate a path
  ;; that allows the robot to pass close to the object.
  (ros::set-dynamic-reconfigure-param
    "/move_base/local_costmap/inflater" "inflation_radius" :double local-costmap-inflation-radius)
  t)


(defun get-spot-coords (name)
  (unless *spots*
    (setq *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray)))
  (let ((spot-coords nil) (frame-id nil))
    (dolist (x (send *spots* :markers))
      (if (equal (send x :text) name)
        (progn
          (setq spot-coords (send x :pose))
          (setq frame-id (send (send x :header) :frame_id)))))
    (send (send spot-coords :position) :z 0)
    (setq spot-coords (ros::tf-pose->coords spot-coords))
    (cons spot-coords frame-id)))


(defun simple-dock (&key (use-pose t))
  (unless *dock-action*
    (setq *dock-action*
          (instance ros::simple-action-client :init
                    "/dock" fetch_auto_dock_msgs::DockAction)))
  (unless (send *dock-action* :wait-for-server 5)
    (ros::ros-error "/dock action server is not started")
    (return-from dock nil))
  (let ((dock-action-goal (instance fetch_auto_dock_msgs::DockActionGoal :init)))
    (when use-pose
      (let* ((timestamp (ros::time-now))
             (cret (get-spot-coords *dock-spot*))
             (frame-to-dock (car cret))
             (frame-id (cdr cret))
             (lret (send *tfl* :wait-for-transform "base_link" frame-id timestamp 5))
             (base-to-frame (send *tfl* :lookup-transform "base_link" frame-id timestamp))
             (goal-pose (ros::coords->tf-pose (send frame-to-dock :transform base-to-frame :world)))
             (pose-msg (instance geometry_msgs::PoseStamped :init)))
        (send pose-msg :header :stamp timestamp)
        (send pose-msg :header :frame_id "base_link")
        (send pose-msg :pose goal-pose)
        (send dock-action-goal :goal :dock_pose pose-msg)))
    (send *dock-action* :send-goal dock-action-goal)
    (unless (send *dock-action* :wait-for-result :timeout 60)
      (send *dock-action* :cancel-all-goals)
      (ros::ros-error "No result returned from /dock action server")
      (return-from simple-dock nil))
    (send (send *dock-action* :get-result) :docked)))


(defun dock ()
  ;; look down
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :look-at (float-vector 800 0 500))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (simple-dock))


(defun undock ()
  (unless *undock-action*
    (setq *undock-action*
          (instance ros::simple-action-client :init
                    "/undock" fetch_auto_dock_msgs::UndockAction)))
  (unless (send *undock-action* :wait-for-server 5)
    (ros::ros-error "/undock action server is not started")
    (return-from undock nil))
  (send *undock-action* :send-goal
        (instance fetch_auto_dock_msgs::UndockActionGoal :init))
  (unless (send *undock-action* :wait-for-result :timeout 60)
    (ros::ros-error "No result returned from /undock action server")
    (return-from undock nil))
  (send (send *undock-action* :get-result) :undocked))


(defun get-battery-charging-state (&key (timeout 1500))
   (let* ((msg (one-shot-subscribe "/battery_state" power_msgs::batterystate :timeout timeout))
          (is-charging (if msg (send msg :is_charging))))
     ;; You may fail to subscribe /battery_state
     ;; because of message md5 difference between melodic and indigo.
     (if (not msg) (return-from get-battery-charging-state nil))
     (if is-charging :charging :discharging)))


(defun go-to-spot (name &key (relative-pos nil) (relative-rot nil) (undock-rotate nil) (clear-costmap t))
  "Move the robot to the spot defined in /spots_marker_array topic. The reason for using relative-pos and relative-rot instead of relative-coords is that it is easier to understand if relative-pos and relative-rot are specified in the different coords (specifically, world (map) and local (spot) coords respectively). For detail, see https://github.com/jsk-ros-pkg/jsk_robot/pull/1458#pullrequestreview-1039654868
Args:
- name : The name of the spot defined in /spots_marker_array topic
- relative-pos : The robot moves to a position that is distant from the spot by relative-pos[mm]. relative-pos is specified in the world (map) coords.
- relative-rot : The robot moves to a position rotated by relative-pos[deg] from the spot. relative-rot is specified in the local (spot) coords.
- undock-rotate : If t, the robot rotates 180 degrees after undock before moving to the spot.
- clear-costmap : If t, clear costmap for obstacle avoidance before the robot moves.
"

  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (if clear-costmap (send *ri* :clear-costmap))
  (let ((undock-success nil))
    (block go-to-spot-undock
      (dotimes (i 3)
        (let* ((battery-charging-state (get-battery-charging-state)))
          ;; You may fail to subscribe /battery_state
          ;; because of message md5 difference between melodic and indigo.
          (unless battery-charging-state
            (ros::ros-warn "Failed to subscribe /battery_state")
            (ros::ros-warn "Skip undocking, so please make sure that Fetch is already undocked.")
            (return-from go-to-spot-undock t))
          (if (equal battery-charging-state :charging)
            (progn
              (setq undock-success (auto-undock :n-trial 3))
              ;; rotate after undock
              (if (and undock-success undock-rotate)
                (send *ri* :go-pos-unsafe 0 0 180)))
            (return-from go-to-spot-undock t))
          (if (not undock-success)
            (progn
              (ros::ros-error "Failed to undock")
            (send *ri* :speak "I failed to undock.")))))
      (if (not undock-success) (return-from go-to-spot nil))))
  ;; go to spot
  (let* ((ret (get-spot-coords name))
         (goal-pose (car ret))
         (frame-id (cdr ret)))
    (when relative-pos
      (setq goal-pose (send goal-pose :translate relative-pos :world)))
    (when relative-rot
      (setq goal-pose (send goal-pose :rotate relative-rot :z :local)))
    (send *ri* :move-to goal-pose :frame-id frame-id)))


(defun auto-dock (&key (n-trial 1) (clear-costmap t))
  (let ((success nil))
    (dotimes (i n-trial)
      (when (go-to-spot *dock-spot*
                        :relative-pos #f(-800 0 0)
                        :clear-costmap clear-costmap)
        (ros::ros-info "arrived at the dock.")
        (setq success (dock))
        (when success (return-from auto-dock success))))
    success))


(defun auto-undock (&key (n-trial 1))
  (let ((success nil))
    (unless (boundp '*ri*)
      (require :fetch-interface "package://fetcheus/fetch-interface.l")
      (fetch-init))
    (dotimes (i n-trial)
      (ros::ros-info "trying to do undock.")
      (setq success (undock))
      (when success (return-from auto-undock success)))
    (if (not success)
      (let ((enable-request (instance power_msgs::BreakerCommandRequest :init :enable t))
            (disable-request (instance power_msgs::BreakerCommandRequest :init :enable nil))
            (breaker-service-name "base_breaker"))
        (ros::wait-for-service breaker-service-name 5)
        (ros::service-call breaker-service-name disable-request)
        (ros::ros-error "Diable base breaker")
        (send *ri* :speak "I disable and enable base breaker.")
        (unix::sleep 5)
        (ros::service-call breaker-service-name enable-request)
        (ros::ros-error "Enable base breaker")))
    success))


(defun get-light-on ()
  (let* ((room-light-msg
           (one-shot-subscribe "/check_room_light/output" jsk_robot_startup::RoomLight))
         (light-on (if room-light-msg (send room-light-msg :light_on))))
    light-on))


(defun room-light-on (&key (control-switchbot :api))
  (cond
   ((or (eq control-switchbot :en) (eq control-switchbot :jp))
    ;; Look at Google Home
    (send *fetch* :head :neck-y :joint-angle 89)
    (send *fetch* :head :neck-p :joint-angle 25)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
    (send *ri* :wait-interpolation)
    ;; The accuracy of Google Home recognition is better in English than in Japanese.
    (if (eq control-switchbot :en)
      (progn
        (send *ri* :speak "OK, Google" :wait t)
        (send *ri* :speak "Turn on the light." :wait t))
      (progn
        (send *ri* :speak-jp "オッケー、グーグル" :wait t)
        (send *ri* :speak-jp "電気をつけて" :wait t)))
    (send *fetch* :head :neck-y :joint-angle 0)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
    (send *ri* :wait-interpolation))
   ((eq control-switchbot :api)
    (control-device "/eng2/7f/73b2_bot_kitchen" "turnOn" :wait t)
    (control-device "/eng2/7f/73b2_bot_window" "turnOn" :wait t)
    (send *ri* :speak-jp "電気をつけました" :wait t))))


(defun room-light-off (&key (control-switchbot :api))
  (cond
   ((or (eq control-switchbot :en) (eq control-switchbot :jp))
    ;; Look at Google Home
    (send *fetch* :head :neck-y :joint-angle 89)
    (send *fetch* :head :neck-p :joint-angle 25)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
    (send *ri* :wait-interpolation)
    ;; The accuracy of Google Home recognition is better in English than in Japanese.
    (if (eq control-switchbot :en)
      (progn
        (send *ri* :speak "OK, Google" :wait t)
        (send *ri* :speak "Turn off the light." :wait t))
      (progn
        (send *ri* :speak-jp "オッケー、グーグル" :wait t)
        (send *ri* :speak-jp "電気を消して" :wait t)))
    (send *fetch* :head :neck-y :joint-angle 0)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
    (send *ri* :wait-interpolation))
   ((eq control-switchbot :api)
    (control-device "/eng2/7f/73b2_bot_kitchen" "turnOff" :wait t)
    (control-device "/eng2/7f/73b2_bot_window" "turnOff" :wait t)
    (send *ri* :speak-jp "電気を消しました" :wait t))))


(defun report-auto-dock ()
  (ros::ros-info "start going back to the dock.")
  (send *ri* :speak-jp "ドックに戻ります。" :wait t))


(defun report-move-to-sink-front-success ()
  (ros::ros-info "arrived at the kitchen stove.")
  (send *ri* :speak-jp "キッチンのコンロの前につきました。" :wait t))


(defun report-move-to-sink-front-failure ()
  (ros::ros-error "failed going to the kitchen.")
  (send *ri* :speak-jp "キッチンのコンロの前に行くのに失敗しました。" :wait t))

(defun report-move-to-trashcan-front ()
  (ros::ros-info "arrived at the trash can")
  (send *ri* :speak-jp "ゴミ箱の前につきました。" :wait t))

(defun report-move-to-trashcan-front-failure ()
  (ros::ros-error "failed going to the trash can front")
  (send *ri* :speak-jp "ゴミ箱の前に行くのに失敗しました。" :wait t))

(defun report-start-go-to-kitchen ()
  (ros::ros-info "start going to the kitchen.")
  (send *ri* :speak-jp "キッチンに向かいます。" :wait t))


(defun report-light-on ()
  (ros::ros-info "room light is already on.")
  (send *ri* :speak-jp "すでに電気がついています。" :wait t))


(defun report-light-off ()
  (ros::ros-info "room light is off.")
  (send *ri* :speak-jp "電気が消えています。" :wait t))

(defun notify-recognition (&key (location "kitchen"))
  (let* ((msg (one-shot-subscribe "/edgetpu_object_detector/output/class"
                                  jsk_recognition_msgs::ClassificationResult))
         (label-names (send msg :label_names)))
    (when label-names
      (ros::ros-info (format nil "Notify app that ~A is found." label-names))
      (notify-app "object recognition"
                  (send msg :header :stamp)
                  location
                  (format nil "~A is found" label-names)))))

(defun inspect-kitchen (&key (tweet t))
  (report-move-to-sink-front-success)
  (if tweet
    (progn
      ;; stove
      (tweet-string "I took a photo at 73B2 Kitchen stove." :warning-time 3
                    :with-image "/edgetpu_object_detector/output/image" :speak t)
      (notify-recognition :location "kitchen stove")
      (send *ri* :go-pos-unsafe 0 0 -45)
      ;; sink
      (tweet-string "I took a photo at 73B2 Kitchen sink." :warning-time 3
                    :with-image "/edgetpu_object_detector/output/image" :speak t)
      (notify-recognition :location "kitchen sink"))
    (progn
      (notify-recognition :location "kitchen"))))


(defun move-to-sink-front (&key (n-trial 1) (offset #f(400 -500 0)))
  (let ((success-move-to-sink-front nil))
    (dotimes (i n-trial)
      (setq success-move-to-sink-front
            (go-to-spot "/eng2/7f/room73B2-sink-front0"
                        :relative-pos offset :undock-rotate t))
      (when success-move-to-sink-front (return)))
    success-move-to-sink-front))

(defun inspect-trashcan (&key (tweet t))
  (report-move-to-trashcan-front)
  (send *fetch* :head :neck-p :joint-angle 40)
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
  (send *ri* :wait-interpolation)
  (if tweet
      (progn
        (tweet-string "I took a photo of 73B2 trash cans." :warning-time 3
                      :with-image "/edgetpu_object_detector/output/image" :speak t)
        (notify-recognition :location "trash cans")
        (send *ri* :go-pos-unsafe 0.2 0 0)
        (send *fetch* :head :neck-p :joint-angle 50)
        (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
        (send *ri* :wait-interpolation)
        (tweet-string "I took a photo of 73B2 trash can inside." :warning-time 3
                      :with-image "/edgetpu_object_detector/output/image" :speak t)
        (notify-recognition :location "trash cans inside")
        (send *ri* :go-pos-unsafe -0.2 0 0))
    (progn
      (notify-recognition :location "trash cans"))))

(defun move-to-trashcan-front (&key (n-trial 1) (offset #f(400 -500 0)))
  (let ((success-move-to-trashcan-front nil))
    (dotimes (i n-trial)
      (setq success-move-to-trashcan-front
            (go-to-spot "/eng2/7f/room73B2-sink-front0"
                        :relative-pos offset :relative-rot -pi/2))
      (when success-move-to-trashcan-front (return)))
    (send *ri* :go-pos-unsafe 1.3 0 0)
    (send *ri* :go-pos-unsafe 0 0 -80) ;; face the front against the trash can
    success-move-to-trashcan-front))


(defun go-to-kitchen (&key (tweet t) (n-dock-trial 1) (n-kitchen-trial 1)
                           (control-switchbot :api))
  ;; go to kitchen
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (report-start-go-to-kitchen)
  ;; Check if the lights are on in the room
  (let ((initial-light-on (get-light-on))
        (success-go-to-kitchen nil)
        (success-go-to-trashcan nil)
        (success-auto-dock nil)
        (success-battery-charging nil))
    (store-params)
    ;; turn on light
    (if initial-light-on
      (report-light-on)
      (progn
        (report-light-off)
        (room-light-on :control-switchbot control-switchbot)))
    ;; change the inflation_radius
    (inflation-loose)
    ;; go to kitchen sink
    (setq success-go-to-kitchen (move-to-sink-front :n-trial n-kitchen-trial))
    ;; report result to go to kitchen
    (if success-go-to-kitchen
      (inspect-kitchen :tweet tweet)
      (report-move-to-sink-front-failure))
    ;; go to kitchen trash can
    (setq success-go-to-trashcan (move-to-trashcan-front :n-trial n-trashcan-trial))
    ;; report result to go to trash can
    (if success-go-to-trashcan
      (inspect-trashcan :tweet tweet)
      (report-move-to-trashcan-front-failure))
    ;; go back from dock
    (report-auto-dock)
    (setq success-auto-dock (auto-dock :n-trial n-dock-trial :clear-costmap nil))
    ;; turn off light
    (if (and success-auto-dock (not initial-light-on))
      (room-light-off :control-switchbot control-switchbot))
    ;; change the inflation_radius
    (restore-params)
    (setq success-battery-charging (equal (get-battery-charging-state) :charging))
    (and success-go-to-kitchen success-auto-dock success-battery-charging)))


(defun go-to-kitchen-state-machine ()
  (setq *sm*
        (make-state-machine
          '((:init -> :report-start-go-to-kitchen)
            (:report-start-go-to-kitchen -> :get-light-on)
            (:get-light-on -> :report-light-on)
            (:get-light-on !-> :room-light-on)
            (:report-light-on -> :move-to-sink-front)
            (:room-light-on -> :move-to-sink-front)
            (:move-to-sink-front -> :inspect-kitchen)
            (:move-to-sink-front !-> :report-move-to-sink-front-failure)
            ;;(:inspect-kitchen -> :auto-dock)
            ;;(:report-move-to-sink-front-failure -> :auto-dock)
            (:inspect-kitchen -> :move-to-trashcan-front)
            (:move-to-trashcan-front -> :inspect-trashcan)
            (:move-to-trashcan-front !-> :report-move-to-trashcan-front-failure)
            (:inspect-trashcan -> :auto-dock)
            (:report-move-to-trashcan-front-failure -> :auto-dock)
            (:auto-dock -> :room-light-off)
            (:room-light-off -> :finish)
            (:finish -> t)
            (:finish !-> nil))
          '((:init
              '(lambda (userdata)
                 (fetch-init)
                 (send *ri* :clear-costmap)
                 (store-params)
                 (inflation-loose)
                 (clear-app-notification)
                 t))
            (:report-start-go-to-kitchen
              '(lambda (userdata)
                 (report-start-go-to-kitchen)
                 t))
            (:get-light-on
              '(lambda (userdata)
                 (let ((light-on (get-light-on)))
                   (setf (cdr (assoc 'initial-light-on userdata)) light-on)
                   light-on)))
            (:report-light-on
              '(lambda (userdata)
                 (report-light-on)
                 t))
            (:room-light-on
              '(lambda (userdata)
                 (let ((control-switchbot (cdr (assoc 'control-switchbot userdata))))
                   (report-light-off)
                   (room-light-on :control-switchbot control-switchbot)
                   t)))
            (:move-to-sink-front
              '(lambda (userdata)
                 (let* ((n-trial (cdr (assoc 'n-kitchen-trial userdata)))
                        (success (move-to-sink-front :n-trial n-trial)))
                   (setf (cdr (assoc 'success-go-to-kitchen userdata)) success)
                   success)))
            (:inspect-kitchen
              '(lambda (userdata)
                 (inspect-kitchen :tweet (cdr (assoc 'tweet userdata)))
                 t))
            (:report-move-to-sink-front-failure
              '(lambda (userdata)
                 (report-move-to-sink-front-failure)
                 t))
            (:move-to-trashcan-front
              '(lambda (userdata)
                 (let* ((n-trial (cdr (assoc 'n-trashcan-trial userdata)))
                        (success (move-to-trashcan-front :n-trial n-trial)))
                   (setf (cdr (assoc 'success-go-to-trashcan userdata)) success)
                   success)))
            (:inspect-trashcan
              '(lambda (userdata)
                 (inspect-trashcan :tweet (cdr (assoc 'tweet userdata)))
                 t))
            (:report-move-to-trashcan-front-failure
              '(lambda (userdata)
                 (report-move-to-trashcan-front-failure)
                 t))
            (:auto-dock
              '(lambda (userdata)
                 (report-auto-dock)
                 (let* ((n-trial (cdr (assoc 'n-dock-trial userdata)))
                        (success (auto-dock :n-trial n-trial :clear-costmap nil)))
                   (setf (cdr (assoc 'success-auto-dock userdata)) success)
                   success)))
            (:room-light-off
              '(lambda (userdata)
                 (let ((success-auto-dock (cdr (assoc 'success-auto-dock userdata)))
                       (initial-light-on (cdr (assoc 'initial-light-on userdata)))
                       (control-switchbot (cdr (assoc 'control-switchbot userdata))))
                   (if (and success-auto-dock (not initial-light-on))
                     (room-light-off :control-switchbot control-switchbot)))
                 t))
            (:finish
              '(lambda (userdata)
                 (let ((success-battery-charging
                         (equal (get-battery-charging-state) :charging))
                       (success-auto-dock (cdr (assoc 'success-auto-dock userdata)))
                       (success-go-to-kitchen
                         (cdr (assoc 'success-go-to-kitchen userdata))))
                   (restore-params)
                   (and success-go-to-kitchen success-auto-dock success-battery-charging)))))
          '(:init)
          '(t nil))))
