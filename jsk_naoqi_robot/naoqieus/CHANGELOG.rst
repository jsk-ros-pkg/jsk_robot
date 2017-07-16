^^^^^^^^^^^^^^^^^^^^^^
Changelog for naoqieus
^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-07-16)
------------------

1.0.9 (2016-11-09)
------------------

1.0.8 (2016-11-08)
------------------

1.0.7 (2016-11-02)
------------------
* Merge pull request `#673 <https://github.com/jsk-ros-pkg/jsk_robot/issues/673>`_ from kochigami/add-grasp-method-to-gazebo
  [naoqieus] add grasp method in gazebo
* [naoqieus, peppereus] delete wheel controller
* delete dcm hand controllers
* modify move-hand to use in gazebo
* Merge pull request `#665 <https://github.com/jsk-ros-pkg/jsk_robot/issues/665>`_ from kochigami/add-set-get-move-arms-enabled
  [naoqieus] add set (get) move arms enabled
* Merge pull request `#655 <https://github.com/jsk-ros-pkg/jsk_robot/issues/655>`_ from kochigami/override-angle-vector
  [peppereus] add gazebo controller method
* [naoqieus] add set (get) move arms enabled
* add controllers
* [naoqieus] use let for local variables in set(get)-external-collision-protection-status
* add gazebo controller method and override angle-vector
* add get-external-collision-protection-status method
* Contributors: Kanae Kochigami, Kei Okada

1.0.6 (2016-06-17)
------------------

1.0.5 (2016-04-18)
------------------
* fixed program when the order of two optional params is upside down
* move send-stiffness-controller, servo-on, servo-off methods to naoqi-interface
* Contributors: Kanae Kochigami

1.0.4 (2016-03-21)
------------------
* [naoqi-interface.l] add angle-ratio parameter to :start-grasp, :stop-grasp
* Contributors: Kanae Kochigami

1.0.3 (2016-03-05)
------------------
* modified :get-life method
* :enable-life, :disable-life, :get-life are added
* modified indent
* add animated-speak method (`#557 <https://github.com/jsk-ros-pkg/jsk_robot/issues/557>`_ )
* naoqieus/cmake/compile_naoqi_model.cmake: .l depends on .yaml
* Contributors: Kanae Kochigami, Kei Okada

1.0.2 (2016-02-14)
------------------

1.0.1 (2015-11-19)
------------------

* Add new package name naoqieus; move pepper-interface.l to naoqi-interface.l

  * naoqi-interface.l: /cmd_vel is published under global namespace
  * naoqieus: do not run test on hydro
  * add naoqieus/cmake/compile_naoqi_modeol.cmake
  * add naoqieus from peppereus

* Contributors: Kei Okada, Masahiro Bando

1.0.0 (2015-11-06)
------------------
* [pepper-interface.l] update :go-pos for new NoaQi drivers using move_base_simple/goal
* [pepper-interface.l] update joint action server name space for newer pepper_bringup
* Contributors: Kei Okada

0.0.13 (2015-11-06)
-------------------

0.0.12 (2015-11-06)
-------------------

0.0.11 (2015-09-01)
-------------------

0.0.10 (2015-08-16)
-------------------
* pepper-interface.l: use naoqi_bridge_msgs, but if not found use naoqi_msgs
* Contributors: Kei Okada

0.0.9 (2015-08-03)
------------------
* test/test-peppereus.l: add test codes
* Contributors: Kei Okada, Ryohei Ueda, Akira Kako

0.0.8 (2015-07-16)
------------------
* modify send-stiffness-controller method for pepper
* modify ros::advertise to be consistent with ros::publish
* modify speak method
* Contributors: Kanae Kochigami, Kei Okada, Jiang Jun

0.0.7 (2015-06-11)
------------------
* add speak method
* [pepper-interface.l] add error-vector method, this requires https://github.com/ros-naoqi/naoqi_bridge/pull/37
* [peppereus/pepper-interface.l] add :play-audio-file and :set-master-volume
* Contributors: Kanae Kochigami, Kei Okada, Ryohei Ueda

0.0.6 (2015-04-10)
------------------
* pepper-init added
* Contributors: kochigami

0.0.5 (2015-04-08)
------------------
* change nao_msgs to naoqi_msgs
* Contributors: Jiang Jun

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------
* use package:// for pepper.l
* Contributors: Kei Okada

0.0.1 (2014-12-25)
------------------
