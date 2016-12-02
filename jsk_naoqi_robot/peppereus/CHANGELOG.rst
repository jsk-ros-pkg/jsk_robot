^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package peppereus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2016-11-09)
------------------

1.0.8 (2016-11-08)
------------------

1.0.7 (2016-11-02)
------------------
* [naoqieus, peppereus] delete wheel controller
* [jsk_naoqi_robot] add roseus in find_package
* add controllers
* rotation of rleg endcoords modified
* [peppereus] add head, larm, rarm, rleg end coords (`#637 <https://github.com/jsk-ros-pkg/jsk_robot/issues/637>`_)
* Contributors: Kanae Kochigami

1.0.6 (2016-06-17)
------------------

1.0.5 (2016-04-18)
------------------
* move send-stiffness-controller, servo-on, servo-off methods to naoqi-interface
* Contributors: Kanae Kochigami

1.0.4 (2016-03-21)
------------------

1.0.3 (2016-03-05)
------------------
* modify service type
* add :show-app, :show-webview, :s(g)et-show-image-folder-path
* add :show-image, :show-webview, :hide-image
* Contributors: Kanae Kochigami

1.0.2 (2016-02-14)
------------------

1.0.1 (2015-11-19)
------------------
* move peppereus under jsk_naoqi_robot
* naoeus,peppereus: package.xml: fix typo in run_depend, naoqi->naoqieus, nap_apps->nao_apps
* add naoqieus/cmake/compile_naoqi_modeol.cmake
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
* do not add naoqi_driver in find_package
* pepper-interface.l: use naoqi_bridge_msgs, but if not found use naoqi_msgs
* CMakeLists.txt, package.xml: add nao_interaction_msgs
* add depends to naoqi_driver
* roseus/CMakeLists.txt: add rostest, roseus to find_pakcage
* Contributors: Kei Okada

0.0.9 (2015-08-03)
------------------
* since ros-naoqi repository chages names (naoqi_sensors -. naoqi_sensors_py), we removed unstable pacagkes
* [peppereus] Do not run test if no meshes are found
* add sed command of correct spell
* test/test-peppereus.l: add test codes
* Contributors: Kei Okada, Ryohei Ueda, Akira Kako

0.0.8 (2015-07-16)
------------------
* modify send-stiffness-controller method for pepper
* modify ros::advertise to be consistent with ros::publish
* modify speak method
* [package.xml] add nao_interaction_msgs to depends
* 0.0.7
* Contributors: Kanae Kochigami, Kei Okada, Jiang Jun

0.0.7 (2015-06-11)
------------------
* add speak method
* [pepper-interface.l] add error-vector method, this requires https://github.com/ros-naoqi/naoqi_bridge/pull/37
* [peppereus] Compilation does not fail even though pepper_urdf is not
  installed because the package is not yet released
* [peppereus/CMakeLists.txt] run pepper.l only when pepper_meahes found
* [CMakeLists.txt, pepper.yaml] generate pepper model from pepper_description
* [package.xml] add more depends
* [.gitignore] ignore dae file too
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
* add .gitignore
* add CMakeLists.txt package.xml pepper-interface.l
* Contributors: Kei Okada

0.0.1 (2014-12-25)
------------------
