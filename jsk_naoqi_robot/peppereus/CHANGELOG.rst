^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package peppereus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
