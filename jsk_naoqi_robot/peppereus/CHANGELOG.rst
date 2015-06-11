^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package peppereus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
