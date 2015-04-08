^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_robot_startup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2015-04-08)
------------------
* [jsk_baxter_startup] update to add position diff paramter for tweet
* [jsk_baxter_startup] modify to prevent baxter.launch fail
* [jsk_robot_startup/package.xml: add diagnostic_msgs, pr2_mechanism_controllers, sensor_msgs to build dependencies
* [sk_robot_startup/CMakeLists.txt] update to set permission for installed script files
* [jsk_robot_startup] modfiy CMakeLists.txt to install jsk_robot_startup correctly
* [jsk_robot_startup/lifelog/active_user.l] repair tweet lifelog
* [jsk_robot_startup/lifelog/mongodb.launch] fix typo of option in launch
* [jsk_robot_startup/lifelog/mongodb.launch: add mongodb launch; mongod kill watcher
* Contributors: Yuki Furuta, Yuto Inagaki

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------

0.0.1 (2014-12-25)
------------------
* check joint state and set movep for odom disable robot
* Add sound when launching pr2.launch
* Say something at the end of pr2.launch
* move twitter related program to robot_common from jsk_pr2_startup
* add ros-info
* robot time signal
* add tweet.l, see jsk_nao_startup.launch for example
* repiar mongodb.launch
* repair mongodb.launch and add param
* add jsk_robot_common/jsk_robot_startup
* Contributors: Kanae Kochigami, Ryohei Ueda, Yuto Inagaki, Yusuke Furuta
