^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetcheus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-07-16)
------------------
* fetcheus/CMakeLists.txt : add install rule (`#763 <https://github.com/jsk-ros-pkg/jsk_robot/issues/763>`_)
* [Fetch] jsk_fetch_robot/fetcheus/fetch-interface.l :angle-vector,
  :angle-vector-sequence supports old API (`#795 <https://github.com/jsk-ros-pkg/jsk_robot/issues/795>`_ )
  * pass start-time to start-offset-time and support :start-time key
  * support :start-time key in angle-vector methods
  * pass start-time in start-offset-time
  * support old API
  * :angle-vector-sequence accept tms in list of time
  * add :angle-vector-sequence with moveit in fetcheus
  * update :angle-vector in fetch-interface

* [fetcheus] disable gui in fetch-moveit.test (`#778 <https://github.com/jsk-ros-pkg/jsk_robot/issues/778>`_ )
  * disable gui in fetch-moveit.test
  * use simulation.launch instead of playground.launch

* [fetch] Correct head orientation for look-at `#771 <https://github.com/jsk-ros-pkg/jsk_robot/issues/771>`_ (`#773 <https://github.com/jsk-ros-pkg/jsk_robot/issues/773>`_)
  * Correct fetch head orientation
  * test/test-fetcheus.l: add test to check `#771 <https://github.com/jsk-ros-pkg/jsk_robot/issues/771>`_
  * package.xml: add depends to pr2eus_moveit

* [fetcheus] modify :reset-pose (`#742 <https://github.com/jsk-ros-pkg/jsk_robot/issues/742>`_)
  * send *fetch* :torso :waist-z :joint-angle 20
    moveit need more space between base and `elbow_flex_link`

* [fetch-robot] set effort to 40 in :go-grasp (`#727 <https://github.com/jsk-ros-pkg/jsk_robot/issues/727>`_ )
  * set effort to 40 in :go-grasp

* fetch-interface.l : add args for :speak methods (`#723 <https://github.com/jsk-ros-pkg/jsk_robot/issues/723>`_)
* test/test-fetcheus.l: add test to check :angle-vector on kinematics simulator (`#717 <https://github.com/jsk-ros-pkg/jsk_robot/issues/717>`_ )
  * use :angle-vector-raw for simulation mode
  * test/test-fetcheus.l: add euslisp test to check :angle-vector on kinematics simulator

* Contributors: Noya Yamaguchi, Guilherme-Affonso, Yuki Furuta, Kei Okada, Shingo Kitagawa

1.0.9 (2016-11-09)
------------------
* add :speak methods to fetch-interface
* Contributors: Kei Okada

1.0.8 (2016-11-08)
------------------
* :angle-vector, add :clear-velocities t, this requries https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/255
* add test to check https://github.com/jsk-ros-pkg/jsk_robot/issues/702
* :inverse-kinematics do not update link-list if given
* use fetch-utils.l from fetch-interface.l
* add fetch-utils.l
* Contributors: Kei Okada

1.0.7 (2016-11-02)
------------------
* set :use-torso as default behavior for :angle-vector
* set base-controller-action-name nil
* Contributors: Kei Okada

1.0.6 (2016-06-17)
------------------
* feteus-interface.l : always use moveit within :angle-vector (`#620 <https://github.com/jsk-ros-pkg/jsk_robot/issues/620>`_)
  * fetcheus: fetch-interface.l : always use moveit within :angle-vector
  * fetcheus : add test for fetch-moveit-interface
  * feetcheus : fix typo in fetcheus.test
* Contributors: Kei Okada

1.0.5 (2016-04-18)
------------------
* Control gripper from robot interface
* Fix arm end coords of 'fetch.l'
  Modified:
  - jsk_fetch_robot/fetcheus/fetch.yaml
* Contributors: Kentaro Wada

1.0.4 (2016-03-21)
------------------
* check collad-dom version before convert from urdf to collada
* test/test-fetcheus.l: add test for fetch-interface
* fetch-interface.l : fix robot-move-base-interface.l
* Contributors: Kei Okada

1.0.3 (2016-03-05)
------------------
* add jsk_fetch_robot package
* Contributors: Kei Okada

1.0.2 (2016-02-14)
------------------

1.0.1 (2015-11-19)
------------------

1.0.0 (2015-11-06 15:17)
------------------------

0.0.13 (2015-11-06 15:04)
-------------------------

0.0.12 (2015-11-06 14:47)
-------------------------

0.0.11 (2015-09-01)
-------------------

0.0.10 (2015-08-16)
-------------------

0.0.9 (2015-08-03)
------------------

0.0.8 (2015-07-16)
------------------

0.0.7 (2015-06-11)
------------------

0.0.6 (2015-04-10)
------------------

0.0.5 (2015-04-08)
------------------

0.0.4 (2015-01-30)
------------------

0.0.3 (2015-01-09)
------------------

0.0.2 (2015-01-08)
------------------

0.0.1 (2014-12-25)
------------------
