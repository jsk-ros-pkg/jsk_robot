================
jsk_owl_exporter
================

An OWL Exporter for JSK Robot Episodic Database System


How to use
----------

* List logged task IDs

  .. code-block:: bash

     $ rosrun jsk_owl_exporter export_log.py list
     
         Task ID
     819CD86E-A4F4-11E6-AC46-C85B763717C4
     36332C4A-A4F2-11E6-AA60-C85B763717C4


* Show information about the task

  .. code-block:: bash

     $ rosrun jsk_owl_exporter export_log.py info --task 819CD86E-A4F4-11E6-AC46-C85B763717C4
     
     Date          | 1970/01/01 00:00:21 - 1970/01/01 00:01:03
     Task ID       | 819CD86E-A4F4-11E6-AC46-C85B763717C4
     Data Size     | 4271                          
     Task Name     | pr2-fetch-and-place           
     Statistics
       Data Type                                          |   Size
       move_base_msgs/MoveBaseActionResult                |      1
       control_msgs/FollowJointTrajectoryActionFeedback   |   3761
       control_msgs/FollowJointTrajectoryActionResult     |     77
       jsk_robot_startup/ActionEvent                      |     32
       posedetection_msgs/Object6DPose                    |     24
       geometry_msgs/TransformStamped                     |     36
       sensor_msgs/JointState                             |     97
       move_base_msgs/MoveBaseActionGoal                  |      1
       control_msgs/FollowJointTrajectoryActionGoal       |     88
       move_base_msgs/MoveBaseActionFeedback              |    154

* Visualize event graph

  .. code-block:: bash

     $ rosrun jsk_owl_exporter export_log.py graph --task 819CD86E-A4F4-11E6-AC46-C85B763717C4 --output episode1

     600 documents found.
     saved to episode1/graph.pdf

     $ gnome-open episode1/graph.pdf

* Export event data to OWL file

  .. code-block:: bash

     $ rosrun jsk_owl_exporter export_log.py export --task 819CD86E-A4F4-11E6-AC46-C85B763717C4 --output episode1

     600 documents found.
     saved to episode1/log.owl

* Export geometry data to JSON file

  .. code-block:: bash

     $ roslaunch jsk_owl_exporter robot_transform_server_pr2.launch

     $ rosrun jsk_owl_exporter export_transform.py 819CD86E-A4F4-11E6-AC46-C85B763717C4 --output episode1

     exporting 4072 messages to episode1/tf.json


Author
------

Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
