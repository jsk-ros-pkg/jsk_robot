;; 2013/06/21
(ros::roseus "nao")

(load "package://naoeus/euslisp/nao-interface.l")
(ros::load-ros-manifest "naoeus")

#|
send *nao* :joint-list :name
("LShoulderPitch" "LShoulderRoll" "LElbowYaw" "LElbowRoll" "LWristYaw" "RShoulderPitch" "RShoulderRoll" "RElbowYaw" "RElbowRoll" "RWristYaw" "LHipYawPitch" "LHipRoll" "LHipPitch" "LKneePitch" "LAnklePitch" "LAnkleRoll" "RHipYawPitch" "RHipRoll" "RHipPitch" "RKneePitch" "RAnklePitch" "RAnkleRoll" "HeadYaw" "HeadPitch")
+RHand LHand

rostopic echo /joint-states
name: ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowY
aw', 'LElbowRoll', 'LWristYaw', 'LHand', 'LHipYawPitch', 'LHipRoll', 'LHip
Pitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRo
ll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPit
ch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']    |#

(send *nao* :joint-list :name)

setq input-name-list (list "HeadYaw" "HeadPitch" "LShoulderPitch" "LShoulderRoll" "LElbowYaw" "LElbowRoll" "LWristYaw" "LHand" "LHipYawPitch" "LHipRoll" "LHipPitch" "LKneePitch" "LAnklePitch" "LAnkleRoll" "RHipYawPitch" "RHipRoll" "RHipPitch" "RKneePitch" "RAnklePitch" "RAnkleRoll" "RShoulderPitch" "RShoulderRoll" "RElbowYaw" "RElbowRoll" "RWristYaw" "RHand")

(dolist (n input-name-list) (print n))

#|
---
header: 
  seq: 101
  stamp: 
    secs: 1371801922
    nsecs: 58305978
  frame_id: /torso
name: ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']
position: [0.038308143615722656, -0.14730596542358398, 2.0187020301818848, 0.4724299907684326, -1.830103874206543, -0.7102000713348389, -1.8238691091537476, 0.6247999668121338, -0.16869807243347168, 0.1365680694580078, -1.213352084159851, 0.19477605819702148, -0.37126994132995605, -0.19170808792114258, -0.16869807243347168, 0.0031099319458007812, -1.0416278839111328, -0.0014920234680175781, -0.3512439727783203, 0.01691603660583496, 2.0856685638427734, 0.12114405632019043, 1.9864881038665771, 1.486487865447998, 1.3974320888519287, 0.8600000143051147]
velocity: []
effort: []
|#

#|
(#<rotational-joint #X5568718 LShoulderPitch> #<rotational-joint #X5560170\
 LShoulderRoll> #<rotational-joint #X555ff78 LElbowYaw> #<rotational-joint\
 #X555fd80 LElbowRoll> #<rotational-joint #X555fb88 LWristYaw> #<rotationa\
l-joint #X5568b08 RShoulderPitch> #<rotational-joint #X556b2c8 RShoulderRo\
ll> #<rotational-joint #X556b0d0 RElbowYaw> #<rotational-joint #X556aed8 R\
ElbowRoll> #<rotational-joint #X556de00 RWristYaw> #<rotational-joint #X55\
6b6b8 LHipYawPitch> #<rotational-joint #X5562600 LHipRoll> #<rotational-jo\
int #X5562408 LHipPitch> #<rotational-joint #X5562210 LKneePitch> #<rotati\
onal-joint #X5562018 LAnklePitch> #<rotational-joint #X5564e80 LAnkleRoll>\
 #<rotational-joint #X5568910 RHipYawPitch> #<rotational-joint #X556d818 R\
HipRoll> #<rotational-joint #X556d620 RHipPitch> #<rotational-joint #X5570\
500 RKneePitch> #<rotational-joint #X5570308 RAnklePitch> #<rotational-joi\
nt #X5570158 RAnkleRoll> #<rotational-joint #X556b4c0 HeadYaw> #<rotationa\
l-joint #X5564898 HeadPitch>)
|#