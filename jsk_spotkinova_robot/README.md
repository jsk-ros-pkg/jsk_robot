# jsk_spotkinova_robot

## How to setup develop environment

For more details about Spot and KINOVA arm,
please see [jsk_spot_robot](../jsk_spot_robot) and [jsk_kinova_robot](../jsk_kinova_robot)

### Setup a workspace for spot and kinova (for a user)

You need to install `conan` (Decentralized, open-source (MIT), C/C++ package manager) to build kinova packages.
If you have not installed `conan`, please see [Conan Setup](../jsk_kinova_robot#conan-setup) and install `conan`

```bash
mkdir ~/spotkinova_ws/src -p
cd spotkinova_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge jsk-ros-pkg/jsk_robot/jsk_kinova_robot/jsk_kinova.rosinstall
wstool merge jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_user.rosinstall
wstool update
rosdep update
rosdep install --from-paths . --ignore-src -y -r
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd ~/spotkinova_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin init
catkin build jsk_spotkinova_startup
```

### Setup a workspace for spot and kinova (for an internal PC)

You need to install `conan` (Decentralized, open-source (MIT), C/C++ package manager) to build kinova packages.
If you have not installed `conan`, please see [Conan Setup](../jsk_kinova_robot#conan-setup) and install `conan`

```bash
mkdir ~/spotkinova_ws/src -p
cd spotkinova_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge jsk-ros-pkg/jsk_robot/jsk_kinova_robot/jsk_kinova.rosinstall
wstool merge jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall
wstool update
rosdep update
rosdep install --from-paths . --ignore-src -y -r
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd ~/spotkinova_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build -j4 -c jsk_spotkinova_startup
```

Please see additional config ( e.g. authentification config for Spot ) for [jsk_spot_robot](../jsk_spot_robot#How-to-run)

## How to run

### Bringup robot

1. Turn on spot and turn on motors according to [Bringup spot](../jsk_spot_robot#bringup-spot) 
2. Switch on kinova.
3. Please run the ros driver and other basic programs with `spotkinova_bringup.launch`.

```bash
source ~/spotkinova_ws/devel/setup.bash
roslaunch jsk_spotkinova_startup spotkinova_bringup.launch
```

### How to control spotkinova from roseus

For more details, please see [jsk_kinova_robot](../jsk_kinova_robot#use-euslisp-model) and [jsk_spot_robot](../jsk_spot_robot).
Please start roseus and type as follows.

```
(load "package://spotkinovaeus/spotkinova-interface.l")
(spotkinova-init)
```

Below is a list of typical posture commands for kinova.
`:kinova-rest-pose` is a good posture for Spot to adopt when moving or resting.  
You should use `:kinova-rest-pose` when moving.
```
(send *spotkinova* :reset-pose)
(send *spotkinova* :init-pose)
(send *spotkinova* :kinova-rest-pose)
```

To obtain current robot pose, use `:state :potentio-vector` method.
This shows all joint angles, including spot and kinova.  
The front 12 dimensions represent spot and the rest 6 dimensions represent kinova.
```
(send *ri* :state :potentio-vector) ;; #f(0.0 0.0 45.0 0.0 0.0 45.0 0.0 0.0 90.0 0.0 0.0 90.0 -0.00293 -0.165863 0.162064 -0.104492 0.045563 -0.000641)
```
Use `:angle-vector` method to specify the arm joint angle.  
If you want to send an angle-vector to kinova, you need to send the joint angles in 18 dimensions as follows.  
The first 12 dimensions are for spot, but they will be ignored and the last 6 dimensions will be sent to kinova only.
```
(send *ri* :angle-vector #f(0.0 0.0 45.0 0.0 0.0 45.0 0.0 0.0 90.0 0.0 0.0 90.0 0.0 15.0 180.0 -130.0 0.0 55.0 90.0))
```
To move the gripper 50 [mm] up, you can use `move-end-pos` method.
```
(send *spotkinova* :head :move-end-pos #f(0 0 -50))
```
You can also use `move-end-rot` method to turn the gripper.
```
(send *spotkinova* :head :move-end-rot -90 :z)
```
You can use `inverse-kinematics` to move arm.
```
(send *spotkinova* :head :inverse-kinematics (make-coords :pos #f(700 0 500) :rotation-axis nil))
```
You can use `body-inverse-kinematics` to move body.
```
(dotimes (i 100)
  (send *spot* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 (* 20 (sin (* pi i 0.02))))
                     :rpy (float-vector (* 0.2 (sin (* pi i 0.02))) (* 0.2 (sin (* pi i 0.02))) (* 0.2 (sin (* pi i 0.02))))
  (send *ri* :body-pose (send *spot* :copy-worldcoords)) ;; when sending to real robot
  )
```
You can use `fullbody-inverse-kinematics` to move arm and body.
```
(let ((arm-pose (send *spotkinova* :head :end-coords :copy-worldcoords)))
  (send *spotkinova* :fullbody-inverse-kinematics
        (send arm-pose :translate
              (float-vector 0 600 -200)
              :world)
      :root-link-virtual-joint-weight #f(0.0 0.0 0.1 0.1 0.5 0.5)))
```
You can send trajectory to real robots.  
When you move both arm and body, you have to move body first.  
Arm servo turns off because of large joint torque error.  
```
(send *ri* :body-pose (send *spotkinova* :copy-worldcoords)) ;; for spot posture
(send *ri* :angle-vector (send *spotkinova* :angle-vector) 5000) ;; for kinova
```
