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

Do not forget connecting to Spot Wi-Fi, (network name is "Belka")

```bash
rossetip
rossetmaster belka.local
```
*This network is unstable.*

And source spotkinova workspace.

```bash
source ~/spotkinova_ws/devel/setup.bash
roslaunch jsk_spotkinova_startup spotkinova_bringup.launch
```

### How to dock / undock spot

To undock, use the controller or

```
rosservice call /spot/undock
```

To dock, move spot besides the dock at first and

```
rosservice call /spot/dock "dock_id:[id]
````

### When Ros cannot connect to Belka

1. ssh belka.local
2. check the log

```bash
ssh spot@belka.local
[password]
sudo journalctl -u jsk-spotkinova-ros-bringup.service
```

3. stop and restart rosbringup.service

```bash
sudo systemctl stop jsk-spotkinova-ros-bringup.service
sudo systemctl start jsk-spotkinova-ros-bringup.service
```

*Spot interface can be controlled by the exclusive tablet.
The tablet has higher priority to control than the ROS interface.
If you hijack with the tablet, release at first and reconnect with ROS.*


### How to control spotkinova from roseus

For more details, please see [jsk_kinova_robot](../jsk_kinova_robot#use-euslisp-model) and [jsk_spot_robot](../jsk_spot_robot).
Please start roseus and type as follows.

```
(load "package://spotkinovaeus/spotkinova-interface.l")
(spotkinova-init)
```
This is a robot interface class for using spot + kinova.                    
This class is designed to be able to call both the methods defined in spot-interface and kinova-inte\
rface.
The following is priority when calling methods of the same name defined in different classes.        
1. spotkinova-interface 2. robot-interface 3. spot-interface 4. kinova-interface                     
Please see the following [page](https://github.com/euslisp/EusLisp/issues/454#issuecomment-863136824)


If you want to simulate without real interface,

```
(load "package://spotkinovaeus/spotkinova.l")
(spotkinova)
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
### Control the full body

To change the full body pose, you can use `body-pose` method with rpy list or coords variable.
```
(send *ri* :body-pose '(0 0.2 0))
(send *ri* :body-pose '(0 -0.2 0))
(send *ri* :body-pose '(0 0 0.2))
(send *ri* :body-pose '(0 0 -0.2))
(send *ri* :body-pose '(0.1 0 0))
(send *ri* :body-pose '(-0.1 0 0))
(send *ri* :body-pose (make-coords :pos #f(0 0 50) :rpy #f(0.1 0 0)))
```
To move the full body back and forwad, You can use `go-pos` method.
(x[m] y[m] z[degree])
```
(send *ri* :go-pos 1 0 0) ;; 1m forward
(send *ri* :go-pos 0 1 0) ;; 1m move left
(send *ri* :go-pos 0 0 90) ;; rotate 90 degree
```
You can use `body-inverse-kinematics` to move body.
```
(dotimes (i 100)
  (send *spotkinova* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 (* 20 (sin (* pi i 0.02))))
                     :rpy (float-vector (* 0.2 (sin (* pi i 0.02))) (* 0.2 (sin (* pi i 0.02))) (* 0.2 (sin (* pi i 0.02))))
  (send *ri* :body-pose (send *spotkinova* :copy-worldcoords)) ;; when sending to real robot
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

### Control the kinova part

*Regards to spotkinova robot, kinova is "head". (not arm)*

To move the gripper 50 [mm] up, you can use `move-end-pos` method.
```
(send *spotkinova* :head :move-end-pos #f(0 0 -50))
```
To grasp and release with kinova, 
```
(send *ri* :start-grasp) 
(send *ri* :stop-grasp)
```
And control grasp force, (range [-0.05 ~ 0.95])
```
(send *ri* :go-grasp-lite :pos 0.5)
```

You can also use `move-end-rot` method to turn the gripper.
```
(send *spotkinova* :head :move-end-rot -90 :z)
```
You can use `inverse-kinematics` to move arm.
```
(send *spotkinova* :head :inverse-kinematics (make-coords :pos #f(700 0 500) :rotation-axis nil))
```

### How to use spotkinova's camera

There are some comera types.

1. Spot (ex. /spot/camera/frontleft/image) ;; This image is monochrome.
2. Kinova (ex. /kinova_wrist_camera/color/image_raw) ;; This image is RGB color, but the initial image is upside down.
3. 360 degree camera (ex. /dual_fisheye_to_panorama/output_mouse_left) ;; This image is RGB color and 360 degree.

[HSI color filter](https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/jsk_pcl_ros/nodes/hsi_color_filter.html) can be used with spotkinova's camera.

*This process is only for RGB color camera, so spot camera itself cannot use this filter.*

[Coral USB](https://github.com/knorth55/coral_usb_ros.git) is plugged into the Spotkinova's internel PC, so you can use it with spotkinova's camera.

*This process needs ssh*

```bash
ssh spot@belka.local
[password]
source /opt/ros/melodic/setup.bash
source $HOME/spot_coral_ws/devel/setup.bash
rossetip
rossetmaster belka.local
roslaunch coral_usb edgetpu_object_detector.launch INPUT_IMAGE:=/kinova_wrist_camera/color/image_raw
```

If you want to pointclouds in combination with Coral USB, use the aligned topic (/kinova_wrist_camera/aligned_depth_to_color/image_raw)
because the resolution of kinova's RGB camera and depth camera do not match.
For more details, see this [issue](https://github.com/HiroIshida/snippets/issues/29).
