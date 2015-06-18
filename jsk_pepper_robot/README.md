jsk_pepper_robot
================

setup environment
-----------------
```
mkdir -p catkin_ws/src
cd  catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_pepper_robot/pepper.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src
catkin_make
source devel/setup.bash
```
Make sure that you have already installed the ``Python NAOqi SDK`` in your computer. If not, you can download it from [here](https://community.aldebaran.com/en/resources/software). After downloading the file, unzip and rename it to ``pynaoqi``, then put it under your home folder.
 
Export NAO_IP environment variable and add PYTHON PATH to NAOqi Python SDK path by adding these three lines of code to you ``.bashrc`` file.      
```
source ~/catkin_ws/devel/setup.bash
export PYTHONPATH=$HOME/pynaoqi:$PYTHONPATH
export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp"
```

Install pepper mesh files with manual approval of license
```
sudo apt-get install ros-indigo-pepper-meshes
```

running demo
------------
```
roslaunch jsk_pepper_startup jsk_pepper_startup.launch
```
```
rosrun jsk_pepper_startup sample.l
$ (demo1)
```
