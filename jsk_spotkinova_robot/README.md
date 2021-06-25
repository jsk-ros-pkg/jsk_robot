# jsk_spotkinova_robot
## How to setup develop environment
For more details about Spot and KINOVA arm,
please see [jsk_spot_robot](https://github.com/sktometometo/jsk_robot/tree/develop/spot/jsk_spot_robot) and
[jsk_kinova_robot](https://github.com/708yamaguchi/jsk_robot/tree/spot-kinova/jsk_kinova_robot)

### Setup a workspace for spot and kinova
You need to install `conan` (Decentralized, open-source (MIT), C/C++ package manager) to build kinova packages  
If you have not installed `conan`, please see [Conan Setup](https://github.com/708yamaguchi/jsk_robot/tree/kinova-gen3/jsk_kinova_robot#conan-setup) and install `conan`
```bash
mkdir ~/spotkinova_ws/src -p
cd spotkinova_ws/src
wstool init .
wstool merge -t . https://raw.githubusercontent.com/708yamaguchi/jsk_robot/spot-kinova/jsk_spotkinova_robot/jsk_spotkinova.rosinstall
wstool update
##  If jsk_spot_robot package is updated, we need to merge it until the package is merged into master.
# cd jsk-ros-pkg/jsk_robot
# git remote add sktometometo https://github.com/sktometometo/jsk_robot.git
# git fetch sktometometo
# git merge sktometometo/develop/spot
# cd ~/spotkinova_ws/src
rosdep update
rosdep install --from-paths . --ignore-src -y -r
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd ~/spotkinova_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin init
catkin build jsk_spotkinova_startup
```
