jsk_fetch_gazebo_demo
=====================

- This demo is roseus version of [fetch\_gazebo\_demo](https://github.com/fetchrobotics/fetch_gazebo/tree/gazebo9/fetch_gazebo_demo).
- You can learn how to use roseus and how ROS components are executed. (e.g. pointcloud, navigation, motion planning, ... etc)

## Usage
- Build this package like [jsk\_fetch\_startup README](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_fetch_robot#setup-environment)
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_robot.git
rosdep install --from-paths . --ignore-src -y -r
cd ..
catkin config
catkin build jsk_fetch_gazebo_demo
source devel/setup.bash
```

- Launch demo
```bash
source devel/setup.bash
roslaunch jsk_fetch_gazebo_demo demo.launch
```

NOTE
- fetch\_gazebo is compatible with ROS indigo and melodic, so this repo is compatible with ROS indigo and melodic, too.
- Fetch in Gazebo9 (melodic) is somehow dark, but it seems no problem.

## Demo
#### Start of demo
Fetch generates the costmap (blue and purple thick lines) based on the known room map. AMCL particle (red arrows) is scattered at the beginning.

|Gazebo|Rviz|
|---|---|
|![](https://user-images.githubusercontent.com/19769486/78505523-f9f7c180-77ae-11ea-997f-379fdbf94c89.jpg)|![](https://user-images.githubusercontent.com/19769486/78505541-1ac01700-77af-11ea-98eb-e6c1b3e9caaf.png)|

#### Move to table using navigation stacks
While fetch is moving, AMCL particle (red arrows) come together by comparing laser data (red dotted line) and room map. Navigation path (green line) is calculated based on the costmap.

|Gazebo|Rviz|
|---|---|
|![](https://user-images.githubusercontent.com/19769486/78506252-565ce000-77b3-11ea-874d-fd1b966b7d15.jpg)|![](https://user-images.githubusercontent.com/19769486/78505555-3a573f80-77af-11ea-9ad3-e99fa06382be.png)|

#### Recognize obstacles by pointcloud
Fetch generates collision obstacles (green blocks) based on the table and cube pointcloud.

|Gazebo|Rviz|
|---|---|
|![](https://user-images.githubusercontent.com/19769486/78505574-5a86fe80-77af-11ea-803c-d4c45bc4d84d.jpg)|![](https://user-images.githubusercontent.com/19769486/78506589-96bd5d80-77b5-11ea-8220-24e5647998cc.png)|

#### Grasp cube
Fetch solves IK, moves the arm while avoiding obstacles and grasps the blue cube.

|Gazebo|Rviz|
|---|---|
|![](https://user-images.githubusercontent.com/19769486/78505616-9ae67c80-77af-11ea-87d8-dcb0a5714c78.jpg)|![](https://user-images.githubusercontent.com/19769486/78505631-a9cd2f00-77af-11ea-9410-773d85e8081e.png)|
