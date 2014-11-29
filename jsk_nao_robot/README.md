#nao euslisp
##Generate nao.l
```
catkin_make --only-pkg-with-deps euscollada
roscd euscollada
./nao.sh
```

##Improve nao.l
```
catkin_make --only-pkg-with-deps eus_assimp
roscd naoeus/scripts
./setup.sh
```

#nao gazebo
##Dependencies
```
sudo apt-get install ros-hydro-joint-state-controller
sudo apt-get install ros-hydro-position-controller
sudo apt-get install ros-hydro-gazebo-ros-control
```
##Download model file
```
catkin_make --only-pkg-with-deps nao_meshes
roscd
cd ../build/ros_nao/nao_meses
make nao_meshes_meshes
roscd
cp -r tmp/nao_meshes/* ../src/ros_nao/nao_meshes/
```
##How to run
```
roslaunch nao_gazebo_plugin nao_gazebo_plugin_H25.launch
```
