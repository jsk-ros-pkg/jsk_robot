## Introduction

This project contains a set of patches and scripts to compile and run ROS1 on a Pepper robot, without the need of a tethered computer, based on https://github.com/esteve/ros2_pepper and https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_unitree_robot/cross/README.md

## Setup Pepper Robot

### Prepare cross-compiling environment (Run only the fist time per host computer)

We're going to use Docker to set up a container that will compile all the tools for cross-compiling ROS and all of its dependencies. Go to https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository to install it for your Linux distribution.

1. Add your user to docker group
```
$ sudo usermod -aG docker $USER
$ newgrp
```

2. Install Qemu software
```
$ sudo apt install -y qemu-user-static
```

3. Install Aldebaran's cross environment tool-chains

Download and extract ctc-linux64-atom-2.5.10.7.zip (use [GoogleDrive](https://drive.google.com/drive/folders/1P49oBEobwyVI4TG1nxxXUftc9TLEYEc7) for JSK Users) in your home directory.

Set environment variable to your `.bashrc`

```
export ALDE_CTC_CROSS=$HOME/ctc-linux64-atom-2.5.10.7
```

4. Install command used in scripts

```
apt install python-vcstool sshpass
```

### Build ROS System on Docker  (Run only the fist time per host computer)

```
make system
```

Caution!!! It will take more than a few hours !!

Run following command to copy ROS1 base sytem to Pepper onboard computer. This should be done only in the first time. So normally user do not have to run this command. You need to specify password by -p option.
```
./install.sh -p <robot password> -d System
```

### Build `jsk_pepper_robot` software on Docker

You can build your current `jsk_pepper_robot` workspace on Docker environment which is ready to copy into Pepper onboard computer.

```
make user
```

To copy your software to Pepper onboard computer, run following command
```
./install.sh -p <robot password>
```

## Start `jsk_pepper_startup.launch`

ssh to NAO_IP machine and run
```
source User/user_setup.bash
roslaunch --screen jsk_pepper_startup jsk_pepper_startup.launch network_interface:=eth0 launch_dashboard:=false launch_joy:=false
```

You can connect to `app_chooser` by http://<$NAO_IP>:8000/rwt_app_chooser/#!robot

## Known Issues

### `./install.sh -d System` errors at the first time.

./install.sh -d System` requires `User` environment, so it causes error at the first time. You need to re-run `./install.sh` after you installed User space.

### `./build_user.sh build -c` fails

`./build_user.sh build -c` fails as follows. This is expected behavior and we recommend to combile all packages in the first time. `make user` runs `catkin build jsk_pepper_startup peppereus` and pacakages such as `pepepr_meshes` and `rosbash` is not compiled with this command.

```
 [    Failed] roseus_remote
 [    Failed] speak_and_wait_recovery
```

### To comple `pepper.l` within the robot

```
cp -r /opt/ros/melodic/share/pepper_meshes/meshes/  ./i386_User/src/pepper_meshes/
rm -fr i386_User/build/pepper_meshes/ i386_User/build/peppereus
./build_user.sh build pepper_meshes peppereus
```

### Development

On development phase, users are expected to develop sofoware on a remote machine. All codes are expected to add in `jsk_pepper_startup` package.


You can send all development files to robot and start them on boot time. Note that this process requres `NAO_IP` environment variable.

```
make user
make install
```

If you want to use ethernet, you need to change from `wlan0` to `eth0` in `jsk_naoqi_robot/cross/startup_scripts/user_setup.bash` and run `make install`
```
export ROS_IP=$(ip addr show eth0 | grep -Po '(?<= inet )([0-9]{1,3}.){3}[0-9]{1,3}')
```

If you add more dependencies to `package.xml`, you need to remove `i386_Users` and run `make user` again.