## Introduction

This project contains a set of patches and scripts to compile and run ROS1 on a Pepper robot, without the need of a tethered computer, based on https://github.com/esteve/ros2_pepper and https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_unitree_robot/cross/README.md

## Setup Pepper Robot

### Prerequisites

- Pepper robot running NAOqi OS 2.5.X

Note that this project does not cover NAOqi OS 2.9.

As for robot version, please refer to http://doc.aldebaran.com/2-8/family/pepper_technical/pepper_versions.html

- Linux PC which meets the prerequisites at https://docs.docker.com/engine/install/ubuntu/#prerequisites to prepare cros-compiling environment and copy system to Pepper.

- (Recommendation) Linux PC (OS 18.04 + ROS melodic or OS 16.04 + ROS kinetic) which has environment for NAOqi robot for developping programs for Pepper. For further information, please refer to https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot#setup-environment.

### Prepare cross-compiling environment (Run only the fist time per host computer)

We're going to use Docker to set up a container that will compile all the tools for cross-compiling ROS and all of its dependencies. Go to https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository to install it for your Linux distribution.

Also, when you verify that the Docker Engine installation is successful by running the hello-world image, it is recommended to add `-rm` option like `sudo docker run --rm hello-world`.

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
sudo apt install python-vcstool sshpass
```

If you use Ubuntu 20.04, please execute the command below:

```
sudo apt install python3-vcstool sshpass
```

### Build ROS System on Docker  (Run only the fist time per host computer)

```
make system
```

Caution!!! It will take more than a few hours !!

For JSK users, you can find latest backup of `i386_System.tgz` at [Google Drive](https://drive.google.com/drive/u/1/folders/10rINVGt1iDM2WNofmf0sZBX_iTnpXya6).

You can unzip this by:
```
gzip -d i386_System.tgz
tar -xvf i386_System.tar
```
For further information, please see https://github.com/jsk-ros-pkg/jsk_robot/pull/1583#issuecomment-1235043472

Run following command to copy ROS1 base system to Pepper onboard computer. This should be done only in the first time. So normally user do not have to run this command. You need to specify password by -p option.
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

ssh to NAO_IP machine and run this command directly under the home directory.
```
./User/start.sh
```

`C-t d` enables you to detach a session from a terminal.

If you want to attach the session again, please execute `./User/attach.sh`.

If there are several sessions like this (You can see same outputs with `screen -c User/screenrc -ls`.):
```
There are several suitable screens on:
	31164.session	(04/04/14 12:28:31)	(Detached)
	29334.session	(04/04/14 12:26:33)	(Detached)
Use -S to specify a session.
```

Please specify a session like `screen -c User/screenrc -r 31164` and it is recommended to close all sessions one by one.

You can connect to `app_chooser` by http://<$NAO_IP>:8000/rwt_app_chooser/#!robot

Also, you can access to running ROS process on Pepper from your remote PC by typing `rossetmaster <$NAO_IP>`

## Known Issues

### `./install.sh -d System` errors at the first time.

`./install.sh -d System` requires `User` environment, so it causes error at the first time. You need to re-run `./install.sh` after you installed User space.

### `./build_user.sh build -c` fails

`./build_user.sh build -c` fails as follows. This is expected behavior and we recommend to combile all packages in the first time. `make user` runs `catkin build jsk_pepper_startup peppereus` and pacakages such as `pepper_meshes` and `rosbash` is not compiled with this command.

```
 [    Failed] roseus_remote
 [    Failed] speak_and_wait_recovery
```

### To comple `pepper.l` within the robot

Note: This issue was solved by https://github.com/kochigami/jsk_robot/commit/1a0cfe08e51421a2378bc2938ea3014b62e788fb and https://github.com/jsk-ros-pkg/jsk_robot/pull/1847

Please refer to this comment: https://github.com/jsk-ros-pkg/jsk_robot/pull/1583#issuecomment-1235043472
```
cp -r /opt/ros/$ROS_DISTRO/share/pepper_meshes/meshes/  ./i386_User/src/pepper_meshes/
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

If you want to use ethernet, you need to change from `wlan0` to `eth0` in `jsk_naoqi_robot/cross/startup_scripts/user_setup.bash` and `jsk_naoqi_robot/cross/startup_scripts/start.sh`, and run `make install`
```
export ROS_IP=$(ip addr show eth0 | grep -Po '(?<= inet )([0-9]{1,3}.){3}[0-9]{1,3}')
```

```
screen -c User/screenrc -S session -p 0 -X stuff "roslaunch jsk_pepper_startup jsk_pepper_startup.launch launch_dashboard:=false network_interface:=eth0
```

If you add more dependencies to `package.xml`, you need to remove `i386_Users` and run `make user` again.


You can restore from saved docker container. For JSK users, you can find latest data at [Google Drive](https://drive.google.com/drive/u/1/folders/10rINVGt1iDM2WNofmf0sZBX_iTnpXya6). You can also find backup of `i386_System`.
```
docker load < ros1-pepper.tar
```