## Introduction

This project contains a set of patches and scripts to compile and run ROS1 on a Go1 robot, without the need of a tethered computer, based on https://github.com/esteve/ros2_pepper

## Setup Go1 Robot

### Prepare cross-compiling environment (Run only the fist time per host computer)

We're going to use Docker to set up a container that will compile all the tools for cross-compiling ROS and all of its dependencies. Go to https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository to install it for your Linux distribution.

1. Add your user to docker group
```
$ sudo usermod -aG docker $USER
$ newgrp
```

2. Install Qemu software and other prerequisites
```
$ sudo apt install -y qemu-user-static sshpass python-vcstool
```

### Build ROS System on Docker  (Run only the fist time per host computer)

```
make system
```

Caution!!! It will take more than a few hours !! So for JSK users, download the `arm64v8_System.tgz` archive file from [here](https://drive.google.com/drive/u/2/folders/1SBA9oAwjfD84yRFEB-jsCH1m5Q8eEGSK) and extract under `jsk_unitree_robot/cross/` directory before run `make` command.

Run following command to copy ROS1 base sytem to Go1 onboard computer. This should be done only in the first time. So normally user do not have to run this command
```
./install.sh -p 123 -d System
```

### Build `jsk_unitree_robot` software on Docker

You can build your current `jsk_unitree_robot` workspace on Docker environment which is ready to copy into Go1 onboard computer.

```
make user
```

To copy your software to Go1 onboard computer, run following command
```
./install.sh -p 123
```

## Detailed information

### Procedures not included in `make` process

#### Setup CUDA environment

nano3(192.168.123.15) of Pro robot does not contain CUDA environment. Please look at `pro_nano2_install_scripts` directory.

#### copy `rosdep` cache file

nano2(192.168.123.14) of Edu robot does not contain `rosdep` cache file, thus `app_manager.launch` outputs following warnings.

```
the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
[INFO] [1631952867.412752]: Loading from plugin definitions
[WARN] [1631952867.416907]: No applist directory found.
[INFO] [1631952867.426196]: Using apps for platform 'go1'
[INFO] [1631952867.441346]: Starting app manager for robot
[INFO] [1631952867.467832]: Waiting for foreign master [http://localhost:11313] to come up...
[INFO] [1631952867.491849]: Foreign master is available
```
and you can also find find following error.
```
rospack export --lang=app_manager --attrib=app_manager jsk_unitree_startup
[rospack] Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
```

To fix this problem, go to `pro_nano2_install_scripts` directory and copy cache files.
```
cd pro_nano2_install_scripts
./copy.sh
```

#### List of software installed in Docker environment

Create List of ROS packages to be installed
```
ssh pi@192.168.123.161 'source /opt/ros/melodic/setup.bash; rospack list' |tee 161-list.txt
ssh unitree@192.168.123.13 'source /opt/ros/melodic/setup.bash; rospack list' |tee 13-list.txt
ssh unitree@192.168.123.14 'source /opt/ros/melodic/setup.bash; rospack list' |tee 14-list.txt
ssh unitree@192.168.123.15 'source /opt/ros/melodic/setup.bash; rospack list' |tee 15-list.txt
cat 161-list.txt 13-list.txt 14-list.txt 15-list.txt  | sort | uniq -c | sort | egrep "^.*4" | sed 's/^\s*4\s*\(\S*\)\s.*$/ros-melodic-\1/' | sed 's/_/-/g' | tee ros-packages.txt
```

Create List of Debian packages to be installed
```
ssh pi@192.168.123.161 'dpkg --get-selections' | tee 161-select.txt
ssh unitree@192.168.123.13 'dpkg --get-selections' | tee 13-select.txt
ssh unitree@192.168.123.14 'dpkg --get-selections' | tee 14-select.txt
ssh unitree@192.168.123.15 'dpkg --get-selections' | tee 15-select.txt
cat 13-select.txt 14-select.txt 161-select.txt | sort | uniq -c | egrep "^\s+3\s" | sed 's/\s\s*/ /g' | cut -f 3 -d\  | sed 's/:arm64$//' | tee deb-packages.txt
```

## Known Issues

### Running python3

Since `st-000-ros1.bash` set PYTHONPATH and we installed `python-futures` via pip, It breaks python3 execution.
When you run python3, you need to unset PYTHONPATH, i.e `PYTHONPATH= vcs`

### Build time

Compile all System packages on aarch64 takes long time, It will take a whole day. You'd metter to obtain `arm64v8_System` directory from someone else.



### Development

On development phase, users are expected to develop sofoware on remote machine. All codes are expected to add in jsk_unitree_startup package.

### Deployment

You can send all development files to robot and start them on boot time.

```
make user
make install
```
