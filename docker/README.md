# docker for jsk_robot

## Usage

```
# melodic
docker build --build-arg ROS_DISTRO=melodic --build-arg UBUNTU_VERSION=bionic -t jskrobotics/jsk_robot:melodic-latest .
# kinetic
docker build --build-arg ROS_DISTRO=kinetic --build-arg UBUNTU_VERSION=xenial -t jskrobotics/jsk_robot:kinetic-latest .
```
