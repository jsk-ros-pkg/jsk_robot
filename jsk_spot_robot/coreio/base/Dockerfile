#####
##  Setup development environment
#####
##### Use sha256 as of 2023/03/16
FROM arm64v8/ros:melodic-robot@sha256:c0a0a8a25dc822ea48b40c45d74b267ddf4dbe22b8accc9fd66bb146c34eb974 AS base_build
ENV DEBIAN_FRONTEND noninteractive

# development tools
RUN apt-get update && apt install -y -q python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-pip python3-empy python3-opencv python3-sip-dev python3-defusedxml cython3

# network tools
RUN apt-get update && apt install -y -q net-tools iputils-ping dnsutils traceroute curl

# ROS packages
RUN apt-get update && apt install -y -q ros-melodic-catkin ros-melodic-vision-msgs

# setup user space
RUN sed -i 's@%sudo\s*ALL=(ALL:ALL)\s*ALL@%sudo ALL=(ALL:ALL) NOPASSWD: ALL@' /etc/sudoers 
RUN useradd -ms /bin/bash spot
RUN echo spot:spot | chpasswd
RUN adduser spot sudo
USER spot
WORKDIR /home/spot
RUN rosdep update -y

# get dummy workspace to install dpeendent packages
RUN mkdir -p /tmp/ws/src/jsk_robot/jsk_spot_robot/jsk_spot_startup /tmp/ws/src/jsk_robot/jsk_spot_robot/spoteus
COPY src/jsk_robot/jsk_spot_robot/jsk_spot_startup/package.xml /tmp/ws/src/jsk_robot/jsk_spot_robot/jsk_spot_startup/
COPY src/jsk_robot/jsk_spot_robot/spoteus/package.xml /tmp/ws/src/jsk_robot/jsk_spot_robot/spoteus/
COPY src/jsk_robot/jsk_spot_robot/requirements.txt /tmp/ws/src/jsk_robot/jsk_spot_robot/

# install pip3 dependencies
RUN pip3 install pip==21.3.1
RUN pip3 install wrapt==1.12.1 six==1.15.0 PyJWT==2.0.0 numpy==1.19.4 grpcio==1.34.0

# install python requirements
RUN pip3 install -r /tmp/ws/src/jsk_robot/jsk_spot_robot/requirements.txt

# rosdep install
RUN rosdep install -y -r --from-paths /tmp/ws/src --ignore-src --rosdistro melodic || echo "OK"

#####
##  Initialize catkin workspace after wstool merge and update with jsk_spot_robot/jsk_spot_driver.rosinstall
#####
FROM base_build AS pre_build

# setup catkin develeopment environment with mounted workspace
COPY package.xml.tar /tmp/
RUN tar -C /tmp/ws/ -xvf /tmp/package.xml.tar
RUN rosdep install -y -r --from-paths /tmp/ --ignore-src --rosdistro melodic || echo "OK"
##
## 'RUN --mount' called everytime when conttnes of spot/ws changed, so use 'COPY'
#RUN --mount=type=bind,source=/,target=/home/spot/ws cd ws && rosdep install -y -r --from-paths src --ignore-src --rosdistro melodic || echo "OK"


#####
##  User preferences for developent tools
#####
FROM pre_build AS dev_build
RUN sudo apt install -y -q python-catkin-tools less emacs vim

RUN sudo apt install -y -q bluez bluez-tools

RUN echo "#!/bin/bash"	 > /home/spot/ros_entrypoint.sh && \
    echo "set -e"	>>  /home/spot/ros_entrypoint.sh && \
    echo "source /opt/ros/melodic/setup.bash --"	>>  /home/spot/ros_entrypoint.sh && \
    echo "source /home/spot/ws/devel/setup.bash --"	>>  /home/spot/ros_entrypoint.sh && \
    echo "sudo service dbus start"	>>  /home/spot/ros_entrypoint.sh && \
    echo "sudo bluetoothd &"	>>  /home/spot/ros_entrypoint.sh && \
    echo "exec \"\$@\""	>>  /home/spot/ros_entrypoint.sh

RUN chmod u+x /home/spot/ros_entrypoint.sh

### FIXME ros_entrypoint did not work with roscd
RUN echo "source /opt/ros/melodic/share/rosbash/rosbash" >> ~/.bashrc

ENV TZ=Asia/Tokyo

ENTRYPOINT ["/home/spot/ros_entrypoint.sh"]
CMD ["bash"]
