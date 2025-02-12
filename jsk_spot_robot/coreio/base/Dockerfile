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

# development tools
RUN apt-get update && apt install -y -q python-catkin-tools ros-melodic-catkin-virtualenv

# editors
RUN apt-get update && apt install -y -q less emacs vim

# bluetooth
RUN apt-get update && apt install -y -q bluez bluez-tools

# ROS packages
RUN apt-get update && apt install -y -q ros-melodic-catkin ros-melodic-vision-msgs

# setup user space
RUN sed -i 's@%sudo\s*ALL=(ALL:ALL)\s*ALL@%sudo ALL=(ALL) NOPASSWD: ALL@' /etc/sudoers
RUN cat /etc/sudoers
RUN useradd -ms /bin/bash spot
RUN echo spot:spot | chpasswd
RUN adduser spot sudo
USER spot
WORKDIR /home/spot
RUN rosdep update -y --include-eol-distros

# get dummy workspace to install dpeendent packages
# setup catkin develeopment environment with mounted workspace
RUN mkdir -p /tmp/ws
COPY package.tar /tmp/
RUN tar -C /tmp/ws/ -xvf /tmp/package.tar

# install pip3 dependencies
RUN sudo -H pip3 install pip==21.3.1
RUN sudo -H pip3 install wrapt==1.12.1 six==1.15.0 PyJWT==2.0.0 numpy==1.19.4 grpcio==1.34.0

# install python requirements
RUN sudo -H pip3 install -r /tmp/ws/src/jsk_robot/jsk_spot_robot/requirements.txt

# # rosdep install
# RUN sudo apt-get update && rosdep install -y -r --from-paths /tmp/ws/src --ignore-src --rosdistro melodic || echo "OK"

#####
##  Initialize catkin workspace after wstool merge and update with jsk_spot_robot/jsk_spot_driver.rosinstall
#####
FROM base_build AS pre_build

# setup catkin develeopment environment with mounted workspace
COPY package.tar /tmp/
RUN tar -C /tmp/ws/ -xvf /tmp/package.tar
RUN sudo apt-get update && rosdep install -y -r --from-paths /tmp/ --ignore-src --rosdistro melodic || echo "OK"
# rosdep install for python3
RUN sudo apt-get update && ROS_PYTHON_VERSION=3 rosdep install -y -r --skip-keys="python3-catkin-pkg python3-catkin-tools python3-rosinstall python3-rosdep" --from-paths /tmp/ --ignore-src --rosdistro melodic || echo "OK"

##
## 'RUN --mount' called everytime when conttnes of spot/ws changed, so use 'COPY'
#RUN --mount=type=bind,source=/,target=/home/spot/ws cd ws && rosdep install -y -r --from-paths src --ignore-src --rosdistro melodic || echo "OK"


#####
##  User preferences for developent tools
#####
FROM pre_build AS dev_build

# extra tools
RUN sudo apt-get update && sudo apt install -y -q tmux expect

RUN echo "#!/bin/bash"	 > /home/spot/ros_entrypoint.sh && \
    echo "set -e"	>>  /home/spot/ros_entrypoint.sh && \
    echo "source /opt/ros/melodic/setup.bash --"	>>  /home/spot/ros_entrypoint.sh && \
    echo "[ -e /home/spot/ws/install/setup.bash ] && source /home/spot/ws/install/setup.bash"	>>  /home/spot/ros_entrypoint.sh && \
    echo "sudo service dbus start"	>>  /home/spot/ros_entrypoint.sh && \
    echo "sudo bluetoothd &"	>>  /home/spot/ros_entrypoint.sh && \
    echo "exec \"\$@\""	>>  /home/spot/ros_entrypoint.sh

RUN chmod 0777 /home/spot/ros_entrypoint.sh

### FIXME ros_entrypoint did not work with roscd
RUN echo "source /opt/ros/melodic/share/rosbash/rosbash" >> ~/.bashrc

ENV TZ=Asia/Tokyo

ENTRYPOINT ["/home/spot/ros_entrypoint.sh"]
CMD ["bash"]

FROM dev_build AS user_build

ARG USER
ENV USER ${USER:-spot}
ARG UID
ENV UID ${UID:-1000}
ARG GID
ENV GID ${GID:-1000}
RUN sudo addgroup --gid ${GID} ${USER}
RUN sudo adduser --disabled-password --gecos '' --uid ${UID} --gid ${GID} ${USER}
RUN sudo adduser ${USER} sudo
USER ${USER}

RUN echo "#!/bin/bash"	 > /home/${USER}/ros_entrypoint.sh && \
    echo "set -e"	>>  /home/${USER}/ros_entrypoint.sh && \
    echo "source /opt/ros/melodic/setup.bash --"	>>  /home/${USER}/ros_entrypoint.sh && \
    echo "source /home/spot/ws/install/setup.bash --"	>>  /home/${USER}/ros_entrypoint.sh && \
    echo "[ -e /home/${USER}/ws/devel/setup.bash ] && source /home/${USER}/ws/devel/setup.bash --"	>>  /home/${USER}/ros_entrypoint.sh && \
    echo "exec \"\$@\""	>>  /home/${USER}/ros_entrypoint.sh

RUN chmod 0777 /home/${USER}/ros_entrypoint.sh

RUN echo "source /opt/ros/melodic/share/rosbash/rosbash" >> ~/.bashrc

## add your favorite package
RUN sudo apt install -y clisp

## FIXME @@HOME@@ is replaced by create_src_tree_tar in Makefile
ENTRYPOINT ["@@HOME@@/ros_entrypoint.sh"]
CMD ["bash"]
