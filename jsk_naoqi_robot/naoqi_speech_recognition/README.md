# naoqi_speech_recognition

Use ros_speech_recognition with NAO microphone.

https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/ros_speech_recognition

## Setup Environment

```
cd ~/catkin_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty
cd ..
rosdep install -y -r --from-paths src --ignore-src
catkin build ros_speech_recognition
source devel/setup.bash
```

## How to use
```
roslaunch naoqi_speech_recognition naoqi_speech_recognition.launch audio_org:=/nao_robot/naoqi_driver/audio
```
