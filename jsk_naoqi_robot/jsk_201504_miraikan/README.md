jsk_201504_miraikan
===================

What is this?
-------------

Pepper@JSK introduces him/ herself.  
Pepper can speak three languages (Japanese, Chinese and English).  

How to run demo?
----------------

This demo was tested with Pepper (NAOqi version: 2.5.5.5) and ROS kinetic.

- (Optional) Preparation for Chinese demo

Please move mp3 files under `/jsk_201504_miraikan/file` to `/home/nao/audio_file` inside Pepper's PC.  
You can log in to Pepper's PC by `ssh nao@<Pepper's IP>`.  
If there is no `audio_file` folder, please make it under `/home/nao/`.  
You can move mp3 files as follows:

```
roscd jsk_201504_miraikan
cd file
scp <file name>  nao@<Pepper IP>:/home/nao/audio_file/
```

- Please execute these for all languages

```
roslaunch jsk_pepper_startup jsk_pepper_startup.launch
roseus mirai-demo-2015-0413.l
demo ; Japanese
demo :en ; English
demo :chi ; Chinese
```