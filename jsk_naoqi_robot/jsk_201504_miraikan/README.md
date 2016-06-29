jsk_201504_miraikan
===================

how to run demo
--------------

For Japanese and English demo,
Please confirm the language (Japanese/ English). You can change it from Pepper's tablet -> setup. (Note: This function is under construction.)

For Chinese demo,
please move mp3 files in file directory you want Pepper to speak under ``` /home/nao/audio_file . If not, please make ```audio_file``` folder in ```/home/nao/```. You can use ```scp <file> nao@<Pepper IP>:/home/nao/audio_file/```.

```
roslaunch jsk_pepper_startup jsk_pepper_startup.launch
roslaunch nao_interaction_launchers nao_audio_interface.launch nao_ip:=<Pepper's IP>
roseus mirai-demo-2015-0413.l
demo (:en, :chi)
```