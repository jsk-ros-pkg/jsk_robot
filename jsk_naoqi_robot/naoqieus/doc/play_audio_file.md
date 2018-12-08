## :play-audio-file `file` (naoqi_driver [`kochigami-develop`])

### What is this?

Play audio file inside a robot.  

### Parameters

`file`: path to your audio file (str)  
Please locate mp3, wav file under `/home/nao/`. Then, set a path to your audio file to `file`.  

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALAudioPlayer::playFile](http://doc.aldebaran.com/2-5/naoqi/audio/alaudioplayer-api.html#ALAudioPlayerProxy::playFile__ssCR)  

Related PR is [here](https://github.com/ros-naoqi/naoqi_driver/pull/109)

### Sample

This is an example of how to play test.mp3 file from `/home/nao/audio_file/test.mp3` path.  

```
; ssh nao@<robot IP> 
; then, create audio_file folder and put test.mp3 file
; scp test.mp3  nao@<robot IP>:/home/nao/audio_file/

send *ri* :play-audio-file "/audio_file/test.mp3"
```
