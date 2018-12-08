## :set-language `language` (naoqi_driver [`master`])

### What is this?

Set a language which a robot speaks.  

### Parameters

`language`: language (str)  

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALTextToSpeech::setLanguage](http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeechProxy::setLanguage__ssCR)  

Related PR is [here](https://github.com/ros-naoqi/naoqi_driver/pull/87)

### Sample

```
send *ri* :set-language "Japanese" ; "English"
```
