## :get-language (naoqi_driver [`master`])

### What is this?

Get a language which a robot speaks.  

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALTextToSpeech::getLanguage](http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeechProxy::getLanguage)  

### Sample

```
send *ri* :get-language
"Japanese"
```
