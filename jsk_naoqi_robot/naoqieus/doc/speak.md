## :speak `str` (naoqi_driver [`master`])

### What is this?

Speak a sentence.

### Parameters

`str`: speech sentence (str)

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALTextToSpeechProxy::say](http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeechProxy::say__ssCR)  

### Sample

```
send *ri* :speak "Hello. Nice to meet you."
```
