## :animated-speak `str` (naoqi_bridge [`kochigami-develop`])

### What is this?

Speak a sentence and animate it.  

### Parameters

`str`: speech sentence (str)

### Location

`naoqi_apps/launch/animated_speech.launch`  

### NAOqi API

[ALAnimatedSpeech::say](http://doc.aldebaran.com/2-5/naoqi/audio/alanimatedspeech-api.html#ALAnimatedSpeechProxy::say__ssCR)  

Related commit (not a PR to master) is [here](https://github.com/kochigami/naoqi_bridge/tree/add-animated-speak)

### Sample

```
; Robot speaks the sentence with some gesture.
send *ri* :animated-speak "Hello. Nice to meet you."
```
