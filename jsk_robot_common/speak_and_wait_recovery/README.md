# Speak and Wait Recovery

This reovery behavior will enables a robot to speak a given text and wait for a given duration.
This will be useful when a robot stuck in a crowded environment.

<img src="https://user-images.githubusercontent.com/9410362/113511987-8644ec00-959d-11eb-8749-60842330e7bc.png" width="500px">

## speak_and_wait_recovery/SpeakAndWaitRecovery

### Parameters

- `~speak_text` (string, default: `Make way, Please.`)

Text which you are goint to make a robot say.

- `~duration_wait` (double, default: `5.0`)

Duration for which a robot will wait after saying the given text. (seconds)

- `~duration_timeout` (double, default: `1.0`)

Duration for tf looking up transformations. (seconds)

- `~sound_action` (string, default: `sound_play`)

Action name of sound_play used for Text-To-Speech.

### example

Please see demo.launch for example usage.
`navigation_stage` is required for demo.

```bash
roslaunch speak_and_wait_recovery demo.launch
```
