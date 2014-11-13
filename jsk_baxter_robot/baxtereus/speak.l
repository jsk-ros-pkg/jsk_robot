(ros::load-ros-manifest "sound_play")

(defun speak-en (word)
  (let ((req (instance sound_play::SoundRequest :init :sound sound_play::SoundRequest::*SAY* :command sound_play::SoundRequest::*PLAY_ONCE* :arg word)) ret)
    (ros::advertise "/robotsound" sound_play::SoundRequest)
    (ros::publish "/robotsound" req)
    )
  )


(defun beep ()
  (let ((req (instance sound_play::SoundRequest :init :sound sound_play::SoundRequest::*PLAY_START* :command sound_play::SoundRequest::*PLAY_ONCE*)) ret)
    (ros::advertise "/robotsound" sound_play::SoundRequest)
    (ros::publish "/robotsound" req)
    )
  )
