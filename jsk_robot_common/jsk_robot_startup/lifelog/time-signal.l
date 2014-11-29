#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(load "package://pr2eus/speak.l")
(ros::roseus "tiem_signal")

(ros::advertise "robotsound" sound_play::SoundRequest)

(defun speak-time
  (hour)
  (cond
   ((equal hour 0)
    (speak-en "The time is 12 PM" :google t)
    )
   ((<= hour 12)
    (speak-en (format nil "The time is ~A AM" hour) :google t)
   )
   (t
    (speak-en (format nil "The time is ~A PM" (- hour 12)) :google t)
)))

(defun publish-time-signal
  (music-file music-length hour)
  (ros::publish "robotsound"
		(instance sound_play::SoundRequest :init
			  :sound sound_play::SoundRequest::*play_file*
			  :command sound_play::SoundRequest::*play_once*
			  :arg music-file
			  ))
  (unix::usleep (* music-length 1000))
  (speak-time hour)
)

(defun start-time-signal
  (hour)
  (ros::ros-info "start time signal")
  (let (music-list
	music-index
	music)

    (setq music-list
	  (list
	   ;;http://musicmaterial.jpn.org/jingle_top.htm
	   (list "http://musicmaterial.jpn.org/loop/jingle_logo_001.mp3" 4000)
	   (list "http://musicmaterial.jpn.org/loop/jingle_logo_002.mp3" 6000)
	   (list "http://musicmaterial.jpn.org/loop/jingle_logo_003.mp3" 9000)
	   ;;http://www.hmix.net/music_gallery/sound_effect.htm
	   (list "http://www.hmix.net/music/n/n12.mp3" 8000)
	   (list "http://www.hmix.net/music/c/c11.mp3" 12000)
	   (list "http://www.hmix.net/music/d/d8.mp3" 10000)
	   (list "http://www.hmix.net/music/s/s3.wma" 7000)
	   ))
    (setq music-index (random (length music-list)))
    (setq music (elt music-list music-index))
    (publish-time-signal (elt music 0) (elt music 1) hour))
)

(defun main
  ()
  (setq last-speak-hour (elt (unix:localtime) 2))

  (do-until-key
   (let* ((now (unix:localtime))
	  (sec (elt now 0))
	  (min (elt now 1))
	  (hour (elt now 2)))
     (unless (equal hour last-speak-hour)
       (start-time-signal hour)
       (setq last-speak-hour hour)
       )
     (unix::sleep (- 60 sec))
     )))

(main)
