#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
from datetime import datetime
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal
import json
import rospy
import sys
import urllib2


class TimeSignal(object):
    def __init__(self):
        self.client_en = actionlib.SimpleActionClient(
            '/sound_play', SoundRequestAction)
        self.client_jp = actionlib.SimpleActionClient(
            '/robotsound_jp', SoundRequestAction)
        self.now_time = datetime.now()
        self.now_hour = self.now_time.hour
        self.day = self.now_time.strftime('%a')
        reload(sys)
        sys.setdefaultencoding('utf-8')
        self.citycode = '130010'  # 130010 is tokyo. See http://weather.livedoor.com/forecast/rss/primary_area.xml
        self.resp = json.loads(
            urllib2.urlopen('http://weather.livedoor.com/forecast/webservice/json/v1?city=%s'%self.citycode).read())

    def speak(self, client, speech_text, lang=None):
        client.wait_for_server(timeout=rospy.Duration(1.0))
        sound_goal = SoundRequestGoal()
        sound_goal.sound_request.sound = -3
        sound_goal.sound_request.command = 1
        sound_goal.sound_request.volume = 1.0
        if lang is not None:
            sound_goal.sound_request.arg2 = lang
        sound_goal.sound_request.arg = speech_text
        client.send_goal(sound_goal)
        client.wait_for_result()
        return client.get_result()

    def speak_jp(self):
        # time signal
        speech_text = str(self.now_hour) + '時です。'
        if self.now_hour == 0:
            speech_text += '早く帰りましょう。'
        if self.now_hour == 12:
            speech_text += '昼食の時間です。'
        if self.now_hour == 19:
            speech_text += '夕食の時間です。'
        if self.day == 'Mon' and self.now_hour == 12:
            speech_text += 'そろそろ研究会です。'
        if self.day == 'Tue' and self.now_hour == 12:
            speech_text += 'そろそろ輪講です。'
        if self.day == 'Tue' and self.now_hour == 15:
            speech_text += '掃除の時間です。'
        if self.day == 'Fri' and self.now_hour == 14:
            speech_text += '創造輪講の時間です。'
        if self.day == 'Fri' and self.now_hour == 16:
            speech_text += '掃除の時間です。'

        # weather forecast
        if self.now_hour == 0 or self.now_hour == 19:
            speech_text += '今日の天気は' + self.resp['forecasts'][0]['telop'] + 'です。'

        self.speak(self.client_jp, speech_text, lang='jp')


    def speak_en(self):
        speech_text = self._get_text(self.now_hour)
        # time signal
        if self.now_hour == 0:
            speech_text += " Let's go home."
        if self.now_hour == 12:
            speech_text += " Let's go to lunch."
        self.speak(self.client_en, speech_text)

    def _get_text(self, hour):
       if hour == 0:
          text = 'midnight'
       elif hour == 12:
          text = 'noon'
       else:
          if hour > 12:
              text = str(hour % 12) + ' PM'
          else:
              text = str(hour % 12) + ' AM'
       text = "It's " + text + "."
       return text 


if __name__ == '__main__':
    rospy.init_node('time_signal')
    t = TimeSignal()
    t.speak_jp()
    t.speak_en()
