#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
from datetime import datetime
import json
import rospy
import sys
import urllib2

from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal


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
        api_key_file = rospy.get_param(
            '~api_key_file', '/var/lib/robot/openweathermap_api_key.txt')
        with open(api_key_file, 'r') as f:
            self.appid = f.read().split('\n')[0]

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
        speech_text = str(self.now_hour) + 'じです。'
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
        if self.now_hour in [0, 7, 12, 19]:
            try:
                forecast_text = self._get_weather_forecast(lang='ja')
                speech_text += forecast_text
            except Exception as e:
                rospy.logerr(e)
        rospy.logdebug(speech_text)
        self.speak(self.client_jp, speech_text, lang='jp')

    def speak_en(self):
        speech_text = self._get_text(self.now_hour)
        # time signal
        if self.now_hour == 0:
            speech_text += " Let's go home."
        if self.now_hour == 12:
            speech_text += " Let's go to lunch."
        if self.now_hour == 19:
            speech_text += " Let's go to dinner."

        # weather forecast
        if self.now_hour in [0, 7, 12, 19]:
            try:
                forecast_text = self._get_weather_forecast(lang='en')
                speech_text += forecast_text
            except Exception as e:
                rospy.logerr(e)
        rospy.logdebug(speech_text)
        self.speak(self.client_en, speech_text)

    def _get_text(self, hour):
        if hour == 0:
            text = 'midnight'
        elif hour == 12:
            text = 'noon'
        else:
            if hour > 12:
                text = str(hour % 12) + ' P.M.'
            else:
                text = str(hour % 12) + ' A.M.'
        text = "It's " + text + "."
        return text

    def _get_weather_forecast(self, lang='en'):
        url = 'http://api.openweathermap.org/data/2.5/weather?q=tokyo&lang={}&units=metric&appid={}'.format(lang, self.appid)  # NOQA
        resp = json.loads(urllib2.urlopen(url).read())
        weather = resp['weather'][0]['description']
        # aques_talk replaces decimal point to "。" (e.g. 7.8度 -> 7。8ど)
        # So we use integer for temperature, humidity and wind speed.
        temp = int(resp['main']['temp'])
        humidity = int(resp['main']['humidity'])
        wind_speed = int(resp['wind']['speed'])
        forecast_text = ""
        if lang == 'ja':
            forecast_text = "現在、天気は" + weather + "、"
            forecast_text += "気温は{}度、".format(temp)
            forecast_text += "湿度は{}パーセントです。".format(humidity)
            forecast_text += "風速は{}メートル秒です。".format(wind_speed)
        else:
            forecast_text = " The weather is " + weather + " now."
            forecast_text += " The temperature is {} celsius,".format(temp)
            forecast_text += " and the humidity is {}%.".format(humidity)
            forecast_text += " The wind speed is {} meter per second.".format(wind_speed)
        return forecast_text


if __name__ == '__main__':
    rospy.init_node('time_signal')
    t = TimeSignal()
    t.speak_jp()
    t.speak_en()
