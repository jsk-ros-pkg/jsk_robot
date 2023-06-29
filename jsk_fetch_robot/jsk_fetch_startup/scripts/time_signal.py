#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
from datetime import datetime
import json
import rospy
import sys
import urllib2

from dynamic_reconfigure.server import Server
from jsk_fetch_startup.cfg import TimeSignalConfig as Config
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal


class TimeSignal(object):
    def __init__(self, volume=1.0):
        self.client_en = actionlib.SimpleActionClient(
            '/sound_play', SoundRequestAction)
        self.client_jp = actionlib.SimpleActionClient(
            '/robotsound_jp', SoundRequestAction)
        self.now_time = datetime.now()
        self.now_date = self.now_time.date()
        self.now_hour = self.now_time.hour
        self.now_minute = self.now_time.minute
        self.day = self.now_time.strftime('%a')
        self.volume = volume
        self.srv = Server(Config, self.config_callback)
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
        sound_goal.sound_request.volume = self.volume
        if lang is not None:
            sound_goal.sound_request.arg2 = lang
        sound_goal.sound_request.arg = speech_text
        client.send_goal(sound_goal)
        client.wait_for_result()
        return client.get_result()

    def speak_jp(self):
        # time signal
        speech_text = self._get_time_text(
            self.now_date, self.now_hour, self.now_minute,
            lang='ja')
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
        if self.day == 'Fri' and self.now_hour == 15:
            speech_text += '創造輪講の時間です。'

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
        speech_text = self._get_time_text(
            self.now_date, self.now_hour, self.now_minute,
            lang='en')
        # time signal
        if self.now_hour == 0:
            speech_text += " Let's go home."
        if self.now_hour == 12:
            speech_text += " Let's go to lunch."
        if self.now_hour == 19:
            speech_text += " Let's go to dinner."
        if self.day == 'Mon' and self.now_hour == 12:
            speech_text += ' The lab meeting will start soon.'
        if self.day == 'Tue' and self.now_hour == 12:
            speech_text += ' The lab meeting will start soon.'
        if self.day == 'Tue' and self.now_hour == 15:
            speech_text += ' It is cleaning time now.'
        if self.day == 'Fri' and self.now_hour == 15:
            speech_text += ' Rinko will start soon. '

        # weather forecast
        if self.now_hour in [0, 7, 12, 19]:
            try:
                forecast_text = self._get_weather_forecast(lang='en')
                speech_text += forecast_text
            except Exception as e:
                rospy.logerr(e)
        rospy.logdebug(speech_text)
        self.speak(self.client_en, speech_text)

    def _get_time_text(self, date, hour, minute, lang='en'):
        if lang == 'ja':
            if hour == 0 and minute == 0:
                time_text = '{}年{}月{}日'.format(
                    date.year, date.month, date.day)
            elif hour == 12 and minute == 0:
                time_text = '正午'
            else:
                time_text = '{}時'.format(hour)
                if minute != 0:
                    time_text += '{}分'.format(minute)
            time_text += 'です。'
        else:
            if hour == 0 and minute == 0:
                time_text = date.strftime('%Y %B %d')
            elif hour == 12 and minute == 0:
                time_text = 'noon'
            else:
                time_text = str(hour % 12)
                if minute != 0:
                    time_text += ' {}'.format(minute)
                if hour > 12:
                    time_text += ' P.M.'
                else:
                    time_text += ' A.M.'
            time_text = "It's {}.".format(time_text)
        return time_text

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

    def _set_volume(self, volume):
        '''
        Set speak volume between 0.0 and 1.0
        '''
        volume = min(max(0.0, volume), 1.0)
        if self.volume != volume:
            self.volume = volume
            rospy.loginfo("time_signal's volume was set to {}".format(
                self.volume))

    def config_callback(self, config, level):
        self._set_volume(config.volume)
        return config


if __name__ == '__main__':
    rospy.init_node('time_signal')
    t = TimeSignal()
    t.speak_jp()
    t.speak_en()
