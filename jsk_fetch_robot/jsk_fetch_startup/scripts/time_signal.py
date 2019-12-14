#!/usr/bin/env python
# -*- coding: utf-8 -*-

from datetime import datetime
from sound_play.msg import SoundRequestActionGoal
import json
import rospy
import sys
import urllib2


class TimeSignal(object):
    def __init__(self):
        self.pub = rospy.Publisher('/robotsound_jp/goal',
                                   SoundRequestActionGoal, queue_size=1)
        rospy.sleep(1)
        self.now_time = datetime.now()
        self.now_hour = self.now_time.hour
        self.day = self.now_time.strftime('%a')
        reload(sys)
        sys.setdefaultencoding('utf-8')
        self.citycode = '130010'  # 130010 is tokyo. See http://weather.livedoor.com/forecast/rss/primary_area.xml
        self.resp = json.loads(
            urllib2.urlopen('http://weather.livedoor.com/forecast/webservice/json/v1?city=%s'%self.citycode).read())

    def speak(self):
        sound_goal = SoundRequestActionGoal()
        sound_goal.goal_id.stamp = rospy.Time.now()
        sound_goal.goal.sound_request.sound = -3
        sound_goal.goal.sound_request.command = 1
        sound_goal.goal.sound_request.volume = 1.0
        sound_goal.goal.sound_request.arg2 = "jp"

        # time signal
        speech_text = str(self.now_hour) + '時です。'
        if (self.now_hour == 0):
            speech_text += '早く帰りましょう。'
        if (self.day == 'Mon') and (self.now_hour == 12):
            speech_text += 'そろそろ研究会です。'
        if (self.day == 'Tue') and (self.now_hour == 12):
            speech_text += 'そろそろ輪講です。'
        if (self.day == 'Tue') and (self.now_hour == 15):
            speech_text += '掃除の時間です。'
        if (self.day == 'Fri') and (self.now_hour == 14):
            speech_text += '創造輪講の時間です。'
        if (self.day == 'Fri') and (self.now_hour == 16):
            speech_text += '掃除の時間です。'

        # weather forecast
        if (self.now_hour == 0) or (self.now_hour == 19):
            speech_text += '今日の天気は' + self.resp['forecasts'][0]['telop'] + 'です。'

        rospy.loginfo('time signal')
        sound_goal.goal.sound_request.arg = speech_text
        self.pub.publish(sound_goal)


if __name__ == '__main__':
    rospy.init_node('time_signal')
    t = TimeSignal()
    t.speak()
