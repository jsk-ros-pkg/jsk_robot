#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy

from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal


class PersonalUse(object):
    def __init__(self):
        duration = rospy.get_param('~duration', 10.0)
        self.warning = rospy.get_param('~warning', True)
        warning_duration = rospy.get_param(
            '~warning_duration', 60.0 * 15)
        self.client_jp = actionlib.SimpleActionClient(
            '/robotsound_jp', SoundRequestAction)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)

        self.user_name = rospy.get_param(
            '/app_manager/running_user_name', None)
        self.robot_name = rospy.get_param('/robot/name', None)

        self.warning_limit = warning_duration / duration
        self.warning_count = 0

        if self.user_name:
            rospy.loginfo(
                "{} is running personal use app.".format(self.user_name))
            sentence = "{}に{}が実行ユーザとして登録されました．".format(
                self.robot_name, self.user_name)
            sentence += "{}が{}の使用を開始しました．".format(
                self.user_name, self.robot_name)
        else:
            rospy.logerr("failed to get /app_manager/running_user_name param")
            sentence = "実行ユーザ登録に失敗しました．"
        self._speak(self.client_jp, sentence, 'jp')

    def _timer_cb(self, event):
        user_name = rospy.get_param('/app_manager/running_user_name', None)
        self.warning_count = self.warning_count + 1
        if self.user_name:
            # overwriting user
            if user_name and self.user_name != user_name:
                rospy.logerr(
                    'running user has changed: {} -> {}'
                    .format(self.user_name, user_name))
                rospy.logerr(
                    'overwrite running user: {} -> {}'
                    .format(user_name, self.user_name))
                rospy.set_param(
                    '/app_manager/running_user_name', self.user_name)
                sentence = "{}に{}が実行ユーザとして上書き登録されました．".format(
                    self.robot_name, user_name)
                sentence += "{}に{}を再度実行ユーザとして上書き登録しました．".format(
                    self.robot_name, self.user_name)
                self._speak(self.client_jp, sentence, 'jp')
            # deleting user
            elif not user_name:
                rospy.logerr(
                   'running user has removed: {}'.format(self.user_name))
                rospy.logerr(
                    'set running user: {}'.format(self.user_name))
                sentence = "{}から{}の実行ユーザ登録が削除されました．".format(
                    self.robot_name, self.user_name)
                sentence += "{}に{}を再度実行ユーザとして登録しました．".format(
                    self.robot_name, self.user_name)
                rospy.set_param(
                    '/app_manager/running_user_name', self.user_name)
                self._speak(self.client_jp, sentence, 'jp')

            if self.warning and self.warning_count >= self.warning_limit:
                rospy.loginfo(
                    "{} is running personal use app.".format(self.user_name))
                sentence = "{}が{}を使用しています．".format(
                    self.user_name, self.robot_name)
                self._speak(self.client_jp, sentence, 'jp')
        else:
            rospy.logerr('/app_manager/running_user_name is not set')
            sentence = "実行ユーザが登録されていません．"
            if user_name:
                self.user_name = user_name
                rospy.set_param(
                    '/app_manager/running_user_name', self.user_name)
                rospy.logerr(
                    'set running user: {}'.format(self.user_name))
                sentence += "{}を実行ユーザとして登録しました．".format(self.user_name)
            self._speak(self.client_jp, sentence, 'jp')
        self.warning_count = self.warning_count % self.warning_limit

    def _speak(self, client, speech_text, lang=None):
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


if __name__ == '__main__':
    rospy.init_node('personal_use')
    app = PersonalUse()
    rospy.spin()
