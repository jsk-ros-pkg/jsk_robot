#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import actionlib
import rospy

from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal

from sound_play.libsoundplay import SoundClient
from ros_speech_recognition import SpeechRecognitionClient


def main():

    rospy.init_node('person_lead_demo')

    speech_recognition_client = SpeechRecognitionClient()
    sound_client = SoundClient(sound_action='/robotsound_jp', sound_topic='/robotsound_jp')
    action_server_lead_to = actionlib.SimpleActionClient('~execute_behaviors', LeadPersonAction)

    node_list = rospy.get_param('~nodes', {})

    while not rospy.is_shutdown():

        rospy.loginfo('Asking package information')
        while not rospy.is_shutdown():
            sound_client.say('行き先を教えてください。', blocking=True)
            recogntion_result = speech_recognition_client.recognize()
            if len(recogntion_result.transcript) == 0:
                rospy.logerr('No matching node found from spoken \'{}\''.format(recogntion_result))
                sound_client.say('行き先がわかりませんでした', blocking=True)
                continue
            recognized_destination = recogntion_result.transcript[0]
            target_node_candidates = {}
            for node_id, value in node_list.items():
                if value.has_key('name_jp') and value['name_jp'].encode('utf-8') == recognized_destination:
                    target_node_candidates[node_id] = value
            if len(target_node_candidates) == 0:
                rospy.logerr('No matching node found from spoken \'{}\''.format(recogntion_result))
                sound_client.say('行き先がわかりませんでした', blocking=True)
            else:
                rospy.loginfo('target_node_candidates: {}'.format(target_node_candidates))
                break

        target_node_id = target_node_candidates.keys()[0]
        target_node_name_jp = node_list[target_node_id]['name_jp'].encode('utf-8')

        rospy.loginfo('executing behaviors to {}'.format(target_node_name_jp))
        action_server_lead_to.send_goal_and_wait(LeadPersonGoal(target_node_id=target_node_id))


if __name__ == '__main__':
    main()
