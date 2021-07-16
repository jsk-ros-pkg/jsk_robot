#!/usr/bin/env python

import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
from audio_common_msgs.msg import AudioData


class AudioDataPublisher(object):

    def __init__(self):

        self.audiodata_pub = rospy.Publisher(rospy.get_param("~audio_topic", "/audio"), AudioData, queue_size=1)
        rospy.Subscriber(rospy.get_param("~audio_org", "/nao_robot/naoqi_driver/audio"), AudioBuffer, self.topic_cb)

    def topic_cb(self, msg):
        # Retrieve 1 channel audio data
        tmp = list(msg.data)
        tmp = tmp[0::len(msg.channelMap)]
        # Convert audio format: Int -> Buffer(Uint8)
        dataBuff = ""
        for i in range(0, len(tmp)):
            if tmp[i] < 0:
                tmp[i] = tmp[i]+65536
            dataBuff = dataBuff + chr(tmp[i] % 256)
            dataBuff = dataBuff + chr((tmp[i] - (tmp[i] % 256)) / 256)
        # Publish audio topic
        audio_data = AudioData()
        audio_data.data = dataBuff
        self.audiodata_pub.publish(audio_data)


if __name__ == '__main__':
    rospy.init_node('audio_data_publisher')
    AudioDataPublisher()
    rospy.spin()
