#!/usr/bin/env python
import rospy
import subprocess
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float64MultiArray

class ErrorMonitor:
    def __init__(self):
        self.error_count = 0
        self.threshold = 50
        self.last_restart = rospy.Time.now()
        self.restart_interval = rospy.Duration(10)
        self.sub = rospy.Subscriber('/rosout_agg', Log, self.callback)
        self.pub_rhand = rospy.Publisher('/rhand/joint_group_effort_controller/command', Float64MultiArray, queue_size=1)
        self.pub_lhand = rospy.Publisher('/lhand/joint_group_effort_controller/command', Float64MultiArray, queue_size=1)

    def restart_hands(self):
        now = rospy.Time.now()
        if (now - self.last_restart) < self.restart_interval:
            return

        rospy.logwarn("Restarting hand controllers...")

        nodes_to_kill = [
            '/lhand/dynamixel_general_control',
            '/rhand/dynamixel_general_control',
            '/lhand/controller_spawner',
            '/rhand/controller_spawner'
        ]

        for node in nodes_to_kill:
            try:
                subprocess.call(['rosnode', 'kill', node], timeout=3)
            except:
                pass

        rospy.sleep(3)

        left_tool = rospy.get_param('/lhand/current_tool', 'generic')
        right_tool = rospy.get_param('/rhand/current_tool', 'generic')
        subprocess.Popen([
            'roslaunch', 'dynamixel_detachable_hand', 'dual_hand.launch',
            'left_tool:={}'.format(left_tool),
            'right_tool:={}'.format(right_tool)
        ])

        rospy.sleep(10)

        msg = Float64MultiArray()
        msg.data = [0.92, 0.92]
        self.pub_rhand.publish(msg)
        self.pub_lhand.publish(msg)

        self.error_count = 0
        self.last_restart = now

    def callback(self, msg):
        if 'groupSyncRead getdata failed' in msg.msg:
            self.error_count += 1
            rospy.logwarn(f"Error count: {self.error_count}/{self.threshold}")
            if self.error_count >= self.threshold:
                self.restart_hands()

if __name__ == '__main__':
    rospy.init_node('dynamixel_connection_monitor')
    monitor = ErrorMonitor()
    rospy.spin()
