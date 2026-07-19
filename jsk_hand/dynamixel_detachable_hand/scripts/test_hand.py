#!/usr/bin/env python3
import rospy
from hand_controller import LHandController, RHandController
import time

if __name__ == "__main__":
    rospy.init_node("test_hand_node")
    lh = LHandController()
    rh = RHandController()
    lh.open()
    time.sleep(1)
    lh.close()
    time.sleep(1)
    rh.close()
    time.sleep(1)
    rh.open()
    time.sleep(1)
