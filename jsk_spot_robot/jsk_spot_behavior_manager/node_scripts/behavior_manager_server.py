#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from jsk_spot_behavior_manager.behavior_manager_node import BehaviorManagerNode

def main():

    rospy.init_node('behavior_manager_node')
    node = BehaviorManagerNode()
    node.run()

if __name__ == '__main__':
    main()