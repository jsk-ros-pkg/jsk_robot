#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from spot_msgs.srv import Dock

class SpotDockWithID(object):

    def __init__(self):

        self._dock_id = int(rospy.get_param('~dock_id', 1))

        self._server = rospy.Service('/spot/dock_fixed_id', Trigger, self._cb)
        self._client = rospy.ServiceProxy('/spot/dock', Dock)

    def _cb(self, req):

        resp = self._client(self._dock_id)

        rospy.loginfo("Call /spot/dock(dock_id={}) returns {}".format(self._dock_id, resp))
        return TriggerResponse(success=resp.success, message=resp.message)

def main():
    rospy.init_node('spot_dock_with_id')
    end_effector_to_joy = SpotDockWithID()
    rospy.spin()


if __name__ == '__main__':
    main()
