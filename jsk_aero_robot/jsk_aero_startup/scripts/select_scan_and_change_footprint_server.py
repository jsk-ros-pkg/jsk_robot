#!/usr/bin/env python
import rospy

import dynamic_reconfigure.client
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, EmptyResponse
from topic_tools.srv import MuxSelect, MuxSelectResponse

class TransportDemoServer:

  flag = "normal"

  def __init__(self):
      rospy.init_node('change_footprint_server')
      self.scan_service_call = rospy.ServiceProxy("/sel_scan_select", MuxSelect)
      self.reset_costmap_service_call = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
      self.global_costmap_client = dynamic_reconfigure.client.Client("/move_base/global_costmap", timeout=30, config_callback=self.global_costmap_callback)
      self.local_costmap_client = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=30, config_callback=self.local_costmap_callback)
      s = rospy.Service('/transport_demo/select', Trigger, self.callback)
      rospy.spin()

  def callback(self, _):
      req = TriggerResponse()
      req.success = True
      if self.flag == "normal":
        #select scan
        self.scan_service_call("/scan_filtered")
        #change footprint
        self.global_costmap_client.update_configuration({"footprint":[[-0.35, -0.25],[-0.35, 0.25],[0.35, 0.25],[0.35, 0.35],[0.9, 0.35],[0.9, -0.35],[0.35, -0.35],[0.35, -0.25]]})
        self.local_costmap_client.update_configuration({"footprint":[[-0.35, -0.25],[-0.35, 0.25],[0.35, 0.25],[0.35, 0.35],[0.9, 0.35],[0.9, -0.35],[0.35, -0.35],[0.35, -0.25]]})
        #reset costmap
        self.reset_costmap_service_call()
        #change flag
        self.flag = "filtered"
        req.message = "filtered"
      elif self.flag == "filtered":
        #select scan
        self.scan_service_call("/scan")
        #reset costmap
        self.reset_costmap_service_call()
        #change footprint
        self.global_costmap_client.update_configuration({"footprint":[[-0.35, -0.25],[-0.35, 0.25],[0.35, 0.25],[0.35, -0.25]]})
        self.local_costmap_client.update_configuration({"footprint":[[-0.35, -0.25],[-0.35, 0.25],[0.35, 0.25],[0.35, -0.25]]})
        self.flag = "normal"
        req.message = "normal"
      return req

  def global_costmap_callback(self, _):
      pass

  def local_costmap_callback(self, _):
      pass

if __name__ == "__main__":
    srv = TransportDemoServer()
