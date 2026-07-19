#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dynamixel_general_hw.msg import DynamixelStateList
from sound_play.libsoundplay import SoundClient

TEMP_TOPIC_L = "/lhand/temperature"
TEMP_TOPIC_R = "/rhand/temperature"
IMAGE_TOPIC = "/dynamixel_temperature_guages"
TEMP_THRESHOLD = 50.0
TEMP_WARN_INTERVAL = 5.0

class TempVisualizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.sound = SoundClient()
        self.pub_l = rospy.Publisher(TEMP_TOPIC_L, Float32, queue_size=1)
        self.pub_r = rospy.Publisher(TEMP_TOPIC_R, Float32, queue_size=1)
        self.pub_img = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1)

        self.lhand_temps = {}
        self.rhand_temps = {}
        self.last_warn_time = {}

        rospy.Subscriber("/lhand/dynamixel_general_control/dynamixel_state",
                        DynamixelStateList, self.lhand_cb)
        rospy.Subscriber("/rhand/dynamixel_general_control/dynamixel_state",
                        DynamixelStateList, self.rhand_cb)

        rospy.Timer(rospy.Duration(0.5), self.viz_cb)

    def check_warn(self, name, temp):
        if temp >= TEMP_THRESHOLD:
            now = rospy.Time.now()
            if name not in self.last_warn_time or (now - self.last_warn_time[name]).to_sec() > TEMP_WARN_INTERVAL:
                self.sound.say("Warning, motor overheating")
                self.last_warn_time[name] = now
                rospy.logwarn(f"{name} temperature: {temp}C")

    def lhand_cb(self, msg):
        for s in msg.dynamixel_state:
            self.lhand_temps[s.name] = s.present_temperature
            f = Float32()
            f.data = float(s.present_temperature)
            self.pub_l.publish(f)
            self.check_warn(f"LHAND_{s.name}", s.present_temperature)

    def rhand_cb(self, msg):
        for s in msg.dynamixel_state:
            self.rhand_temps[s.name] = s.present_temperature
            f = Float32()
            f.data = float(s.present_temperature)
            self.pub_r.publish(f)
            self.check_warn(f"RHAND_{s.name}", s.present_temperature)

    def viz_cb(self, event):
        if not self.lhand_temps and not self.rhand_temps:
            return

        fig, ax = plt.subplots(figsize=(8, 4))

        names = []
        values = []
        colors = []

        for name, temp in self.lhand_temps.items():
            names.append(f"LHAND Motor")
            values.append(temp)
            colors.append('red' if temp >= TEMP_THRESHOLD else 'green')

        for name, temp in self.rhand_temps.items():
            names.append(f"RHAND Motor")
            values.append(temp)
            colors.append('red' if temp >= TEMP_THRESHOLD else 'green')

        ax.barh(names, values, color=colors)
        ax.set_xlabel('Temperature (C°)')
        ax.set_title('Dynamixel Temperature Monitor')
        ax.set_xlim(0, 100)
        ax.axvline(x=TEMP_THRESHOLD, color='orange', linestyle='--', linewidth=2)

        plt.tight_layout()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = buf.reshape(h, w, 3)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        plt.close(fig)

        img_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        self.pub_img.publish(img_msg)

if __name__ == "__main__":
    rospy.init_node("dxl_temperature_visualizer")
    rospy.sleep(1.0)
    viz = TempVisualizer()
    rospy.spin()
