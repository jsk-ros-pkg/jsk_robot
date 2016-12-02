#!/usr/bin/env python

import os

import rospy
import roslib

roslib.load_manifest("jsk_pr2_startup")

##
import std_srvs
import std_msgs

##
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton
from python_qt_binding.QtCore import Qt, QTimer

class ImageSnapShotGUI(Plugin):
    def __init__(self, context):
        super(ImageSnapShotGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ImageSnapShotGUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        self._toolbar = QToolBar()
        self._toolbar.addWidget(QLabel('ImageSnapShot'))

        # Create a container widget and give it a layout
        self._container = QWidget()
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)
        self._layout.addWidget(self._toolbar)

        # Add a button for ....
        self._go_button = QPushButton('Head')
        self._go_button.clicked.connect(self._head)
        self._layout.addWidget(self._go_button)

        self._clear_button = QPushButton('L arm')
        self._clear_button.clicked.connect(self._larm)
        self._layout.addWidget(self._clear_button)

        self._clear_button = QPushButton('R arm')
        self._clear_button.clicked.connect(self._rarm)
        self._layout.addWidget(self._clear_button)

        # self._step_run_button.setStyleSheet('QPushButton {color: black}')
        context.add_widget(self._container)

        self._head_pub = rospy.Publisher('/head_snap/snapshot', std_msgs.msg.Empty)
        self._lhand_pub = rospy.Publisher('/lhand_snap/snapshot', std_msgs.msg.Empty)
        self._rhand_pub = rospy.Publisher('/rhand_snap/snapshot', std_msgs.msg.Empty)

    def _head(self):
        #go = rospy.ServiceProxy('/head_snap/snapshot', std_srvs.srv.Empty)
        #go()
        self._head_pub.publish(std_msgs.msg.Empty())
    def _larm(self):
        #go = rospy.ServiceProxy('/lhand_snap/snapshot', std_srvs.srv.Empty)
        #go()
        self._lhand_pub.publish(std_msgs.msg.Empty())
    def _rarm(self):
        #go = rospy.ServiceProxy('/rhand_snap/snapshot', std_srvs.srv.Empty)
        #go()
        self._rhand_pub.publish(std_msgs.msg.Empty())

    def shutdown_plugin(self):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
