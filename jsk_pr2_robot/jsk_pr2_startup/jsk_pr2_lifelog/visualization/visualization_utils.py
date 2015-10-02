#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy
from transform_utils import TransformationUtils as T
import colorsys
import math

def distanceOfVector3(v1,v2):
    return math.sqrt((v1.x - v2.x) * (v1.x - v2.x)
                     + (v1.y - v2.y) * (v1.y - v2.y)
                     + (v1.z - v2.z) * (v1.z - v2.z))

class VisualizationUtils():
    marker_id = 0

    @classmethod
    def transformStampedArrayToLabeledArrayMarker(cls, tsdata_lst, fmt="%Y-%m-%d %H:%M:%S", zoffset=0.05, label_downsample=1, discrete=False):
        "[[transformStamped, meta],...] -> LineStrip / String"
        h = Header()
        h.stamp = rospy.Time(0) #rospy.Time.now()
        h.frame_id = "eng2" #t_first.child_frame_id
        res = []

        t_first = tsdata_lst[0][0]
        prev_t = t_first.transform.translation
        for ts, _ in tsdata_lst:
            m = Marker(type=Marker.ARROW,
                       action=Marker.ADD,
                       header=h,
                       id=cls.marker_id)
            cls.marker_id += 1
            t = ts.transform.translation
            dist = distanceOfVector3(prev_t, t) * 0.65
            rgb = colorsys.hsv_to_rgb(dist, 1.0, 1.0)
            m.points = [Point(x=prev_t.x,y=prev_t.y,z=prev_t.z),
                        Point(x=(prev_t.x + t.x) / 2.,y=(prev_t.y + t.y) /2.,z=(prev_t.z + t.z) /2.)]
            m.ns = "labeled_line"
            m.lifetime = rospy.Time(3000)
            m.scale.x, m.scale.y, m.scale.z = 0.02, 0.06, 0.1
            m.color = ColorRGBA(rgb[0],rgb[1],rgb[2],1.0)
            if dist < 0.65:
                res.append(m)
            prev_t = t

        for t, t_meta in tsdata_lst[::label_downsample]:
            text = Marker(type=Marker.TEXT_VIEW_FACING,
                          action=Marker.ADD,
                          header=h,
                          id=cls.marker_id)
            cls.marker_id += 1
            text.scale.z = 0.1
            text.pose = T.poseFromTransform(t.transform)
            text.pose.position.z += zoffset
            text.color = ColorRGBA(0.0,0.0,1.0,1.0)
            text.text = t_meta["inserted_at"].strftime(fmt)
            text.ns = "labeled_line_text"
            text.lifetime = rospy.Time(3000)
            res.append(text)
        return res

    @classmethod
    def transformStampedArrayToLabeledLineStripMarker(cls, tsdata_lst, fmt="%Y-%m-%d %H:%M:%S", zoffset=0.05, label_downsample=1, discrete=False):
        "[[transformStamped, meta],...] -> LineStrip / String"
        res = []
        # make line strip
        points = []
        colors = []
        t_first = tsdata_lst[0][0]
        prev_t = t_first.transform.translation
        for ts, _ in tsdata_lst:
            t = ts.transform.translation
            dist = distanceOfVector3(prev_t, t) * 0.65
            rgb = colorsys.hsv_to_rgb(dist, 1.0, 1.0)
            points.append(Point(x=t.x, y=t.y, z=t.z))
            colors.append(ColorRGBA(rgb[0],rgb[1],rgb[2],1.0))
            prev_t = t

        h = Header()
        h.stamp = rospy.Time(0) #rospy.Time.now()
        h.frame_id = "eng2" #t_first.child_frame_id
        if discrete:
            m_type = Marker.POINTS
        else:
            m_type = Marker.LINE_STRIP
        m = Marker(type=m_type,
                   action=Marker.ADD,
                   header=h,
                   id=cls.marker_id)
        cls.marker_id += 1
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.points = points
        m.colors = colors
        m.ns = "labeled_line"
        m.lifetime = rospy.Time(3000)
        res.append(m)

        for t, t_meta in tsdata_lst[::label_downsample]:
            text = Marker(type=Marker.TEXT_VIEW_FACING,
                          action=Marker.ADD,
                          header=h,
                          id=cls.marker_id)
            cls.marker_id += 1
            text.scale.z = 0.1
            text.pose = T.poseFromTransform(t.transform)
            text.pose.position.z += zoffset
            text.color = ColorRGBA(0.0,0.0,1.0,1.0)
            text.text = t_meta["inserted_at"].strftime(fmt)
            text.ns = "labeled_line_text"
            text.lifetime = rospy.Time(3000)
            res.append(text)
        return res

    @classmethod
    def poseStampedToLabeledSphereMarker(cls, psdata, label, fmt="{label} %Y-%m-%d %H:%M:%S", zoffset=0.05):
        "[poseStamped, meta] -> sphereMarker"
        ps, meta = psdata
        res = []
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = ps.header.frame_id
        sphere = Marker(type=Marker.SPHERE,
                   action=Marker.ADD,
                   header=h,
                   id=cls.marker_id)
        sphere.scale.x = 0.07
        sphere.scale.y = 0.07
        sphere.scale.z = 0.07
        sphere.pose = ps.pose
        sphere.color = ColorRGBA(1.0,0,0,1.0)
        sphere.ns = "db_play"
        sphere.lifetime = rospy.Time(3)
        cls.marker_id += 1
        res.append(sphere)

        text = Marker(type=Marker.TEXT_VIEW_FACING,
                   action=Marker.ADD,
                   header=h,
                   id=cls.marker_id)
        text.scale.z = 0.1
        text.pose = deepcopy(ps.pose)
        text.pose.position.z += zoffset
        text.color = ColorRGBA(1.0,1.0,1.0,1.0)
        text.text = meta["inserted_at"].strftime(fmt).format(label=label)
        text.ns = "db_play_text"
        text.lifetime = rospy.Time(300)
        cls.marker_id += 1
        res.append(text)
        return res

