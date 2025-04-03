#! /usr/bin/python

import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import sys
import os
import rosbag
import tf
from biped_localization.OdometryBagViewer import *

if __name__ == '__main__':
    bagname = sys.argv[1]
    base_bagname = os.path.splitext(bagname)[0]
    ground_truth_odom = SlamData2d("Ground Truth Odometry")
    biped_odom = SlamData2d("Odometry from Gait Generator")
    viso_odom = SlamData2d("Raw Visual Odometry")
    viso_odom_integrated = SlamData2d("Visual Odometry + Feedback")
    biped_odom_particle = SlamData2d("Odometry from Particle Filter", True)
    slam_odom = SlamData2d("Estimated Posture from SLAM")

    # parse
    with rosbag.Bag(os.path.abspath(bagname)) as bag:
        for topic, msg, timestamp in bag.read_messages(topics=["/ground_truth_odom",
                                                               "/biped_odom_offset",
                                                               "/viso_odom_offset",
                                                               "/viso_odom_integrated",
                                                               "/biped_odom_particle",
                                                               "/slam_odom"
                                                           ]):
            if topic == "/ground_truth_odom":
                parse_odometory_data(msg, ground_truth_odom)
            elif topic == "/biped_odom_offset":
                parse_odometory_data(msg, biped_odom)
            elif topic == "/viso_odom_offset":
                parse_odometory_data(msg, viso_odom)
            elif topic == "/viso_odom_integrated":
                parse_odometory_data(msg, viso_odom_integrated)
            elif topic == "/biped_odom_particle":
                parse_odometory_data(msg, biped_odom_particle)
            elif topic == "/slam_odom":
                parse_odometory_data(msg, slam_odom)
                # if msg.header.seq < 3899:
                #     parse_odometory_data(msg, slam_odom)

    # fix
    # diff_x = ground_truth_odom.x[0] - biped_odom.x[0]
    # ground_truth_odom.x = map(lambda x: x - diff_x, ground_truth_odom.x)
    # diff_y = ground_truth_odom.y[0] - biped_odom.y[0]
    # ground_truth_odom.y = map(lambda y: y - diff_y, ground_truth_odom.y)
    # diff_z = ground_truth_odom.z[0] - biped_odom.z[0]
    # ground_truth_odom.z = map(lambda z: z - diff_z, ground_truth_odom.z)

    mpl.rc('figure', figsize=(11.69,8.27))
    mpl.rc('font', size=12)
    mpl.rc('pdf', fonttype=42)
    mpl.rc('ps', fonttype=42)

    # error plot
    # data_list = [ground_truth_odom, biped_odom, viso_odom, viso_odom_integrated, biped_odom_particle, slam_odom]
    # # data_list = [ground_truth_odom, biped_odom, viso_odom, viso_odom_integrated, biped_odom_particle]

    # subplt_x = plt.subplot(2, 3, 1)
    # make_error_plot(subplt_x, "x translation", "Time[sec]", "Translation Error[m]", data_list[0], data_list[1:], "x", view_legend = "upper left")
    # subplt_y = plt.subplot(2, 3, 2)
    # make_error_plot(subplt_y, "y translation", "Time[sec]", "Translation Error[m]", data_list[0], data_list[1:], "y", view_legend = None)
    # subplt_z = plt.subplot(2, 3, 3)
    # make_error_plot(subplt_z, "z translation", "Time[sec]", "Translation Error[m]", data_list[0], data_list[1:], "z", view_legend = None)
    # subplt_roll = plt.subplot(2, 3, 4)
    # make_error_plot(subplt_roll, "roll_rotation", "Time[sec]", "Rotation Error[rad]", data_list[0], data_list[1:], "roll", view_legend = None)
    # subplt_pitch = plt.subplot(2, 3, 5)
    # make_error_plot(subplt_pitch, "pitch rotation", "Time[sec]", "Rotation Error[rad]", data_list[0], data_list[1:], "pitch", view_legend = None)
    # subplt_yaw = plt.subplot(2, 3, 6)
    # make_error_plot(subplt_yaw, "yaw rotation", "Time[sec]", "Rotation Error[rad]", data_list[0], data_list[1:], "yaw", view_legend = None)
    # plt.gcf().subplots_adjust(bottom=0.13)
    # plt.tight_layout()
    # plt.savefig(base_bagname + '_errors.pdf', format="pdf", figsize=(8, 6), dpi=300)

    # trajectory plot
    # data_list = [biped_odom, viso_odom, viso_odom_integrated, biped_odom_particle, slam_odom, ground_truth_odom]
    # data_list = [biped_odom, viso_odom, viso_odom_integrated, biped_odom_particle,  ground_truth_odom]
    # data_list = [biped_odom, viso_odom, slam_odom]
    data_list = [ground_truth_odom, biped_odom, viso_odom, slam_odom]

    subplt_x = plt.subplot(1, 3, 1)
    make_subplot(subplt_x, "x translation", "Time[sec]", "Translation[m]", data_list, "x", view_legend = "upper left")
    subplt_y = plt.subplot(1, 3, 2)
    make_subplot(subplt_y, "y translation", "Time[sec]", "Translation[m]", data_list, "y", view_legend = None)
    subplt_theta = plt.subplot(1, 3, 3)
    make_subplot(subplt_theta, "yaw rotation", "Time[sec]", "Rotation[rad]", data_list, "theta", view_legend = None)
    # subplt_z = plt.subplot(1, 4, 4)
    # make_subplot(subplt_z, "z translation", "Time[sec]", "Translation[m]", data_list_z, "z")
    plt.tight_layout()
    # plt.savefig(base_bagname + '_time_evaluation.eps', format="eps", figsize=(8, 6), dpi=300)
    plt.savefig(base_bagname + '_time_evaluation.pdf', format="pdf", figsize=(8, 6), dpi=300)

    # trajectory plot
    mpl.rc('font', size=14)
    subplt_2dtraj = plt.subplot(1, 1, 1)
    make_2d_trajectory(subplt_2dtraj, "trajectory", "X Translation[m]", "Y Translation[m]", data_list, view_legend = "upper right")
    plt.tight_layout()
    ylim = list(subplt_2dtraj.get_ylim())
    subplt_2dtraj.set_ylim([ylim[0], ylim[1] + 0.2])
    # plt.savefig(base_bagname + '_trajectory.eps', format="eps", dpi=800)
    plt.savefig(base_bagname + '_trajectory.pdf', format="pdf", dpi=800)
