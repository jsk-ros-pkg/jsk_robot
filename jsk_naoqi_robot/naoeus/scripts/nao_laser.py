#! /usr/bin/env python
import roslib
roslib.load_manifest('nao_driver')
import rospy
from nao_driver import NaoNode
from sensor_msgs.msg import LaserScan
from naoqi import ALProxy

class NaoLaser (NaoNode):
    def __init__(self):
        NaoNode.__init__(self)
        rospy.init_node('nao_laser')

        # for measureddata
        self.startpoint = -30.0
        self.endpoint = 210.0
        self.scaninterval = 0
        self.dataGroupingnumber = 1
        self.dataInterval = 0.35
        self.sensorstate = 'NORMAL'
        self.distancenum = 682
        # for rangedata
        self.minangle = -0.49087
        self.maxangle = 3.64474
        self.maxdatanum = 768
        self.angres = 0.0061
        self.minrange = 0.06
        self.maxrange = 5.6
        self.rangeres = 0.01
        self.frequency = 10

        self.laserscan = LaserScan()
        self.laserscan.header.frame_id = "/urg_frame"
        self.laserscan.range_min = self.minrange
        self.laserscan.range_max = self.maxrange

        try:
            self.alurg = self.getProxy("ALLaser")
            self.alurg.setOpeningAngle(-2.09439, 2.09439)
            self.alurg.setDetectingLength(20, 5600)
            self.alurg.laserON()
        except Exception,e:
            print "Error when creating ALLaser proxy:"
            print str(e)
            exit(1)

        try:
            self.almem = self.getProxy("ALMemory")
        except Exception,e:
            print "Error when creating ALMemory proxy:"
            print str(e)
            exit(1)

        self.pub_laser = rospy.Publisher('scan', LaserScan)


    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                laserdata = self.almem.getData("Device/Laser/Value", 0)
            except:
                continue

            self.laserscan.header.stamp = rospy.get_rostime()

            minangle = -1
            maxangle = -1
            for i in range(len(laserdata)):
                if laserdata[i][1] != 0:
                    minangle = laserdata[i][1]
                    break
            cnt = 0
            # distance = []
            self.laserscan.ranges = []

            i_pre = -1
            for i in range(len(laserdata)):
                if laserdata[i][1] != 0:
                    if i_pre >= 0:
                        angdif = laserdata[i][1] - laserdata[i_pre][1]
                        angrate = int(round(angdif / self.angres, 0))
                        # print "i = ", i, ", angrate = ", angrate
                        for j in range(angrate - 1):
                            # interpolation
                            # distance.append(laserdata[i][0])
                            self.laserscan.ranges.append(laserdata[i][0] / 1000.0)
                            cnt += 1
                    # distance.append(laserdata[i][0])
                    self.laserscan.ranges.append(laserdata[i][0] / 1000.0)
                    i_pre = i
                    maxangle = laserdata[i][1]
                    cnt += 1
                # print "laserdata[", i_pre, "][0] = ", laserdata[i_pre][0]
                # print "laserdata[", i_pre, "][1] = ", laserdata[i_pre][1]
                # print "cnt = ", cnt
            self.laserscan.angle_min = minangle
            self.laserscan.angle_max = maxangle
            # self.laserscan.angle_increment = (maxangle - minangle) / cnt
            self.laserscan.angle_increment = self.angres
            self.pub_laser.publish(self.laserscan)


if __name__ == "__main__":
    try:
        naolaser = NaoLaser()
        naolaser.main_loop()
    except RuntimeError, e:
        rospy.logerr('Something went wrong: %s' % (e) )
    rospy.loginfo('Laser stopped')

