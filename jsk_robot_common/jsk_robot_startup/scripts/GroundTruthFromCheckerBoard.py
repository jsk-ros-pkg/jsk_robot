#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from std_msgs.msg import Empty
import tf
import threading
import numpy


# from hrp::omegaFromRot,  http://54.183.65.232/docs/indigo/api/openhrp3/html/Eigen3d_8cpp_source.html
def omega_from_rot(rot):
    alpha = (rot[0, 0] + rot[1, 1] + rot[2, 2] - 1.0) / 2.0

    if alpha > 1.0:
        rospy.logwarn("alpha too large: %f", alpha)
        alpha = 1.0 # alpha exceeded the upper limit
        
    if abs(alpha - 1.0) < 1e-6: # theta = 0 or 2*pi
        return numpy.array([0, 0, 0])
    else:
        theta = numpy.arccos(alpha)
        sin_theta = numpy.sin(theta)
        if abs(sin_theta) < 1e-16: # theta = pi
            return numpy.array([numpy.sqrt((rot[0, 0] + 1.0) * 0.5) * theta,
                                numpy.sqrt((rot[1, 1] + 1.0) * 0.5) * theta,
                                numpy.sqrt((rot[2, 2] + 1.0) * 0.5) * theta])
        else:
            k = -0.5 * theta / sin_theta
            return numpy.array([(rot[1, 2] - rot[2, 1]) * k,
                                (rot[2, 0] - rot[0, 2]) * k,
                                (rot[0, 1] - rot[1, 0]) * k])


class GroundTruthFromCheckerBoard:
    def __init__(self):
        rospy.init_node("GroundTruthFromCheckerBoard", anonymous=True)
        self.base_frame = rospy.get_param("~base_frame", "/BODY")
        self.odom_frame = rospy.get_param("~odom_frame", "/checkerboard_ground_truth")
        self.invert_tf = rospy.get_param("~invert_tf", True)
        self.lock = threading.Lock()
        self.odom = None
        self.rate = rospy.get_param("~rate", 10)
        self.listener = tf.TransformListener()        
        self.broadcast = tf.TransformBroadcaster()        
        self.r = rospy.Rate(self.rate) # 10hz
        self.init_homogeneous_matrix = None
        self.pose_sub = rospy.Subscriber("~input_pose", PoseStamped, self.callback)
        self.init_sub = rospy.Subscriber("~init_trigger", Empty, self.init_callback)
        self.pub = rospy.Publisher("~output", Odometry, queue_size=10)

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def init_callback(self, msg):
        with self.lock:
            self.init_homogeneous_matrix = None
            self.odom = None

    def callback(self, msg):
        try:
            (trans,rot) = self.listener.lookupTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("failed to solve tf: %s to %s", self.base_frame, msg.header.frame_id)
            return
        with self.lock:
            base_to_camera_homogeneous_matrix = self.make_homogeneous_matrix(Point(*trans), Quaternion(*rot)) # base(t) -> camera(t)
            camera_to_marker_homogeneous_matrix = self.make_homogeneous_matrix(msg.pose.position, msg.pose.orientation) # camera(t) -> marker(const)
            base_to_marker_homogeneous_matrix = base_to_camera_homogeneous_matrix.dot(camera_to_marker_homogeneous_matrix) # base(t) -> marker(const)
            if self.init_homogeneous_matrix == None:
                self.init_homogeneous_matrix = base_to_marker_homogeneous_matrix # base(0) -> marker(const)
            result_homogeneous_matrix = self.init_homogeneous_matrix.dot(numpy.linalg.inv(base_to_marker_homogeneous_matrix)) # base(0) -> marker(const) -> base(t)

            pub_msg = Odometry()
            pub_msg.pose.pose.position = Point(*list(result_homogeneous_matrix[:3, 3]))
            pub_msg.pose.pose.orientation = Quaternion(*list(tf.transformations.quaternion_from_matrix(result_homogeneous_matrix)))
            pub_msg.header.stamp = msg.header.stamp
            pub_msg.header.frame_id = self.odom_frame
            pub_msg.child_frame_id = self.base_frame
            
            if self.odom != None:
                dt = (pub_msg.header.stamp - self.odom.header.stamp).to_sec()
                rotation_matrix = tf.transformations.quaternion_matrix([pub_msg.pose.pose.orientation.x, pub_msg.pose.pose.orientation.y,
                                                                        pub_msg.pose.pose.orientation.z, pub_msg.pose.pose.orientation.w])[:3, :3]
                prev_rotation_matrix = tf.transformations.quaternion_matrix([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                                                            self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[:3, :3]
                local_omega = numpy.dot(numpy.transpose(rotation_matrix), omega_from_rot(numpy.dot(rotation_matrix, numpy.transpose(prev_rotation_matrix))) / dt)
                global_vel = []
                global_vel.append((pub_msg.pose.pose.position.x - self.odom.pose.pose.position.x) / dt)
                global_vel.append((pub_msg.pose.pose.position.y - self.odom.pose.pose.position.y) / dt)
                global_vel.append((pub_msg.pose.pose.position.z - self.odom.pose.pose.position.z) / dt)
                local_vel = numpy.dot(numpy.transpose(rotation_matrix), numpy.array(global_vel))
                pub_msg.twist.twist.linear.x = local_vel[0]
                pub_msg.twist.twist.linear.y = local_vel[1]
                pub_msg.twist.twist.linear.z = local_vel[2]
                pub_msg.twist.twist.angular.x = local_omega[0]
                pub_msg.twist.twist.angular.y = local_omega[1]
                pub_msg.twist.twist.angular.z = local_omega[2]
                
            self.broadcast_transform(pub_msg)
            self.pub.publish(pub_msg)
            self.odom = pub_msg

    def make_homogeneous_matrix(self, position, orientation):
        homogeneous_matrix = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        homogeneous_matrix[:3, 3] = numpy.array([position.x, position.y, position.z]).reshape(1, 3)
        return homogeneous_matrix
    
    def broadcast_transform(self, odom):
        position = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
        orientation = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, 
                       odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        if self.invert_tf:
            homogeneous_matrix = tf.transformations.quaternion_matrix(orientation)
            homogeneous_matrix[:3, 3] = numpy.array(position).reshape(1, 3)
            homogeneous_matrix_inv = numpy.linalg.inv(homogeneous_matrix)
            position = list(homogeneous_matrix_inv[:3, 3])
            orientation = list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_inv))
            parent_frame = self.base_frame
            target_frame = self.odom_frame
        else:
            parent_frame = self.odom_frame
            target_frame = self.base_frame
        self.broadcast.sendTransform(position, orientation, odom.header.stamp, target_frame, parent_frame)
            
if __name__ == '__main__':
    try:
        node = GroundTruthFromCheckerBoard()
        node.execute()
    except rospy.ROSInterruptException: pass
