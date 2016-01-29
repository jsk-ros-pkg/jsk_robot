#! /usr/bin/env python

import rospy
import tf
import numpy
import scipy.stats
import math
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Twist, Pose, Point, Quaternion, Vector3

# twist transformation
def transform_local_twist_to_global(pose, local_twist):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y,
           pose.orientation.z, pose.orientation.w]
    rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
    global_velocity = numpy.dot(rotation_matrix, numpy.array([[local_twist.linear.x],
                                                              [local_twist.linear.y],
                                                              [local_twist.linear.z]]))
    global_omega = numpy.dot(rotation_matrix, numpy.array([[local_twist.angular.x],
                                                           [local_twist.angular.y],
                                                           [local_twist.angular.z]]))
    return Twist(Vector3(*global_velocity[:, 0]), Vector3(*global_omega[:, 0]))

def transform_local_twist_covariance_to_global(pose, local_twist_with_covariance):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y,
           pose.orientation.z, pose.orientation.w]
    rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
    twist_cov_matrix = numpy.matrix(local_twist_with_covariance).reshape(6, 6)
    global_twist_cov_matrix = numpy.zeros((6, 6))
    global_twist_cov_matrix[:3, :3] = (rotation_matrix.T).dot(twist_cov_matrix[:3, :3].dot(rotation_matrix))
    global_twist_cov_matrix[3:6, 3:6] = (rotation_matrix.T).dot(twist_cov_matrix[3:6, 3:6].dot(rotation_matrix))
    return global_twist_cov_matrix.reshape(-1,).tolist()

# pose calculation
def update_pose(pose, global_twist, dt):
    ret_pose = Pose()
    # calculate current pose as integration
    ret_pose.position.x = pose.position.x + global_twist.linear.x * dt
    ret_pose.position.y = pose.position.y + global_twist.linear.y * dt
    ret_pose.position.z = pose.position.z + global_twist.linear.z * dt
    ret_pose.orientation = update_quaternion(pose.orientation, global_twist.angular, dt)
    return ret_pose

def update_quaternion(orientation, angular, dt): # angular is assumed to be global
    # quaternion calculation
    quat_vec = numpy.array([[orientation.x],
                            [orientation.y],
                            [orientation.z],
                            [orientation.w]])
    skew_omega = numpy.matrix([[0, -angular.z, angular.y, angular.x],
                               [angular.z, 0, -angular.x, angular.y],
                               [-angular.y, angular.x, 0, angular.z],
                               [-angular.x, -angular.y, -angular.z, 0]])
    new_quat_vec = quat_vec + 0.5 * numpy.dot(skew_omega, quat_vec) * dt
    norm = numpy.linalg.norm(new_quat_vec)
    if norm == 0:
        rospy.logwarn("norm of quaternion is zero")
    else:
        new_quat_vec = new_quat_vec / norm # normalize
    return Quaternion(*numpy.array(new_quat_vec).reshape(-1,).tolist())

# covariance calculation
def update_twist_covariance(twist, v_sigma, min_sigma = 1e-3): # twist is assumed to be local
    twist_list = [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]
    if all([abs(x) < 1e-6 for x in twist_list]):
        current_sigma = [min_sigma] * 6 # trust "completely stopping" state
    else:
        current_sigma = v_sigma
    return numpy.diag([max(x ** 2, min_sigma ** 2) for x in current_sigma]).reshape(-1,).tolist() # covariance should be singular

def update_pose_covariance(pose_cov, global_twist_cov, dt):
    ret_pose_cov = []
    # make matirx from covariance array
    prev_pose_cov_matrix = numpy.matrix(pose_cov).reshape(6, 6)
    global_twist_cov_matrix = numpy.matrix(global_twist_cov).reshape(6, 6)
    # jacobian matrix
    # elements in pose and twist are assumed to be independent on global coordinates
    jacobi_pose = numpy.diag([1.0] * 6)
    jacobi_twist = numpy.diag([dt] * 6)
    # covariance calculation
    pose_cov_matrix = jacobi_pose.dot(prev_pose_cov_matrix.dot(jacobi_pose.T)) + jacobi_twist.dot(global_twist_cov_matrix.dot(jacobi_twist.T))
    # update covariances as array type (twist is same as before)
    ret_pose_cov = numpy.array(pose_cov_matrix).reshape(-1,).tolist()
    return ret_pose_cov

# tf broadcast
def broadcast_transform(broadcast, odom, invert_tf):
    if not odom or not broadcast:
        return
    position = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
    orientation = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    if invert_tf:
        homogeneous_matrix = make_homogeneous_matrix(position, orientation)
        homogeneous_matrix_inv = numpy.linalg.inv(homogeneous_matrix)
        position = list(homogeneous_matrix_inv[:3, 3])
        orientation = list(tf.transformations.quaternion_from_matrix(homogeneous_matrix_inv))
        parent_frame = odom.child_frame_id
        target_frame = odom.header.frame_id
    else:
        parent_frame = odom.header.frame_id
        target_frame = odom.child_frame_id
    broadcast.sendTransform(position, orientation, odom.header.stamp, target_frame, parent_frame)

# mathmatical tools    
def make_homogeneous_matrix(trans, rot):
    homogeneous_matrix = tf.transformations.quaternion_matrix(rot)
    homogeneous_matrix[:3, 3] = numpy.array(trans).reshape(1, 3)
    return homogeneous_matrix
    
# scipy.stats.multivariate_normal only can be used after SciPy 0.14.0
# input: x(array), mean(array), cov_inv(matrix) output: probability of x
# covariance has to be inverted to reduce calculation time
def norm_pdf_multivariate(x, mean, cov_inv):
    size = len(x)
    if size == len(mean) and (size, size) == cov_inv.shape:
        inv_det = numpy.linalg.det(cov_inv)
        if not inv_det > 0:
            rospy.logwarn("Determinant of inverse cov matrix {0} is equal or smaller than zero".format(inv_det))
            return 0.0
        norm_const = math.pow((2 * numpy.pi), float(size) / 2) * math.pow(1 / inv_det, 1.0 / 2) # determinant of inverse matrix is reciprocal
        if not norm_const > 0 :
            rospy.logwarn("Norm const {0} is equal or smaller than zero".format(norm_const))
            return 0.0
        x_mean = numpy.matrix(x - mean)
        exponent = -0.5 * (x_mean * cov_inv * x_mean.T)
        if exponent > 0:
            rospy.logwarn("Exponent {0} is larger than zero".format(exponent))
            exponent = 0
        result = math.pow(math.e, exponent)
        return result / norm_const
    else:
        rospy.logwarn("The dimensions of the input don't match")
        return 0.0

# tf.transformations.euler_from_quaternion is slow because the function calculates matrix inside.
# prev_euler expects previous [roll, pitch, yaw] angle list and fix ret_euler 
# cf. https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def transform_quaternion_to_euler(quat, prev_euler = None):
    # singularity check
    sin_half_pitch = quat[3] * quat[1] - quat[2] * quat[0]
    if abs(sin_half_pitch) > 0.499:
        # use tf.transformations.euler_from_quaternion only at singularity points
        ret_euler = list(tf.transformations.euler_from_quaternion(quat))
    else:
        # zero check
        zero_thre = numpy.finfo(float).eps * 4.0 # epsilon for testing whether a number is close to zero
        roll_numerator = 2 * (quat[3] * quat[0] + quat[1] * quat[2])
        if abs(roll_numerator) < zero_thre:
            roll_numerator = numpy.sign(roll_numerator) * 0.0
        yaw_numerator = 2 * (quat[3] * quat[2] + quat[0] * quat[1])
        if abs(yaw_numerator) < zero_thre:
            yaw_numerator = numpy.sign(yaw_numerator) * 0.0
        ret_euler = [numpy.arctan2(roll_numerator, 1 - 2 * (quat[0] ** 2 + quat[1] ** 2)),
                     numpy.arcsin(2 * sin_half_pitch),
                     numpy.arctan2(yaw_numerator, 1 - 2 * (quat[1] ** 2 + quat[2] ** 2))]
    # consider arctan/arcsin range
    # TODO: This solution is not fundamental because it does not consider ununiqueness of euler angles
    if prev_euler != None:
        # roll: arctan2 is in range of [-pi, pi]
        for i in range(3):
            while abs(prev_euler[i] - ret_euler[i]) > numpy.pi:
                ret_euler[i] += numpy.sign(prev_euler[i] - ret_euler[i]) * 2 * numpy.pi
    return ret_euler
