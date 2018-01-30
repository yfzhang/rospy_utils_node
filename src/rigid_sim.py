#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import tf
import tf.transformations as t
import math
import numpy as np


class RigidSim:
    """
    Simple kinematics simulation for rigid body
    Input: Twist command
    Output: update TF
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.dt = 0.1
        # self.transformation_mat = np.identity(4)
        self.transformation_mat = self.init_transformation_mat(x, y, z, roll, pitch, yaw)
        self.br = tf.TransformBroadcaster()
        self.twist = Twist()
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_cb)
        self.integrate_timer = rospy.Timer(rospy.Duration(self.dt), self.integrate_spin)
        self.publish_timer = rospy.Timer(rospy.Duration(0.02), self.publish_spin)

    @staticmethod
    def init_transformation_mat(x, y, z, roll, pitch, yaw):
        trans_mat = t.translation_matrix([x, y, z])
        rot_mat = t.euler_matrix(roll, pitch, yaw)
        return np.dot(trans_mat, rot_mat)

    def twist_cb(self, msg):
        self.twist = msg

    def publish_spin(self, event):
        quat = t.quaternion_from_matrix(self.transformation_mat)
        trans = t.translation_from_matrix(self.transformation_mat)
        self.br.sendTransform(trans, quat, rospy.Time.now(), "base_link", "world")

    def integrate_spin(self, event):
        dx = self.twist.linear.x * self.dt
        dy = self.twist.linear.y * self.dt
        dz = self.twist.linear.z * self.dt
        trans_mat = t.translation_matrix([dx, dy, dz])

        droll = self.twist.angular.x * self.dt
        dpitch = self.twist.angular.y * self.dt
        dyaw = self.twist.angular.z * self.dt
        rot_mat = t.euler_matrix(droll, dpitch, dyaw)

        homogeneous_mat = np.dot(trans_mat, rot_mat)

        # update transformation matrix (world->base_link)
        self.transformation_mat = np.dot(self.transformation_mat, homogeneous_mat)


if __name__ == '__main__':
    rospy.init_node('rigid_sim_node', log_level=rospy.DEBUG)
    node = RigidSim(-18, -10, -302, 0, 0, -2.443)
    rospy.spin()
