#!/usr/bin/python3

import numpy as np
import rospy
from aruco_localization.msg import MarkerMeasurementArray as MMA
from nav_msgs.msg import Odometry
from mhe import MHE

class MHENode():
    def __init__(self):
        self.z_cur = np.zeros(0)
        self.z_ind = np.zeros(0, dtype=np.bool)
        self.id2idx = {5:0, 25:1, 55:2, 64:3, 76:4, 110:5, 121:6, 245:7, 248:8}

        self.prev_t = 0.
        self.estimator = MHE()

        rospy.Subscriber('aruco/measurements', MMA, self.measCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)

        while not rospy.is_shutdown():
            rospy.spin()

        print('[MHE] shutdown: writing to file')
        poses = np.array(self.estimator.pose_hist)
        np.savetxt('/tmp/MHE_outputs.txt', poses)

    def measCallback(self, msg):
        idx = []
        z = []
        for pose in msg.poses:
            idx.append([self.id2idx[pose.aruco_id]])
            pt = np.array([pose.position.x, pose.position.y, pose.position.z])
            r = np.sqrt(pt @ pt) * 10
#            r = 11 * pt.item(2)
            phi = -np.arctan2(pt[0], pt[2])
            z.append([r, phi])
        self.z_cur = np.array(z).T
        self.z_ind = np.array(idx).T

    def odomCallback(self, msg):
        if self.prev_t == 0:
            self.prev_t = msg.header.stamp.to_sec()
            return
        now = msg.header.stamp.to_sec()
        dt = now - self.prev_t
        self.prev_t = now

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.estimator.update(self.z_cur, self.z_ind, v, w, dt)

if __name__ == '__main__':
    rospy.init_node('mhe', anonymous=False)

    try:
        obj = MHENode()
    except rospy.ROSInterruptException:
        pass
