#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler


class coordinate_converter:

    def __init__(self):
    
    	self.quaternion = rospy.Subscriber("vicon/bebop/bebop",TransformStamped,self.callback)

        self.euler_pub = rospy.Publisher("euler_angles",Float64MultiArray, queue_size=10)

        

    def callback(self,pose):
        x = pose.transform.translation.x
        y = pose.transform.translation.y
        z = pose.transform.translation.z
        rx = pose.transform.rotation.x
        ry = pose.transform.rotation.y
        rz = pose.transform.rotation.z
        rw = pose.transform.rotation.w

        quaternion = (
            rx,
            ry,
            rz,
            rw)

        self.bebop_quaternion_pose = [x, y, z, rx, ry, rz]

        (roll, pitch, yaw) = euler_from_quaternion([x, ry, rz, rw])

        euler_pose = Float64MultiArray()

        euler_pose.data = [roll, pitch, yaw]

        
        self.euler_pub.publish(euler_pose)

        print(roll, pitch, yaw)


def main(args):

    rospy.init_node('quaternion_euler_node')
    ic = coordinate_converter()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
