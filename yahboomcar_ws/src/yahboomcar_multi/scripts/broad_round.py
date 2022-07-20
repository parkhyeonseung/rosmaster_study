#!/usr/bin/env python
# coding: utf-8
import rospy, sys
from math import pi, cos, sin
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class RoundBroad:
    def __init__(self, robot_name):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("RoundBroad")
        self.alpha = 0.0
        self.radius = 0.7
        self.robot_pose = "/" + robot_name + "/base_footprint"
        self.sub_dist = rospy.Subscriber("/dist", Float32, self.dist_callback)
        self.broadcaster = TransformBroadcaster()

    def dist_callback(self, msg):
        if not isinstance(msg, Float32): return
        self.radius = msg.data

    def cancel(self):
        self.sub_dist.unregister()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            self.alpha += 5.0 * pi / 180.0
            self.senTransform(self.radius * cos(self.alpha), self.radius * sin(self.alpha),
                              (self.alpha + 135) % 360, "point1", self.robot_pose)

    def senTransform(self, x, y, theta, parent, child):
        translation = (x, y, 0)
        rotation = quaternion_from_euler(0, 0, theta)
        self.broadcaster.sendTransform(translation, rotation, rospy.Time.now(), parent, child)

if __name__ == '__main__':
    if len(sys.argv) != 1: broad = RoundBroad(sys.argv[1])
    else: broad = RoundBroad("robot1")
    broad.run()
    rospy.spin()
