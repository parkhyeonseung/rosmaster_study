#!/usr/bin/env python
# coding: utf-8
import rospy, sys, math
from std_msgs.msg import String,Float32
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler

class QueueBroad:
    def __init__(self, robot_name):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("QueueBroad")
        self.order = "convoyl"
        self.Joy_active = False
        self.dist = 0.5
        self.robot_pose = "/" + robot_name + "/base_footprint"
        self.sub_order = rospy.Subscriber("/order", String, self.order_callback)
        self.sub_dist = rospy.Subscriber("/dist", Float32, self.dist_callback)
        self.broadcaster = TransformBroadcaster()

    def cancel(self):
        self.sub_dist.unregister()
        self.sub_order.unregister()

    def order_callback(self,msg):
        if not isinstance(msg, String): return
        self.order = msg.data

    def dist_callback(self, msg):
        if not isinstance(msg, Float32): return
        self.dist = msg.data

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.order == "convoyl": x1, y1, x2, y2 = -self.dist, self.dist, -self.dist, -self.dist
            elif self.order == "vertical": x1, y1, x2, y2 = -self.dist, 0, -self.dist*2, 0
            else: x1, y1, x2, y2 = 0, self.dist, 0, -self.dist
            self.senTransform(x1, y1, 0, "point1", self.robot_pose)
            self.senTransform(x2, y2, 0, "point2", self.robot_pose)
            r.sleep()

    def senTransform(self, x, y, theta, parent, child):
        translation = (x, y, 0)
        rotation = quaternion_from_euler(0, 0, theta)
        self.broadcaster.sendTransform(translation, rotation, rospy.Time.now(), parent, child)

if __name__ == '__main__':
    if len(sys.argv) != 1: q = QueueBroad(sys.argv[1])
    else: q = QueueBroad("robot1")
    q.run()
    rospy.spin()
