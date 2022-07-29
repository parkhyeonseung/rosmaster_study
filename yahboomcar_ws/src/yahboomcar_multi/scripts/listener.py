#!/usr/bin/env python
# coding: utf-8
import math
import sys
import rospy
import numpy as np
from time import sleep
from math import sqrt, atan2
from singlePID import SinglePID
from tf import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from tf.listener import TransformListener
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from yahboomcar_multi.cfg import RobotListenerConfig

RAD2DEG = 180 / math.pi


class RobotListener:
    def __init__(self, robot_name, source_frame):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("RobotListener", anonymous=False)
        self.switch = True
        self.navigate = False
        self.Joy_active = False
        self.pause = False
        self.warning = 1
        self.LaserAngle = 30
        self.ResponseDist = 0.3
        self.listener = TransformListener()
        self.broadcaster = TransformBroadcaster()
        self.lin_pid = SinglePID()
        self.ang_pid = SinglePID()
        self.source_frame = source_frame
        self.robot_model = "omni"
        self.robot_pose = "/" + robot_name + "/base_footprint"
        # 发布目标点 || Publish target point
        self.pub_order = rospy.Publisher("/order", String, queue_size=10)
        self.pub_dist = rospy.Publisher("/dist", Float32, queue_size=10)
        self.pub_vel = rospy.Publisher("/" + robot_name + "/cmd_vel", Twist, queue_size=10)
        self.pub_goal = rospy.Publisher("/" + robot_name + "/move_base_simple/goal", PoseStamped, queue_size=1)
        self.sub_scan = rospy.Subscriber("/" + robot_name + "/scan", LaserScan, self.registerScan, queue_size=1)
        self.sub_JoyState = rospy.Subscriber("/" + robot_name + "/JoyState", Bool, self.JoyStateCallback)
        Server(RobotListenerConfig, self.dynReConfCallback)
        self.linPIDparam = rospy.get_param('~linPIDparam', [1.0, 0, 1.0])
        self.angPIDparam = rospy.get_param('~angPIDparam', [0.8, 0, 1.0])
        self.lin_pid.Set_pid(self.linPIDparam[0], self.linPIDparam[1], self.linPIDparam[2])
        self.ang_pid.Set_pid(self.angPIDparam[0], self.angPIDparam[1], self.angPIDparam[2])

    def cancel(self):
        self.pub_vel.publish(Twist())
        self.pub_order.unregister()
        self.pub_vel.unregister()
        self.pub_goal.unregister()
        self.sub_scan.unregister()
        self.sub_JoyState.unregister()

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        self.warning = 1
        if not isinstance(scan_data, LaserScan): return
        if self.Joy_active == True: return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        ranges = np.array(scan_data.ranges)
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if abs(angle) > (180 - self.LaserAngle):
                if ranges[i] < self.ResponseDist: self.warning += 1
        # print "warning: ",self.warning

    def PubTargetPoint(self, transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = transform[0][0]
        pose.pose.position.y = transform[0][1]
        pose.pose.position.z = transform[0][2]
        pose.pose.orientation.x = transform[1][0]
        pose.pose.orientation.y = transform[1][1]
        pose.pose.orientation.z = transform[1][2]
        pose.pose.orientation.w = transform[1][3]
        self.pub_goal.publish(pose)
        sleep(1)

    def run(self):
        rate = rospy.Rate(10)
        start_status = True
        diff_pid_status = True
        min_dist = 100
        minX = 0
        minY = 0
        sleep(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.Joy_active or self.switch:
                start_status = True
                min_dist = 100
                if self.pause:
                    for i in range(3): self.pub_vel.publish(Twist())
                    self.pause = False
            else:
                self.pause = True
                if self.navigate:
                    try:
                        transform = self.listener.lookupTransform("map", self.source_frame, rospy.Time())
                    except Exception as e:
                        rospy.logerr("listen map frame failed !!!")
                        continue
                    self.PubTargetPoint(transform)
                else:
                    try:
                        transform = self.listener.lookupTransform(self.robot_pose, self.source_frame, rospy.Time())
                    except Exception as e:
                        rospy.logerr("listener failed !!!")
                        continue
                    x, y, z = transform[0]
                    theta = euler_from_quaternion(transform[1])[2]
                    distance = sqrt(pow(x, 2) + pow(y, 2))
                    if start_status:
                        if abs(min_dist - distance) < 0.01 and abs(minX - x) < 0.1 and abs(minY - y) < 0.1:
                            start_status = False
                        elif min_dist > distance:
                            min_dist = distance
                            minX = x
                            minY = y
                    else:
                        twist = Twist()
                        if self.robot_model == "omni":
                            diff_pid_status = True
                            # print "x: {},y: {},theta: {}".format(x, y, theta)
                            twist.linear.x = self.lin_pid.pid_compute(x, 0) / 4
                            twist.linear.y = self.lin_pid.pid_compute(y, 0) / 4
                            twist.angular.z = self.lin_pid.pid_compute(np.interp(theta * RAD2DEG, [-180, 180], [-5, 5]), 0) / 4
                            self.pub_vel.publish(twist)

                        elif self.robot_model == "diff":
                            if diff_pid_status:
                                self.lin_pid.Set_pid(1.0, 0, 1.0)
                                self.ang_pid.Set_pid(0.8, 0, 1.0)
                                diff_pid_status = False
                            if x >= 0: direction = 1
                            else: direction = -1
                            angular = np.interp(atan2(y, x) * RAD2DEG, [-180, 180], [-5, 5])
                            if abs(distance) > 0.1:
                                if self.lin_pid.pid_compute(distance, 0) > 0.6: twist.linear.x = 0.6 * direction
                                else: twist.linear.x = self.lin_pid.pid_compute(distance, 0) * direction
                            if abs(y) > 0.1: twist.angular.z = self.ang_pid.pid_compute(angular, 0)
                            else: twist.angular.z = theta
                            self.pub_vel.publish(twist)
                            # print("x: {},y: {},angular: {}".format(x, y, angular / 3.14 * 180))
                            # print("x: {},z: {}".format(twist.linear.x, twist.angular.z))
        self.pub_vel.publish(Twist())

    def dynReConfCallback(self, config, level):
        string = String()
        f = Float32()
        self.lin_pid.Set_pid(config['lin_Kp'], config['lin_Ki'], config['lin_Kd'])
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        self.navigate = config['navigate']
        self.switch = config['switch']
        f.data = config['dist']
        if config['robot_model'] == 0: self.robot_model = "omni"
        elif config['robot_model'] == 1: self.robot_model = "diff"
        if config['teams'] == 0: string.data = "convoyl"
        elif config['teams'] == 1: string.data = "vertical"
        elif config['teams'] == 2: string.data = "horizonta"
        for i in range(3): self.pub_order.publish(string)
        for i in range(3): self.pub_dist.publish(f)
        return config


if __name__ == '__main__':
    if len(sys.argv) != 1: robotlistener = RobotListener(sys.argv[1], sys.argv[2])
    else: robotlistener = RobotListener("robot2", "point1")
    robotlistener.run()
    rospy.spin()
