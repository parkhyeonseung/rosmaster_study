#!/usr/bin/env python
# coding: utf-8
import time
import rospy
import cv2 as cv
from astra_common import simplePID
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import Position
from dynamic_reconfigure.server import Server
from yahboomcar_astra.cfg import ColorTrackerPIDConfig

class color_Tracker:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.minDist = 1500
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.Joy_active = False
        self.Robot_Run = False
        self.dist = []
        self.encoding = ['16UC1', '32FC1']
        self.sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_img_Callback, queue_size=1)
        self.sub_JoyState = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)
        self.sub_position = rospy.Subscriber("/Current_point", Position, self.positionCallback)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        Server(ColorTrackerPIDConfig, self.AstraFollowPID_callback)
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.PID_init()

    def AstraFollowPID_callback(self, config, level):
        self.linear_PID = (config['linear_Kp'], config['linear_Ki'], config['linear_Kd'])
        self.angular_PID = (config['angular_Kp'], config['angular_Ki'], config['angular_Kd'])
        self.minDist = config['minDist'] * 1000
        print ("linear_PID: ", self.linear_PID)
        print ("angular_PID: ", self.angular_PID)
        self.PID_init()
        return config

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)

    def depth_img_Callback(self, msg):
        if not isinstance(msg, Image): return
        depthFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding[1])
        self.action = cv.waitKey(1)
        if self.Center_r != 0:
            now_time = time.time()
            if now_time - self.prev_time > 5:
                if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r: self.Center_r = 0
                self.prev_time = now_time
            distance = [0, 0, 0, 0, 0]
            if 0 < int(self.Center_y - 3) and int(self.Center_y + 3) < 480 and 0 < int(
                self.Center_x - 3) and int(self.Center_x + 3) < 640:
                # print("depthFrame: ", len(depthFrame), len(depthFrame[0]))
                distance[0] = depthFrame[int(self.Center_y - 3)][int(self.Center_x - 3)]
                distance[1] = depthFrame[int(self.Center_y + 3)][int(self.Center_x - 3)]
                distance[2] = depthFrame[int(self.Center_y - 3)][int(self.Center_x + 3)]
                distance[3] = depthFrame[int(self.Center_y + 3)][int(self.Center_x + 3)]
                distance[4] = depthFrame[int(self.Center_y)][int(self.Center_x)]
                distance_ = 1000.0
                num_depth_points = 5
                for i in range(5):
                    if 40 < distance[i] < 80000: distance_ += distance[i]
                    else: num_depth_points -= 1
                if num_depth_points == 0: distance_ = self.minDist
                else: distance_ /= num_depth_points
                #print("Center_x: {}, Center_y: {}, distance_: {}".format(self.Center_x, self.Center_y, distance_))
                self.execute(self.Center_x, distance_)
                self.Center_prevx = self.Center_x
                self.Center_prevr = self.Center_r
        else:
            if self.Robot_Run ==True:
                self.pub_cmdVel.publish(Twist())
                self.Robot_Run = False
        if self.action == ord('q') or self.action == 113: self.cleanup()
        # cv.imshow("depth_img", depthFrame)

    def execute(self, point_x, dist):
        if abs(self.prev_dist - dist) > 300:
            self.prev_dist = dist
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return
        if self.Joy_active == True: return
        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(320, point_x)
        if abs(dist - self.minDist) < 30: linear_x = 0
        if abs(point_x - 320.0) < 30: angular_z = 0
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = linear_x
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True
        # rospy.loginfo(
        #     "point_x: {},dist: {},linear_x: {}, angular_z: {}".format(
        #         point_x, dist, twist.linear.x, twist.angular.z))

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())

    def positionCallback(self, msg):
        if not isinstance(msg, Position): return
        self.Center_x = msg.angleX
        self.Center_y = msg.angleY
        self.Center_r = msg.distance

    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        self.sub_depth.unregister()
        self.sub_JoyState.unregister()
        self.sub_position.unregister()
        self.pub_cmdVel.unregister()
        print ("Shutting down this node.")
        cv.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("color_Tracker", anonymous=False)
    color_Tracker()
    rospy.spin()
