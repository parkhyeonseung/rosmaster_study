#!/usr/bin/env python
# coding: utf-8
import os
import rospy
import rospkg
import threading
from astra_common import *
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import Position
from sensor_msgs.msg import CompressedImage, Image
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from yahboomcar_astra.cfg import ColorHSVConfig


class Color_Identify:
    def __init__(self):
        nodeName = "colorHSV"
        rospy.init_node(nodeName, anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.hsv_text = rospkg.RosPack().get_path("yahboomcar_astra") + "/scripts/colorHSV.text"
        Server(ColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client(nodeName, timeout=60)
        self.pub_position = rospy.Publisher("/Current_point", Position, queue_size=10)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        if self.VideoSwitch == False:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.pub_rgb = rospy.Publisher("/astraTracker/rgb", Image, queue_size=1)
            self.sub_img = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.compressed_topic, queue_size=1)
        print("OpenCV Version: ", cv.__version__)

    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        return config

    def compressed_topic(self, msg):
        if not isinstance(msg, CompressedImage): return
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        action = cv.waitKey(10) & 0xFF
        rgb_img, binary = self.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        thread_text = "thread : " + str(len(threading.enumerate()))
        cv.putText(rgb_img, thread_text, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
        else: cv.imshow(self.windows_name, rgb_img)
        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False
        if self.Track_state == 'tracking':
            self.Start_state = True
            if self.circle[2] != 0: threading.Thread(
                target=self.execute, args=(self.circle[0], self.circle[1], self.circle[2])).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(
                target=self.execute, args=(self.point_pose[0], self.point_pose[1], self.point_pose[2])).start()
        else:
            if self.Start_state == True:
                self.pub_cmdVel.publish(Twist())
                self.Start_state = False
        return rgb_img, binary

    def execute(self, x, y, z):
        position = Position()
        position.angleX = x
        position.angleY = y
        position.distance = z
        self.pub_position.publish(position)

    def cancel(self):
        self.Reset()
        self.dyn_client.close()
        self.pub_position.unregister()
        if self.VideoSwitch == False:
            self.pub_rgb.unregister()
            self.sub_img.unregister()
        print("Shutting down this node.")
        cv.destroyAllWindows()

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        for i in range(3): self.pub_position.publish(Position())
        rospy.loginfo("init succes!!!")


if __name__ == '__main__':
    astra_tracker = Color_Identify()
    if astra_tracker.VideoSwitch == False: rospy.spin()
    else:
        capture = cv.VideoCapture(0)
        cv_edition = cv.__version__
        if cv_edition[0]=='3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        while capture.isOpened():
            start = time.time()
            ret, frame = capture.read()
            action = cv.waitKey(10) & 0xFF
            frame, binary = astra_tracker.process(frame, action)
            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
            else:cv.imshow('frame', frame)
            if action == ord('q') or action == 113: break
        capture.release()
        cv.destroyAllWindows()


