
import os
import threading
import math
import rospkg
from follow_common import *
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_linefollw.cfg import LineDetectPIDConfig
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from Speech_Lib import Speech
from yahboomcar_linefollw.cfg import LineDetectPIDConfig
RAD2DEG = 180 / math.pi
import time

class LineDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("LineDetect", anonymous=False)
        self.img = None
        self.circle = ()
        self.hsv_range = ()
        self.Roi_init = ()
        self.warning = 1
        self.Start_state = True
        self.dyn_update = False
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.img_flip = rospy.get_param("~img_flip", False)
        self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.hsv_text = rospkg.RosPack().get_path("yahboomcar_linefollw")+"/scripts/LineFollowHSV.text"
        Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("LineDetect", timeout=60)
        self.scale = 1000
        self.FollowLinePID = (80, 0, 20)
        self.linear = 0.2
        self.LaserAngle = 30
        self.ResponseDist = 0.55
        self.PID_init()
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        self.pub_Buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.spe = Speech()
        self.model = "General"
        self.ros_ctrl.Joy_active = True
        self.command_result = 999
        

    def cancel(self):
        #self.Reset()
        self.ros_ctrl.cancel()
        #self.sub_scan.unregister()
        #self.sub_img.unregister()
        #self.pub_rgb.unregister()
        #self.pub_Buzzer.unregister()
        print ("Shutting down this node.")
        ''''if self.VideoSwitch==False:
            self.sub_img.unregister()
            cv.destroyAllWindows()'''

    #雷达避障碍
    def registerScan(self, scan_data):
        self.warning = 1
        if not isinstance(scan_data, LaserScan): return
        if self.ros_ctrl.Joy_active == True: return
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

    def dynamic_reconfigure_callback(self, config, level):
        self.scale = config['scale']
        self.linear = config['linear']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        #write_HSV(self.hsv_text, self.hsv_range)
        #print ("HSV: ", self.hsv_range)
        self.PID_init()
        return config                    
                    
                    

    def process(self, rgb_img):
        twist = Twist()
        binary = []
        rgb_img = cv.resize(rgb_img, (640, 480))
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        #这里开始做语音识别结果判断：根据结果，取hsv的值以及停止位
        #rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
        self.command_result = self.spe.speech_read()
        self.spe.void_write(self.command_result)
        if self.command_result == 23 :
            self.model = "color_follow_line"
            print("red follow line")
            self.hsv_range =  [(0, 84, 131), (180, 253, 255)]
            #self.PID_init()
            self.dyn_update = True
           
            
        elif self.command_result == 24 :
            self.model = "color_follow_line"
            print("green follow line")
            self.hsv_range =  [(55, 105, 136), (95, 255, 255)]
            #self.PID_init()
            self.dyn_update = True
            
        elif self.command_result == 25 :
            self.model = "color_follow_line"   
            print("bule follow line")
            self.hsv_range =  [(55, 134, 218), (125, 253, 255)]
            #self.PID_init()
            self.dyn_update = True
            
            
        elif self.command_result == 26 :
            self.model = "color_follow_line"
            print("yellow follow line")
            #self.hsv_range =  [(17, 55, 187), (81, 255, 255)]
            self.hsv_range =  [(18, 45, 144), (125, 253, 255)]
            #self.PID_init()
            self.dyn_update = True
            
            

        elif self.command_result == 22 or self.command_result == 0 :
            self.model = "Stop"
            self.ros_ctrl.Joy_active = True
            self.ros_ctrl.pub_cmdVel.publish(Twist())
            print("Cancel color_follow_line")
        self.command_result = 999
        if self.dyn_update == True :
            params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                          'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                          'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
            self.dyn_client.update_configuration(params)
            self.dyn_update = False
        if  self.model == "color_follow_line":
            '''params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                          'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                          'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
            self.dyn_client.update_configuration(params)'''
            #self.PID_init()
            #write_HSV(self.hsv_text, self.hsv_range)
            #command_result = 999
            #self.ros_ctrl.Joy_active == False
            #self.model == "General"
            rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
            #self.hsv_range = ()
            if self.ros_ctrl.Joy_active == False :
                threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
            #self.model = "General"
            

            

        #这里识别停止位和手柄状态位self.ros_ctrl.Joy_active
        #这里做执行状态 execute()
        '''if self.ros_ctrl.Joy_active == True :
             threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
        else :
            self.ros_ctrl.pub_cmdVel.publish(Twist())'''
        return rgb_img, binary

    def execute(self, point_x, color_radius):
        #self.PID_init()
        if color_radius == 0: self.ros_ctrl.pub_cmdVel.publish(Twist())
        else:
            twist = Twist()
            b = Bool()
            [z_Pid, _] = self.PID_controller.update([(point_x - 320)/16, 0])
            if self.img_flip == True: twist.angular.z = -z_Pid
            else: twist.angular.z = +z_Pid
            twist.linear.x = self.linear*0.5
            if self.warning > 10:
                rospy.loginfo("Obstacles ahead !!!")
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.Buzzer_state = True
                b.data = True
                self.pub_Buzzer.publish(b)
            else:
                if self.Buzzer_state == True:
                    b.data = False
                    for i in range(3): self.pub_Buzzer.publish(b)
                    self.Buzzer_state = False
                self.ros_ctrl.pub_cmdVel.publish(twist)

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])
    
if __name__ == '__main__':
    print("start")
    line_detect = LineDetect()
    print("111")
    capture = cv.VideoCapture(0)
    cv_edition = cv.__version__
    if cv_edition[0]=='3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    #time.sleep(20)
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary = line_detect.process(frame)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:cv.imshow('frame', frame)
        if action == ord('q') or action == 113: break
    capture.release()
    cv.destroyAllWindows()
        
