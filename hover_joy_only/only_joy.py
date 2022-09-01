#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import Joy
from ardu_serial import Ardu

class joystick:
    def __init__(self):
        self.hover = Ardu()
        self.sp = '00'
        self.st = '00'
        self.x = None
        self.x_prev = None
        self.y = None
        self.y_prev = None
        self.joy = False
        self.yolo = False
        self.joy_dir = "0"
        self.var = 0
        self.hover.input('00')
        rospy.Subscriber('joy',Joy,self.buttoncallback,queue_size=1)
        print('done')
        
    def buttoncallback(self, joy_data):
        if not isinstance(joy_data,Joy): return
        '''
        :jetson joy_data:
            axes 8: [0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
            ## left joy
            axes[0] : + left/- right
            axes[1] : + front/- back
            ## right joy
            axes[2] : + left/- right
            axes[3] : + front/- back
         buttons 15:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                       A, B, '',X, Y, '','',R1,'',R2
            '''
        if joy_data.buttons[1] == 1:   ## B stop
            self.hover.input('00')
            print('stop')
            self.joy = False
        elif joy_data.buttons[0] == 1:   ## A mode joy
            self.hover.input('00')
            print('joy')
            self.joy = True
            
        if self.joy == True:
            x_data =  ( (round(joy_data.axes[1],1))*10 ) 
            y_data =  ( (round(joy_data.axes[0],1))*10 ) *-1
            x_data = 'g'+str(int(x_data*30))
            y_data = 't'+str(int(y_data*30))
            if (joy_data.axes[1] ==0) and (joy_data.axes[0] ==0) :
                x_data = 's0'
                y_data = 's0'
            self.hover.input(x_data)
            self.hover.input(y_data) 
        
if __name__ == "__main__":
    rospy.init_node('joy_ctrl')
    joy = joystick()
    try : rospy.spin()
    except KeyboardInterrupt:
        joy.hover.input('00')
        joy.hover.close()
    except :
        joy.hover.input('00')
        joy.hover.close()
        
        
            
            
        
        
        
