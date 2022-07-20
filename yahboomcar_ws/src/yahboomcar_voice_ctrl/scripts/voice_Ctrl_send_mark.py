#!/usr/bin/env python3
# coding: utf-8

import Speech_Lib
import time
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

spe = Speech_Lib.Speech()
pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
def voice_pub_goal():
    rospy.init_node('voice_pub_goal_publisher', anonymous=True) # ROS节点初始化
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    # The location of the target point

    # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
    
    
    
    while not rospy.is_shutdown():  
        speech_r = spe.speech_read()

        if speech_r == 19 : 
            print("goal to one")
            spe.void_write(speech_r)
            pose.pose.position.x = 1.73812913895
            pose.pose.position.y =  -1.13534617424
            pose.pose.orientation.z = 0.943704923435
            pose.pose.orientation.w = 0.330788478465
            pub_goal.publish(pose)

        elif speech_r == 20 :
            print("goal to tow")
            spe.void_write(speech_r)
            pose.pose.position.x = -0.305977225304
            pose.pose.position.y = 0.591488838196
            pose.pose.orientation.z = 0.911800693481
            pose.pose.orientation.w = -0.410633042226
            pub_goal.publish(pose)
            
        elif speech_r == 21 :
            print("goal to three")
            spe.void_write(speech_r)
            pose.pose.position.x =  -1.32170200348
            pose.pose.position.y = -0.522812485695
            pose.pose.orientation.z =  -0.333171190339
            pose.pose.orientation.w =  0.942866352103
            pub_goal.publish(pose)

        elif speech_r == 32 :
            print("goal to four")
            spe.void_write(speech_r)
            pose.pose.position.x =  0.741197705269
            pose.pose.position.y = -2.30312108994
            pose.pose.orientation.z =  0.406148383793
            pose.pose.orientation.w =  0.913807140671
            pub_goal.publish(pose)

        elif speech_r == 33 :
            print("goal to Origin")
            spe.void_write(speech_r)
            pose.pose.position.x =  0.324319899082
            pose.pose.position.y = -0.916470825672
            pose.pose.orientation.z =  0.406322044965
            pose.pose.orientation.w =  0.913729935908
            pub_goal.publish(pose)
        elif speech_r == 0 :
            pub_cmdVel.publish(Twist())
        rate.sleep()
if __name__ == '__main__':
    try:
        voice_pub_goal()
    except rospy.ROSInterruptException:
        pass
    
            
            
