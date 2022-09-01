#!/usr/bin/env python3
# coding=utf-8
# Version: V1.7.3
from flask import Flask, render_template, Response
import socket
import os
import time
import threading
import cv2 as cv
import sys
import struct
import inspect
import ctypes

from Rosmaster_Lib import Rosmaster

from camera_rosmaster import Rosmaster_Camera
from wifi_rosmaster import Rosmaster_WIFI


from gevent import pywsgi
from control_hover_app import hover_app


g_debug = False
hover_b = False
if len(sys.argv) > 1:
    if str(sys.argv[1]) == "debug":
        g_debug = True
    elif str(sys.argv[1]) == 'hover':
        hover = hover_app()
        hover_b = True
        
print("debug=", g_debug)


# 小车底层处理库
g_bot = Rosmaster(debug=g_debug)
# 启动线程接收串口数据
g_bot.create_receive_threading()
# 小车类型
g_car_type = 0

# 摄像头库
g_camera = Rosmaster_Camera(debug=g_debug)

# wifi库
g_wifi = Rosmaster_WIFI(g_bot, debug=g_debug)



g_ip_addr = "x.x.x.x"
g_tcp_ip = g_ip_addr

g_wifi_state = False
g_init = False
g_mode = 'Home'


app = Flask(__name__)


# 速度控制
g_speed_ctrl_xy = 100
g_speed_ctrl_z = 100
g_motor_speed = [0, 0, 0, 0]
g_car_stabilize_state = 0


# 阿克曼小车参数
AKM_DEFAULT_ANGLE = 100
AKM_LIMIT_ANGLE = 45
AKM_PWMSERVO_ID = 1

g_akm_def_angle = 100

# TCP未接收命令超时计数
g_tcp_except_count = 0

g_motor_speed = [0, 0, 0, 0]

# 返回单片机版本号, tcp=TCP服务对象
def return_bot_version(tcp):
    T_CARTYPE = g_car_type
    T_FUNC = 0x01
    T_LEN = 0x04
    version = int(g_bot.get_version() * 10)
    if version < 0:
        version = 0
    checknum = (6 + version) % 256
    data = "$%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, version, checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("Rosmaster Version:", version / 10.0)
        print("tcp send:", data)


# 返回电池电压
def return_battery_voltage(tcp):
    T_CARTYPE = g_car_type
    T_FUNC = 0x02
    T_LEN = 0x04
    vol = int(g_bot.get_battery_voltage() * 10) % 256
    if vol < 0:
        vol = 0
    checknum = (T_CARTYPE + T_FUNC + T_LEN + vol) % 256
    data = "$%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, vol, checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("voltage:", vol / 10.0)
        print("tcp send:", data)
    return vol / 10.0


# 返回机械臂角度
def return_arm_angle(tcp):
    T_CARTYPE = g_car_type
    T_FUNC = 0x14
    T_LEN = 0x1A
    if T_CARTYPE != g_bot.CARTYPE_X3_PLUS:
        return
    angle = g_bot.get_uart_servo_angle_array()
    if angle[1] != -1: angle[1] = 180 - angle[1]
    if angle[2] != -1: angle[2] = 180 - angle[2]
    if angle[3] != -1: angle[3] = 180 - angle[3]
    angle_s1 = bytearray(struct.pack('h', int(angle[0])))
    angle_s2 = bytearray(struct.pack('h', int(angle[1])))
    angle_s3 = bytearray(struct.pack('h', int(angle[2])))
    angle_s4 = bytearray(struct.pack('h', int(angle[3])))
    angle_s5 = bytearray(struct.pack('h', int(angle[4])))
    angle_s6 = bytearray(struct.pack('h', int(angle[5])))

    checknum = (T_CARTYPE + T_FUNC + T_LEN + angle_s1[0] + angle_s1[1] + angle_s2[0] + angle_s2[1] + angle_s3[0] + angle_s3[1]) % 256
    data = "$%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x#" % \
            (T_CARTYPE, T_FUNC, T_LEN, \
            angle_s1[0], angle_s1[1], angle_s2[0], angle_s2[1], angle_s3[0], angle_s3[1], \
            angle_s4[0], angle_s4[1], angle_s5[0], angle_s5[1], angle_s6[0], angle_s6[1], \
            checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("return angle_s1-s6:", angle)
        print("tcp send:", data)

# 返回机械臂设置中位的状态
def return_arm_offset_state(tcp, id, state):
    T_CARTYPE = g_car_type
    T_FUNC = 0x40
    T_LEN = 0x06
    checknum = (T_CARTYPE + T_FUNC + T_LEN + int(id) + int(state)) % 256
    data = "$%02x%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, int(id), int(state), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("return arm offset state:", id, state)
        print("tcp send:", data)

# 返回小车速度控制百分比
def return_car_speed(tcp, speed_xy, speed_z):
    T_CARTYPE = g_car_type
    T_FUNC = 0x16
    T_LEN = 0x06
    checknum = (T_CARTYPE + T_FUNC + T_LEN + int(speed_xy) + int(speed_z)) % 256
    data = "$%02x%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, int(speed_xy), int(speed_z), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("speed:", speed_xy, speed_z)
        print("tcp send:", data)

# 返回小车自稳状态
def return_car_stabilize(tcp, state):
    T_CARTYPE = g_car_type
    T_FUNC = 0x17
    T_LEN = 0x04
    checknum = (T_CARTYPE + T_FUNC + T_LEN + int(state)) % 256
    data = "$%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, int(state), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("stabilize:", int(state))
        print("tcp send:", data)


# 返回小车当前的XYZ速度
def return_car_current_speed(tcp):
    T_CARTYPE = g_car_type
    T_FUNC = 0x22
    T_LEN = 0x0E
    speed = g_bot.get_motion_data()
    num_x = int(speed[0]*100)
    num_y = int(speed[1]*100)
    num_z = int(speed[2]*20)
    speed_x = num_x.to_bytes(2, byteorder='little', signed=True)
    speed_y = num_y.to_bytes(2, byteorder='little', signed=True)
    speed_z = num_z.to_bytes(2, byteorder='little', signed=True)
    checknum = (T_CARTYPE + T_FUNC + T_LEN + speed_x[0] + speed_x[1] + speed_y[0] + speed_y[1] + speed_z[0] + speed_z[1]) % 256
    data = "$%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x#" % \
        (T_CARTYPE, T_FUNC, T_LEN, speed_x[0], speed_x[1], speed_y[0], speed_y[1], speed_z[0], speed_z[1], checknum)
    tcp.send(data.encode(encoding="utf-8"))
    # if g_debug:
    #     print("current_speed:", num_x, num_y, num_z)
    #     print("tcp send:", data)


# 返回阿克曼小车默认角度
def return_ackerman_angle(tcp, id, angle):
    T_CARTYPE = g_car_type
    T_FUNC = 0x50
    T_LEN = 0x06
    checknum = (T_CARTYPE + T_FUNC + T_LEN + int(id) + int(angle)) % 256
    data = "$%02x%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, int(id), int(angle), checknum)
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("ackerman_angle:", int(angle))
        print("tcp send:", data)



# 数值变换
def my_map(x, in_min, in_max, out_min, out_max):
    return (out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min


# 协议解析部分
def parse_data(sk_client, data):
    # print(data)
    global g_mode, g_bot, g_camera, g_car_type
    global g_akm_def_angle
    global g_motor_speed
    global g_speed_ctrl_xy, g_speed_ctrl_z
    global g_car_stabilize_state
    data_size = len(data)
    # 长度校验
    if data_size < 8:
        if g_debug:
            print("The data length is too short!", data_size)
        return
    if int(data[5:7], 16) != data_size-8:
        if g_debug:
            print("The data length error!", int(data[5:7], 16), data_size-8)
        return
    # 和校验
    checknum = 0
    num_checknum = int(data[data_size-3:data_size-1], 16)
    for i in range(0, data_size-4, 2):
        checknum = (int(data[1+i:3+i], 16) + checknum) % 256
        # print("check:", i, int(data[1+i:3+i], 16), checknum)
    if checknum != num_checknum:
        if g_debug:
            print("num_checknum error!", checknum, num_checknum)
            print("checksum error! cmd:0x%02x, calnum:%d, recvnum:%d" % (int(data[3:5], 16), checknum, num_checknum))
        return
    
    # 小车类型匹配
    num_carType = int(data[1:3], 16)
    if num_carType <= 0 or num_carType > 5:
        if g_debug:
            print("num_carType error!")
        return
    else:
        if g_car_type != num_carType:
            g_car_type = num_carType
            g_bot.set_car_type(g_car_type)
    
    # 解析命令标记
    cmd = data[3:5]
    if cmd == "0F":  # 回到主界面
        func = int(data[7:9])
        if g_debug:
            print("cmd func=", func)
        g_mode = 'Home'
        if func == 0:
            return_battery_voltage(sk_client)
        elif func == 1:
            return_car_speed(sk_client, g_speed_ctrl_xy, g_speed_ctrl_z)
            return_car_stabilize(sk_client, g_car_stabilize_state)
            g_mode = 'Standard'
        elif func == 2:
            return_car_current_speed(sk_client)
            g_mode = 'MecanumWheel'

    elif cmd == "01":  # 获取硬件版本号
        if g_debug:
            print("get version")
        return_bot_version(sk_client)

    elif cmd == "02":  # 获取电池电压
        if g_debug:
            print("get voltage")
        return_battery_voltage(sk_client)

    elif cmd == "10":  # 控制小车
        num_x = int(data[7:9], 16)
        num_y = int(data[9:11], 16)
        if num_x > 127:
            num_x = num_x - 256
        if num_y > 127:
            num_y = num_y - 256
        speed_x = num_y / 100.0
        speed_y = -num_x / 100.0
        if speed_x == 0 and speed_y == 0:
            g_bot.set_car_run(0, g_car_stabilize_state)
        else:    
            if g_car_type == g_bot.CARTYPE_R2:
                speed_y = my_map(speed_y, -1, 1, AKM_LIMIT_ANGLE/1000.0, AKM_LIMIT_ANGLE/-1000.0)
                # speed_z = my_map(speed_y, -1, 1, 3.0, -3.0)
                g_bot.set_car_motion(speed_x*1.8, speed_y, 0)
                # print('r2 : ',speed_x,speed_y)
            else:
                g_bot.set_car_motion(speed_x, speed_y, 0)
                # print('none : ',speed_x,speed_y)
                
        if g_debug:
            print("speed_x:%.2f, speed_y:%.2f" % (speed_x, speed_y))
        if hover_b:
            hover.get_xy(speed_x,speed_y)


    # 控制PWM舵机
    elif cmd == "11":
        num_id = int(data[7:9], 16)
        num_angle = int(data[9:11], 16)
        if g_debug:
            print("pwm servo id:%d, angle:%d" % (num_id, num_angle))
        if g_car_type == g_bot.CARTYPE_R2:
            angle_mini = (AKM_DEFAULT_ANGLE - AKM_LIMIT_ANGLE)
            angle_max = (AKM_DEFAULT_ANGLE + AKM_LIMIT_ANGLE)
            if num_angle < angle_mini:
                num_angle = angle_mini
            elif num_angle > angle_max:
                num_angle = angle_max
            g_bot.set_pwm_servo(num_id, num_angle)
        else:
            g_bot.set_pwm_servo(num_id, num_angle)


    # 控制机械臂
    elif cmd == "12":
        num_id = int(data[7:9], 16)
        num_angle_l = int(data[9:11], 16)
        num_angle_h = int(data[11:13], 16)
        uart_servo_angle = num_angle_h * 256 + num_angle_l
        if g_debug:
            print("uart servo id:%d, angle:%d" % (num_id, uart_servo_angle))
        if 1 < num_id < 5:
            uart_servo_angle = 180 - uart_servo_angle
        g_bot.set_uart_servo_angle(num_id, uart_servo_angle)


    # 设置蜂鸣器
    elif cmd == "13":
        num_state = int(data[7:9], 16)
        num_delay = int(data[9:11], 16)
        if g_debug:
            print("beep:%d, delay:%d" % (num_state, num_delay))
        delay_ms = 0
        if num_state > 0:
            if num_delay == 255:
                delay_ms = 1
            else:
                delay_ms = num_delay * 10
        g_bot.set_beep(delay_ms)

    # 读取机械臂舵机角度。
    elif cmd == "14":
        num_id = int(data[7:9], 16)
        if g_debug:
            print("read angle:%d" % num_id)
        if num_id == 6:
            return_arm_angle(sk_client)
    
    # 按键控制
    elif cmd == "15":
        num_dir = int(data[7:9], 16)
        if g_debug:
            print("btn ctl:%d" % num_dir)
        speed = 0
        if g_car_type == g_bot.CARTYPE_R2:
            if num_dir == 0:
                g_bot.set_car_run(0, g_car_stabilize_state)
            elif num_dir == 1 or num_dir == 2:
                speed = int(g_speed_ctrl_xy*1.8)
                g_bot.set_car_run(num_dir, speed)
            else:
                speed = int(g_speed_ctrl_z*1.8)
                g_bot.set_car_run(num_dir, speed, g_car_stabilize_state)
        else:    
            if num_dir == 0:
                g_bot.set_car_run(0, g_car_stabilize_state)
            elif num_dir == 5 or num_dir == 6:
                speed = g_speed_ctrl_z
                g_bot.set_car_run(num_dir, speed)
            else:
                speed = g_speed_ctrl_xy
                g_bot.set_car_run(num_dir, speed, g_car_stabilize_state)
        if g_debug:
            print("car speed:%.2f" % speed)
    
    # 控制速度
    elif cmd == '16':
        num_speed_xy = int(data[7:9], 16)
        num_speed_z = int(data[9:11], 16)
        if g_debug:
            print("speed ctl:%d, %d" % (num_speed_xy, num_speed_z))
        g_speed_ctrl_xy = num_speed_xy
        g_speed_ctrl_z = num_speed_z
        if g_speed_ctrl_xy > 100:
            g_speed_ctrl_xy = 100
        if g_speed_ctrl_xy < 0:
            g_speed_ctrl_xy = 0
        if g_speed_ctrl_z > 100:
            g_speed_ctrl_z = 100
        if g_speed_ctrl_z < 0:
            g_speed_ctrl_z = 0

    # 自稳开关
    elif cmd == '17':
        num_stab = int(data[7:9], 16)
        if g_debug:
            print("car stabilize:%d" % num_stab)
        if num_stab > 0:
            g_car_stabilize_state = 1
        else:
            g_car_stabilize_state = 0
    
    # 麦克纳姆轮控制  #### all_clearn
    elif cmd == '20':
        num_id = int(data[7:9], 16)
        num_speed = int(data[9:11], 16)
        if num_speed > 127:
            num_speed = num_speed - 256
        if g_debug:
            print("mecanum wheel ctrl:%d, %d" % (num_id, num_speed))
        if hover_b:
            hover.all_stop()
        if num_id >= 0 and num_id <= 4:
            if num_speed > 100:
                num_speed = 100
            if num_speed < -100:
                num_speed = -100
            if num_id == 0:
                g_motor_speed[0] = 0
                g_motor_speed[1] = 0
                g_motor_speed[2] = 0
                g_motor_speed[3] = 0
            else:
                g_motor_speed[num_id-1] = num_speed
            g_bot.set_motor(g_motor_speed[0], g_motor_speed[1], g_motor_speed[2], g_motor_speed[3])

    # 更新速度
    elif cmd == '21':
        num_speed_m1 = int(data[7:9], 16)
        num_speed_m2 = int(data[9:11], 16)
        num_speed_m3 = int(data[11:13], 16)
        num_speed_m4 = int(data[13:15], 16)
        if num_speed_m1 > 127:
            num_speed_m1 = num_speed_m1 - 256
        if num_speed_m2 > 127:
            num_speed_m2 = num_speed_m2 - 256
        if num_speed_m3 > 127:
            num_speed_m3 = num_speed_m3 - 256
        if num_speed_m4 > 127:
            num_speed_m4 = num_speed_m4 - 256
        if g_debug:
            print("mecanum wheel update:%d, %d, %d, %d" % (num_speed_m1, num_speed_m2, num_speed_m3, num_speed_m4))
        g_motor_speed[0] = num_speed_m1
        g_motor_speed[1] = num_speed_m2
        g_motor_speed[2] = num_speed_m3
        g_motor_speed[3] = num_speed_m4
        g_bot.set_motor(g_motor_speed[0], g_motor_speed[1], g_motor_speed[2], g_motor_speed[3])

    # 设置彩色灯带的颜色
    elif cmd == "30":
        num_id = int(data[7:9], 16)
        num_r = int(data[9:11], 16)
        num_g = int(data[11:13], 16)
        num_b = int(data[13:15], 16)
        if g_debug:
            print("lamp:%d, r:%d, g:%d, b:%d" % (num_id, num_r, num_g, num_b))
        g_bot.set_colorful_lamps(num_id, num_r, num_g, num_b)

    # 设置彩色灯带的特效
    elif cmd == "31":
        num_effect = int(data[7:9], 16)
        num_speed = int(data[9:11], 16)
        if g_debug:
            print("effect:%d, speed:%d" % (num_effect, num_speed))
        g_bot.set_colorful_effect(num_effect, num_speed, 255)

    # 设置彩色灯带的单色呼吸灯效果的颜色
    elif cmd == "32":
        num_color = int(data[7:9], 16)
        if g_debug:
            print("breath color:%d" % num_color)
        if num_color == 0:
            g_bot.set_colorful_effect(0, 255, 255)
        else:
            g_bot.set_colorful_effect(3, 255, num_color - 1)


    # 机械臂中位校准
    elif cmd == '40':
        num_cali = int(data[7:9], 16)
        if g_debug:
            print("arm offset:%d" % num_cali)
        if num_cali == 1:
            for i in range(6):
                id = int(i+1)
                state = g_bot.set_uart_servo_offset(id)
                return_arm_offset_state(sk_client, id, state)

    # 机械臂归中
    elif cmd == '41':
        num_verify = int(data[7:9], 16)
        if g_debug:
            print("arm up:%d" % num_verify)
        if num_verify == 1:
            g_bot.set_uart_servo_torque(True)
            time.sleep(.01)
            angle_array = [90, 90, 90, 90, 90, 180]
            g_bot.set_uart_servo_angle_array(angle_array)
            time.sleep(.5)

    # 机械臂扭矩开关
    elif cmd == '42':
        num_verify = int(data[7:9], 16)
        if g_debug:
            print("arm torque:%d" % num_verify)
        if num_verify == 0:
            g_bot.set_uart_servo_torque(False)
        else:
            g_bot.set_uart_servo_torque(True)


    # 读取阿克曼小车默认角度
    elif cmd == '50':
        num_id = int(data[7:9], 16)
        if g_debug:
            print("akm angle read:%d" % num_id)
        if num_id == AKM_PWMSERVO_ID:
            g_akm_def_angle = g_bot.get_akm_default_angle()
            return_ackerman_angle(sk_client, AKM_PWMSERVO_ID, g_akm_def_angle)

    # 临时修改阿克曼舵机的默认角度
    elif cmd == '51':
        num_id = int(data[7:9], 16)
        num_angle = int(data[9:11], 16)
        if g_debug:
            print("akm change angle:%d, %d" % (num_id, num_angle))
        if 60 <= num_angle <= 120 and num_id == AKM_PWMSERVO_ID:
            g_akm_def_angle = num_angle
            g_bot.set_akm_default_angle(num_angle)

    # 确认修改阿克曼小车默认角度
    elif cmd == '52':
        num_verify = int(data[7:9], 16)
        if g_debug:
            print("akm save angle:%d, %d" % (num_verify, g_akm_def_angle))
        if num_verify == 1:
            g_bot.set_akm_default_angle(g_akm_def_angle, True)
            time.sleep(.1)

    # 控制阿克曼小车前轮舵机角度
    elif cmd == '53':
        num_id = int(data[7:9], 16)
        num_angle = int(data[9:11], 16)
        if num_angle > 127:
            num_angle = num_angle - 256
        if g_debug:
            print("akm angle:%d, %d" % (num_id, num_angle))
        if -45 <= num_angle <= 45 and num_id == AKM_PWMSERVO_ID:
            g_bot.set_akm_steering_angle(num_angle)


# socket TCP通信建立
def start_tcp_server(ip, port):
    global g_init, g_tcp_except_count
    global g_socket, g_mode
    g_init = True
    if g_debug:
        print('start_tcp_server')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(None)
    sock.bind((ip, port))
    sock.listen(1)

    while True:
        print("Waiting for the client to connect!")
        tcp_state = 0
        g_tcp_except_count = 0
        g_socket, address = sock.accept()
        print("Connected, Client IP:", address)
        tcp_state = 1
        while True:
            try:
                tcp_state = 2
                cmd = g_socket.recv(1024).decode(encoding="utf-8")
                if not cmd:
                    break
                tcp_state = 3
                if g_debug:
                    print("   [-]cmd:{0}, len:{1}".format(cmd, len(cmd)))
                tcp_state = 4
                index1 = cmd.rfind("$")
                index2 = cmd.rfind("#")
                if index1 < 0 or index2 <= index1:
                    continue
                tcp_state = 5
                # parse_data(g_socket, cmd[index1:index2 + 1])
                g_tcp_except_count = 0
            except:
                if tcp_state == 2:
                    g_tcp_except_count += 1
                    if g_tcp_except_count >= 10:
                        g_tcp_except_count = 0
                        break
                else:
                    if g_debug:
                        print("!!!----TCP Except:%d-----!!!" % tcp_state)
                continue
            parse_data(g_socket, cmd[index1:index2 + 1])
        print("socket disconnected!")
        g_socket.close()
        g_mode = 'Home'


# 初始化TCP Socket
def init_tcp_socket():
    global g_ip_addr, g_tcp_ip
    if g_init:
        return
    while True:
        ip = g_wifi.get_ip_address()
        if ip == "x.x.x.x":
            g_tcp_ip = ip
            print("get ip address fail!")
            time.sleep(.5)
            continue
        if ip != "x.x.x.x":
            g_tcp_ip = ip
            print("TCP Service IP=", ip)
            break
    task_tcp = threading.Thread(target=start_tcp_server, name="task_tcp", args=(ip, 6000))
    task_tcp.setDaemon(True)
    task_tcp.start()
    if g_debug:
        print('-------------------Init TCP Socket!-------------------------')


# 根据状态机来运行程序包含视频流返回
def mode_handle():
    global g_mode, g_camera
    if g_debug:
        print("----------------------------mode_handle--------------------------")
    m_fps = 0
    t_start = time.time()
    while True:
        if g_mode == 'Standard':
            success, frame = g_camera.get_frame()
            m_fps = m_fps + 1
            fps = m_fps / (time.time() - t_start)

            text = "FPS:" + str(int(fps))
            if not success:
                g_camera.clear()
                m_fps = 0
                t_start = time.time()
                if g_debug:
                    print("-----The camera is reconnecting...")
                g_camera.reconnect()
                time.sleep(.5)
                continue
            cv.putText(frame, text, (10, 25), cv.FONT_HERSHEY_TRIPLEX, 0.8, (0, 200, 0), 1)
            ret, img_encode = cv.imencode('.jpg', frame)
            if ret:
                img_encode = img_encode.tobytes()
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + img_encode + b'\r\n')
        else:
            time.sleep(.1)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    global g_camera
    if g_debug:
        print("----------------------------video_feed--------------------------")
    return Response(mode_handle(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/init')
def init():
    init_tcp_socket()
    return render_template('init.html')



# 麦克纳姆轮返回速度的线程
def thread_mecanum():
    while True:
        if g_mode == 'MecanumWheel':
            return_car_current_speed(g_socket)
            time.sleep(.35)
        else:
            time.sleep(1)


if __name__ == '__main__':
    task_mecanum = threading.Thread(target=thread_mecanum, name="task_mecanum")
    task_mecanum.setDaemon(True)
    task_mecanum.start()

    init_tcp_socket()

    time.sleep(.1)
    for i in range(3):
        g_bot.set_beep(60)
        time.sleep(.2)

    print("Version:", g_bot.get_version())
    print("Waiting for connect to the APP!")

    try:
        server = pywsgi.WSGIServer(('0.0.0.0', 6500), app)
        server.serve_forever()
    except KeyboardInterrupt:
        g_bot.set_car_motion(0, 0, 0)
        g_bot.set_car_run(1,0)
        g_bot.set_beep(0)
        if g_debug:
            print("-----del g_bot-----")
        if hover_b:
            hover.close()
        del g_bot
