#!/usr/bin/env python3
# coding=utf-8
# Version: no control
from flask import Flask, render_template, Response
import socket
import os
import time
import threading
import cv2 as cv
import sys



from camera_rosmaster import Rosmaster_Camera

from gevent import pywsgi


g_debug = False
if len(sys.argv) > 1:
    if str(sys.argv[1]) == "debug":
        g_debug = True
print("debug=", g_debug)


# 摄像头库
g_camera = Rosmaster_Camera(debug=g_debug)


g_ip_addr = "x.x.x.x"
g_tcp_ip = g_ip_addr

g_wifi_state = False
g_init = False
g_mode = 'Home'


app = Flask(__name__)

def get_ip_address():
        ip = os.popen(
            "/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0: ip.find('\n')]
        if(ip == ''):
            ip = os.popen(
                "/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
            ip = ip[0: ip.find('\n')]
            if(ip == ''):
                ip = 'x.x.x.x'
        if len(ip) > 15:
            ip = 'x.x.x.x'
        return ip

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

        print("socket disconnected!")
        g_socket.close()
        g_mode = 'Home'


# 初始化TCP Socket
def init_tcp_socket():
    global g_ip_addr, g_tcp_ip
    if g_init:
        return
    while True:
        ip = get_ip_address()
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
        success, frame = g_camera.get_frame()
        m_fps = m_fps + 1
        fps = m_fps / (time.time() - t_start)

        text = "FPS:" + str(int(fps))
        if not success:
            g_camera.clear()
            m_fps = 0
            t_start = time.time()
            # if g_debug:
            #     print("-----The camera is reconnecting...")
            g_camera.reconnect()
            time.sleep(.5)
            continue
        cv.putText(frame, text, (10, 25), cv.FONT_HERSHEY_TRIPLEX, 0.8, (0, 200, 0), 1)
        ret, img_encode = cv.imencode('.jpg', frame)
        if ret:
            img_encode = img_encode.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + img_encode + b'\r\n')



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



if __name__ == '__main__':

    init_tcp_socket()

    try:
        server = pywsgi.WSGIServer(('0.0.0.0', 6500), app)
        server.serve_forever()
    except KeyboardInterrupt:
        if g_debug:
            print("-----del g_bot-----")
