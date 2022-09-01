#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import os,sys

class Ardu():
    def __init__(self,port = "/dev/ttyACM0",baudrate = 115200):
        self.port = port
        self.stop = '00q'
        self.data = '00q'
        self.su_port = "sudo chmod +777 "+port
        os.system(self.su_port)
        try : self.ser = serial.Serial(port =self.port, baudrate = baudrate)
        except : 
            print('port error ')
            self.port = input('port : ')
            os.system(self.su_port)
            self.ser = serial.Serial(port =self.port, baudrate = baudrate)
            
        
    def read(self):
        return self.ser.read()
        
    def input(self,data):
        if self.ser.readable():
            self.data = data
            try :
                self.ser.write(self.data.encode('utf-8'))
                time.sleep(0.1)
                return True
                
                
            except KeyboardInterrupt :
                self.ser.write(self.stop.encode())
                self.ser.close()
                return False
            
            except  :
                self.ser.write(self.stop.encode())
                self.ser.close()
                return False
        else:
            print('check your port')
            return False
        
    def close(self):
        self.ser.close()
        time.sleep(0.1)



if __name__ == "__main__":
    hover = Ardu()
