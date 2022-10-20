#!/usr/bin/env python
# -*- coding: utf-8 -*-
from lib.GPS_util import UDP_GPS_Parser
from lib.morai_udp_parser import udp_parser,udp_sender
from lib.GPS_PT import pathReader,purePursuit,Point
from pyproj import Transformer
import time
import threading
import os,json


path = os.path.dirname( os.path.abspath( __file__ ) )  # current file's path


with open(os.path.join(path,("params.json")),'r') as fp :  # current path + file name
    params = json.load(fp) 

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]
gps_port = params["gps_dst_port"]


class ppfinal :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.gps_parser=UDP_GPS_Parser(user_ip, gps_port,'GPRMC')
        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read('test_path.txt')  # read method >> load x,y coord of global path
        self.pure_pursuit=purePursuit() 
  

        self._is_status=False
        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True


        self.main_loop()


    
    def main_loop(self):
        self.timer=threading.Timer(0.001,self.main_loop)
        self.timer.start()
        
        status_data=self.status.get_data()
        latitude= self.gps_parser.parsed_data[0]
        longitude= self.gps_parser.parsed_data[1]

        transformer = Transformer.from_crs("epsg:4326", "epsg:5186")
        position_y,position_x = transformer.transform(latitude,longitude)

        position_z=status_data[14]
        heading=status_data[17]     # degree
        velocity=status_data[18]
        
        self.pure_pursuit.getPath(self.global_path)
        self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)

        

        ctrl_mode = 2 # 2 = AutoMode / 1 = KeyBoard
        Gear = 4 # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 1 # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration        
        send_velocity = 0 #cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0 #cmd_type이 3일때 원하는 가속도를 넣어준다.     
        accel=1
        brake=0

        steering_angle=self.pure_pursuit.steering_angle() 
        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])
        

      

if __name__ == "__main__":


    kicty=ppfinal()
    while True :
        pass
 