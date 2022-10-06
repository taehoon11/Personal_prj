from lib.morai_udp_parser import udp_parser,udp_sender
from lib.util import pathReader,findLocalPath,purePursuit,Point
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json
import numpy as np
import matplotlib.pyplot as plt


path = os.path.dirname( os.path.abspath( __file__ ) )  
len_ob2car = []
ctn = 0
local_path = []
obj_id = []
obj_con = []
mem_idx = -1
change_obj_id = []

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp) 

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]



class ppfinal :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.obj=udp_parser(user_ip, params["object_info_dst_port"],'erp_obj')
        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.safety_distance = 4
        self.dis4change_path = 5
        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read('test_path.txt') 
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
        global ctn
        global len_ob2car
        global local_path
        global obj_con
        global obj_id
        global mem_idx
        global change_obj_id

        self.timer=threading.Timer(0.001,self.main_loop)
        self.timer.start()
        
        status_data=self.status.get_data()
        obj_data=self.obj.get_data()
        position_x=status_data[12]
        position_y=status_data[13]
        position_z=status_data[14]
        heading=status_data[17]     
        velocity=status_data[18]

        ctrl_mode = 2 
        Gear = 4 
        cmd_type = 1     
        send_velocity = 0 
        acceleration = 0    
        accel=1
        brake=0

        if obj_data != []:
            change_obj_id= []
            for i in range(len(obj_data)):
                data = obj_data[i]
                change_obj_id.append(data[0]) ## id-2 = index
                len_ob2car.append(sqrt(pow((data[2] - position_x),2) + pow((data[3] - position_y),2))) 
                if data[0] not in obj_id: 
                    obj_id.append(data[0]) ## id-2 = index
                    Avoid_Radius = sqrt(pow(data[6],2)+pow(data[7],2))
                    obj_con.append(Avoid_Radius)
                    local_path.append(findLocalPath(self.global_path,Avoid_Radius,self.safety_distance,data[2],data[3]))
                    print(local_path[0])
                    x = []
                    y = []
                    for i in range(len(local_path[0])):
                        p = local_path[0][i]
                        x.append(p[0])
                        y.append(p[1])
                        print(sqrt(pow(p[0],2) + pow(p[1],2)))
                    



if __name__ == "__main__":


    kicty=ppfinal()
    while True :
        pass
