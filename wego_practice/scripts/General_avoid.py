from lib.morai_udp_parser import udp_parser,udp_sender
from lib.util import pathReader,findLocalPath,purePursuit,Point
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json
import numpy as np



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


        if obj_con !=[]:
            wait = float("inf")
            for i in range(len(obj_con)):
                if min(len_ob2car) < obj_con[i] + self.dis4change_path:
                    
                    wait = min(len_ob2car)
                    mem_idx = change_obj_id[0]-2
                if wait !=float("inf"):
                    self.pure_pursuit.getPath(local_path[mem_idx])
                    accel = 0.5


                else:
                    self.pure_pursuit.getPath(self.global_path)
                    

        elif obj_data != []:
            self.pure_pursuit.getPath(self.global_path)


        self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)
        steering_angle=self.pure_pursuit.steering_angle() 
        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])
        len_ob2car = []


if __name__ == "__main__":


    kicty=ppfinal()
    while True :
        pass

        

            

                    
                    
                
            

    
        
        


                

        
