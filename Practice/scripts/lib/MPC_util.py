#demo
import os
from math import cos,sin,sqrt,pow,atan2,acos,pi,tan
import numpy as np

class Point() :
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class pathReader :

    def __init__(self):
       self.file_path=os.path.dirname( os.path.abspath( __file__ ) )
       self.file_path = os.path.normpath(os.path.join(self.file_path, '../..'))
 


    def read(self,file_name):
        out_path=[]
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')

        line=openFile.readlines()

        for i in line :
            pose=[]
            postion=i.split()
            pose.append(float(postion[0]))
            pose.append(float(postion[1]))
            pose.append(float(postion[2]))
            out_path.append(pose)
            
        openFile.close()
        return out_path




class MPC :
    def __init__(self):
        self.control_gain = 2
        self.forward_point=Point()
        self.current_postion=Point()
        self.vehicle_length=2
        self.min_steer=-28*pi/180
        self.max_steer=28*pi/180
        self.steering=0
        
    def getPath(self,path):
        self.path=path 
 
    
    
    def getEgoStatus(self,position_x,position_y,position_z,velocity,heading,steering):

        self.current_vel=velocity  #mps
        self.vehicle_yaw=heading/180*pi # rad
        self.current_postion.x=position_x
        self.current_postion.y=position_y
        self.current_postion.z=position_z
        self.steering = steering




    def steering_angle(self):
        vehicle_position=self.current_postion
        self.is_look_forward_point= False
        ref_pos = []
        ref_ori = []
        cost = []
        
        look_len = int(self.current_vel*0.4)
        if look_len < 5:
            look_len = 5

        for i in range(look_len):
            ref_pos.append(self.path[i])
            tangency = atan2(self.path[i+1][1] - self.path[i][1], self.path[i+1][0] - self.path[i][0]) - self.vehicle_yaw
            ref_ori.append(tangency)

        x = vehicle_position.x
        y = vehicle_position.y
        heading = self.vehicle_yaw
        velocity = self.current_vel
        steering = self.steering
        dt = 0.5

        for i in range(-28,29):
            cost_tot = 0
            steer = i*pi/180
            for j in range(look_len):
                x = x + velocity*cos(steer+heading)*dt
                y = y + velocity*cos(steer+heading)*dt
                heading = heading + velocity*tan(steer)/self.vehicle_length*dt
                steer = 2*steering - steer
                ori_err = 100000*pow(heading-ref_ori[j],2)
                pos_err = pow(x-ref_pos[j][0],2) + pow(y-ref_pos[j][1],2)
                cost_tot = ori_err + pos_err
            cost.append(cost_tot)

        

        if cost != []:
            min_idx = 0
            min_cost = float("inf")
            for i in range(len(cost)):
                if cost[i] < min_cost:
                    min_cost = cost[i]
                    min_idx = i

        self.steering = (min_idx - 28)*pi/180
        print(min_idx)
        return -1*self.steering
        
def findLocalPath(ref_path,position_x,position_y):
    out_path=[]
    current_x=position_x
    current_y=position_y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path)) :
        dx=current_x - ref_path[i][0]
        dy=current_y - ref_path[i][1]
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i

    if current_waypoint+50 > len(ref_path) :
        last_local_waypoint= len(ref_path)
    else :
        last_local_waypoint=current_waypoint+50

    for i in range(current_waypoint,last_local_waypoint) :
        pose=[]
        pose.append(ref_path[i][0])
        pose.append(ref_path[i][1])
        out_path.append(pose)

    return out_path,current_waypoint