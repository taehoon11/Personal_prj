import os
from math import cos,sin,sqrt,pow,atan2,acos,pi
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




class Stanley :
    def __init__(self):
        self.control_gain = 2
        self.forward_point=Point()
        self.current_postion=Point()
        self.vehicle_length=2
        self.min_steer=-30*pi/180
        self.max_steer=30*pi/180
        self.steering=0
        
    def getPath(self,path):
        self.path=path 
 
    
    
    def getEgoStatus(self,position_x,position_y,position_z,velocity,heading):

        self.current_vel=velocity  #mps
        self.vehicle_yaw=heading/180*pi # rad
        self.current_postion.x=position_x
        self.current_postion.y=position_y
        self.current_postion.z=position_z




    def steering_angle(self):
        vehicle_position=self.current_postion
        self.is_look_forward_point= False
        min_dist = float("inf")
        min_idx = 0

        front_x = vehicle_position.x + self.vehicle_length*cos(self.vehicle_yaw)
        front_y = vehicle_position.y + self.vehicle_length*sin(self.vehicle_yaw)

        for i in range(len(self.path)):
            wp = self.path[i]
            dx = front_x - wp[0]
            dy = front_y - wp[1]
            dist = sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        
        if min_idx < len(self.path)-2:
            tangency = atan2(self.path[min_idx+1][1]-self.path[min_idx][1],self.path[min_idx+1][0]-self.path[min_idx][0])
        else:
            tangency = atan2(self.path[min_idx][1]-self.path[min_idx-1][1],self.path[min_idx][0]-self.path[min_idx-1][0])

        psi = tangency - self.vehicle_yaw
        vectorr = [cos(self.vehicle_yaw + pi/2), sin(self.vehicle_yaw + pi/2)]
        dx = self.path[min_idx][0] - front_x
        dy = self.path[min_idx][1] - front_y
        cte = np.dot([dx,dy],vectorr)
        cte_term = atan2(self.control_gain*cte,self.current_vel)
        print(cte_term)
        #print(min_dist)
        self.steering = psi + cte_term
        #print(self.steering)

        while self.steering > pi:
            self.steering -= 2.0*pi
        
        while self.steering < -pi:
            self.steering += 2.0*pi

        self.steering = -1*np.clip(self.steering,self.min_steer,self.max_steer)
        #print(self.steering)

        return self.steering
        
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