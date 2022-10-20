import os
from math import cos,sin,sqrt,pow,atan2,acos,pi

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




class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=2
        self.lfd=5
        self.min_lfd=5
        self.max_lfd=30
        self.steering=0
        
    def getPath(self,path):
        self.path=path 
 
    
    
    def getEgoStatus(self,position_x,position_y,position_z,velocity,heading):

        self.current_vel=velocity  #kph
        self.vehicle_yaw=heading/180*pi   # rad
        self.current_postion.x=position_x
        self.current_postion.y=position_y
        self.current_postion.z=position_z




    def steering_angle(self):
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        if self.current_vel*0.3 < self.min_lfd:
            self.lfd = self.min_lfd
        elif self.current_vel*0.3 > self.max_lfd:
            self.lfd = self.max_lfd-1
        else:
            self.lfd = self.current_vel * 0.3

        for i in range(len(self.path)) :
            pathpoint = self.path[i]
            rel_x= pathpoint[0] - vehicle_position.x
            rel_y= pathpoint[1] - vehicle_position.y
            s = sqrt(rel_x*rel_x + rel_y*rel_y)
            if s > self.min_lfd and s < self.max_lfd and s > self.lfd:
                dot_x = rel_x*cos(self.vehicle_yaw) + rel_y*sin(self.vehicle_yaw)
                dot_y = rel_x*sin(self.vehicle_yaw) - rel_y*cos(self.vehicle_yaw)
                if dot_x > 0 :             
                    alpha=atan2(dot_y,dot_x)
                    self.forward_point=pathpoint
                    self.is_look_forward_point=True
                    break
              

        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(alpha)),s)
            return self.steering #deg
        else : 
            print("There is no waypoint at front")
            return 0
        


def findLocalPath(ref_path,Avoid_Radius,safety_distance,obj_pos_x,obj_pos_y):
    out_path=[]
    threshold = []
    t_idx_min = 0
    t_idx_max = 0

    for i in range(len(ref_path)): 
        dx = ref_path[i][0] - obj_pos_x 
        dy = ref_path[i][1] - obj_pos_y
        dis = sqrt(dx*dx + dy*dy)    
        if dis < Avoid_Radius+safety_distance:  
            threshold.append(i)     
    t_idx_min = min(threshold)      
    t_idx_max = max(threshold)      

    for i in range(t_idx_min,t_idx_max+200): 
        pose = []
        dist = sqrt(pow((ref_path[i][0]-obj_pos_x),2)+pow((ref_path[i][1]-obj_pos_y),2)) 
        if i < t_idx_max and dist <= Avoid_Radius+safety_distance:
            newpoint_x = ref_path[i][0] - obj_pos_x  
            newpoint_y = ref_path[i][1] - obj_pos_y 
            slid_ang = atan2(newpoint_y,newpoint_x)      
            newpoint_x = (Avoid_Radius+safety_distance)*cos(slid_ang) + obj_pos_x
            newpoint_y = (Avoid_Radius+safety_distance)*sin(slid_ang) + obj_pos_y
            
            pose.append(newpoint_x) 
            pose.append(newpoint_y)
            out_path.append(pose)      
        else:
            pose.append(ref_path[i][0])
            pose.append(ref_path[i][1])              
            out_path.append(pose)

    return out_path 

def findLocalPath2(ref_path,Avoid_Radius,safety_distance,obj_pos_x,obj_pos_y):
    out_path=[]
    threshold = []

    min_dis = float("inf")
    min_idx = 0
    t_idx_min = 0
    t_idx_max = 0

    for i in range(len(ref_path)): 
        dx = ref_path[i][0] - obj_pos_x 
        dy = ref_path[i][1] - obj_pos_y
        dis = sqrt(dx*dx + dy*dy)   
        if dis < Avoid_Radius+safety_distance:    
            threshold.append(i)
            if dis < min_dis:
                min_dis = dis
                min_idx = i

    t_idx_min = min(threshold)      
    t_idx_max = max(threshold)

    min_data= []
    max_data = []
    rel_min_obj_x = ref_path[t_idx_min][0] - obj_pos_x  
    rel_min_obj_y = ref_path[t_idx_min][1] - obj_pos_y
    slid_ang_min = atan2(rel_min_obj_y,rel_min_obj_x)

    rel_max_obj_x = ref_path[t_idx_max][0] - obj_pos_x
    rel_max_obj_y = ref_path[t_idx_max][1] - obj_pos_y
    slid_ang_max = atan2(rel_max_obj_y,rel_max_obj_x)
    
    min_data.append((Avoid_Radius+safety_distance)*cos(slid_ang_min) + obj_pos_x)
    min_data.append((Avoid_Radius+safety_distance)*sin(slid_ang_min) + obj_pos_y)
    out_path.append(min_data)

    max_data.append((Avoid_Radius+safety_distance)*cos(slid_ang_max) + obj_pos_x)
    max_data.append((Avoid_Radius+safety_distance)*sin(slid_ang_max) + obj_pos_y)

    gamma = atan2(max_data[1]-min_data[1],max_data[0]-min_data[0])
    min_idx_deg = atan2(min_data[1]-obj_pos_y,min_data[0]-obj_pos_x)*180/pi
    max_idx_deg = atan2(max_data[1]-obj_pos_y,max_data[0]-obj_pos_x)*180/pi
    #print(min_idx_deg)
    #print(max_idx_deg)
    if min_idx_deg < 0:
        min_idx_deg = 360 + min_idx_deg
    if max_idx_deg < 0:
        max_idx_deg = 360 + max_idx_deg

    #print(min_idx_deg)
    #print(max_idx_deg)

    mini_vector = [ref_path[min_idx][0],ref_path[min_idx][1]]
    dot_p = -1*mini_vector[0]*sin(gamma) + mini_vector[1]*cos(gamma)

    if dot_p >= 0:
        if min_idx_deg > max_idx_deg:
            ctn = int(min_idx_deg) - int(max_idx_deg)
        else:
            ctn = int(min_idx_deg) + 360 - int(max_idx_deg)

        for j in range(1,ctn):
            pose = []
            ang = (min_idx_deg - j)*pi/180
            x = (Avoid_Radius+safety_distance)*sin(ang)
            y = (Avoid_Radius+safety_distance)*cos(ang)
            x = x + obj_pos_x
            y = y + obj_pos_y
            pose.append(x)
            pose.append(y)
            out_path.append(pose)

    elif dot_p < 0:
        if min_idx_deg > max_idx_deg:
            ctn = 360 - int(min_idx_deg) + int(max_idx_deg)
        else:
            ctn = int(max_idx_deg) - int(min_idx_deg)

        for i in range(1,ctn):
            pose = []
            ang = (min_idx_deg + i)*pi/180
            x = (Avoid_Radius+safety_distance)*sin(ang)
            y = (Avoid_Radius+safety_distance)*cos(ang)
            x = x + obj_pos_x
            y = y + obj_pos_y
            pose.append(x)
            pose.append(y)
            out_path.append(pose)

    out_path.append(max_data)
    #print(ctn)
    for i in range(t_idx_max+1,t_idx_max+100): 
        pose = []
        pose.append(ref_path[i][0])
        pose.append(ref_path[i][1])              
        out_path.append(pose)

    return out_path 