from lib.morai_udp_parser import udp_parser,udp_sender # 정리된 데이터를 사용하기 위한 import
from math import cos,sin,sqrt,pow,atan2,pi 
import time
import threading
import os,json


path = os.path.dirname( os.path.abspath( __file__ ) )


with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]

class go_straight :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.obj=udp_parser(user_ip, params["object_info_dst_port"],'erp_obj')
        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.car_len = 2  # 자동차의 길이
        self.safe_dis = 2  # 안전거리
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
        
        status_data=self.status.get_data() # 차량 상태 정보 불러오기
        obj_data=self.obj.get_data() # 장애물 정보 불러오기
        obj_data = obj_data[0]
# 불러온 정보로 부터 원하는 값 추출(lib 폴더의 morai__udp_parser에 가면 인덱스별 변수가 명시되어있다.)
        obj_pos_x = obj_data[2]      # 장애물 x 좌표
        obj_pos_y = obj_data[3]      # 장애물 y 좌표
        obj_size_x = obj_data[6]     # 장애물 x 축 방향 길이(가로)
        obj_size_y = obj_data[7]     # 장애물 y 축 방향 길이(세로)
        position_x=status_data[12]   # 차량의 현재 x 좌표
        position_y=status_data[13]   # 차량의 현재 y 좌표

        # 각각 차량과 장애물의 거리, 정지해야 하는 거리에 대하여 변수를 지정해 둔 것
        car2obj_dis = sqrt(pow((obj_pos_x - position_x),2) + pow((obj_pos_y - position_y),2))
        threshold = sqrt(pow(obj_size_x,2)+pow(obj_size_y,2)) + self.car_len + self.safe_dis
        
        # cmd를 통하여 자동차에 명령을 내릴 모드, 기어, 속도, 가속도 등의 변수


        ctrl_mode = 2 # 2 = AutoMode / 1 = KeyBoard
        Gear = 4 # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 1 # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration        
        send_velocity = 0 #cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0 #cmd_type이 3일때 원하는 가속도를 넣어준다.
        accel=1
        brake=0
        steering_angle=0


        if car2obj_dis < threshold:
            Gear = 1
            print("Object is close >> Stop")

        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])


if __name__ == "__main__":


    kicty=go_straight()
    while True :
        pass