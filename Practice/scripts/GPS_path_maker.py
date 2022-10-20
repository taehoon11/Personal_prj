from lib.morai_udp_parser import udp_parser
from lib.GPS_util import UDP_GPS_Parser
from pyproj import Transformer
import time
import threading
from math import cos,sin,sqrt,pow,atan2,pi
import os,json

path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
status_port = params["vehicle_status_dst_port"]
path_folder_name = params["make_path_folder_name"]
path_file_name = params["make_path_file_name"]
gps_port = params["gps_dst_port"]

class path_maker :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.file_path=os.path.dirname( os.path.abspath( __file__ ) )
        self.file_path = os.path.normpath(os.path.join(self.file_path, '..'))
        self.gps_parser=UDP_GPS_Parser(user_ip, gps_port,'GPRMC')
        
        self.full_path = self.file_path+'/'+path_folder_name+'/'+path_file_name
        

        self.prev_x = 0
        self.prev_y = 0
        
        self._is_status=False
        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True

        self.main_loop()
    
    def main_loop(self):
        self.timer=threading.Timer(0.10,self.main_loop)
        self.timer.start()
        f=open(self.full_path, 'a')
        latitude= self.gps_parser.parsed_data[0]
        longitude= self.gps_parser.parsed_data[1]


        transformer = Transformer.from_crs("epsg:4326", "epsg:5186")
        y,x = transformer.transform(latitude,longitude)
        
        distance = sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        if distance > 0.3 :
            data = '{}\t{}\n'.format(x,y)
            f.write(data)
            self.prev_x = x
            self.prev_y = y
            f.close()



if __name__ == "__main__":

    path=path_maker()
    while True :
        pass