import rospy
#importo la funcion del codigo de sergio
from path_planning.main import main_funtion
#importo mi funcion de dibujar
from draw_path import draw_path

from std_msgs.msg import String
#obtengo el tipo de mensaje q usa el topico 
from geometry_msgs.msg import PointStamped 


import utm
import time
import tf
import random
import numpy as np

#para draw_path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from math import pow
import math
import numpy as np

from itertools import count
import pandas as pd

class get_pose_current():
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.nuevo_dato = 0
    def listener(self):
        self.sub=rospy.Subscriber("/clicked_point" , PointStamped, self.callback_pos)

    def callback_pos(self,data):
        #el mensaje tiene un header y un point. dentro del point tengo las variables http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html
        self.x = data.point.x
        self.y = data.point.y
        self.nuevo_dato = 1





if __name__ == '__main__':

    rospy.init_node('clicked_point_subscriber',anonymous=True)

    pose = get_pose_current()
    pose.listener()

    r = rospy.Rate(10)

    #llamo a la funcion q he importado
    #main_funtion(38.274942, -0.686878,38.276335, -0.686282)

    #Referencia
    x_ini,y_ini,Num,Letra=utm.from_latlon(38.275401,-0.686178)  
    draw_path()  #Inicio markers (la primera vez no la va a escribir)
        
        # 1 paso las coordenadas de ref de rviz_satellite a utm
        # 2 les sumo el desplazamiento en x e y del punto q he marcado con clicked point
        # 3 las vuelvo a pasar a lat, lon
        # 4 se las mando al codigo de sergio para q haga el camino
        # 5 Lanzo el controlador PD

    while not rospy.is_shutdown():
        x = pose.x
        y = pose.y

        

        if( pose.nuevo_dato == 1):
            global nuevo_dato
            pose.nuevo_dato = 0

            utm_x = x_ini + x  
            utm_y = y_ini + y 

            print("x_ini,y_ini: ",x_ini, y_ini)
            print("x,y: ",x, y)
            print(utm_x , utm_y)
            
            lat,lon = utm.to_latlon(utm_x, utm_y  , Num,Letra)
            print("lat,lon:",lat, lon)
            main_funtion(38.275401 , -0.686178,lat,lon)
            
            
            draw_path()

            

        
        r.sleep()


