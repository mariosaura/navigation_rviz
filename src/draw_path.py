#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import utm

from math import pow
import math
import numpy as np

from itertools import count
import pandas as pd

def draw_path():

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    
    #FICHERO    
    fichero = "path_planning/csv_routes/path.csv"

    x = []
    y = []
    
    i=0
    coordenadas = pd.read_csv(fichero,delimiter=",")        #leo el csv
    coordenadas = np.array(coordenadas)                     #lo paso a array para quedarme con solo los numer
    print(coordenadas)

    #Coordenadas de referencia
    #x_ini,y_ini,Num2,Letra2 = utm.from_latlon(coordenadas[0,1],coordenadas[0,0])
    x_ini,y_ini,Num2,Letra2 = utm.from_latlon(38.275401 , -0.686178)
    

    while i < len(coordenadas):
        
        pointx,pointy,Num2,Letra2 = utm.from_latlon(coordenadas[i,1],coordenadas[i,0])
        pointx,pointy = pointx - x_ini , pointy - y_ini            #Me hago mi sistema local
        x.append(float(pointx))
        y.append(float(pointy))
        i = i+1




    points= Marker()
    lines = Marker()
    points.header.frame_id = "map"
    lines.header.frame_id = "map"
    points.header.stamp = rospy.Time.now()
    lines.header.stamp= rospy.Time.now()
    points.ns = "mario_points"
    lines.ns= "mario_points"
    points.action = Marker.ADD
    lines.action= Marker.ADD
    points.pose.orientation.w = 1.0
    lines.pose.orientation.w = 1.0
    
    points.id = 0
    lines.id = 1

    points.type = Marker.POINTS
    lines.type = Marker.LINE_STRIP

    points.scale.x = 2
    points.scale.y = 2

    lines.scale.x = 2
    lines.scale.x = 2

    # points.color.r = 1.0
    points.color.b = 1.0  #color morado
    points.color.a = 1.0

    lines.color.b = 0.5
    lines.color.g = 1.0
    lines.color.a = 1.0

    for i in range(len(coordenadas)):
        p = Point()
        p.x = x[i]
        p.y = y[i]
        p.z = 0
        points.points.append(p)
        lines.points.append(p)
        p.z += 1.0
    

    marker_pub.publish(points)
    marker_pub.publish(lines)


