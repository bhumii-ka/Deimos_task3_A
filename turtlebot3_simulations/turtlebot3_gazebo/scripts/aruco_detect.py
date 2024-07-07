#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dic=aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
params=aruco.DetectorParameters_create()
def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    (corners,mark_id,rejected)=aruco.detectMarkers(cv_image,dic,parameters=params)
    print(mark_id)
    if len(corners)>0:
        i=0
        for corn in corners:
            corn=corn.reshape(4,2)
            topl=corn[0]
            topr=corn[1]
            botr=corn[2]
            botl=corn[3]
            cv2.line(cv_image,(int(topl[0]),int(topl[1])),(int(topr[0]),int(topr[1])),(0,255,0),2)
            cv2.line(cv_image,(int(topr[0]),int(topr[1])),(int(botr[0]),int(botr[1])),(0,255,0),2)
            cv2.line(cv_image,(int(botr[0]),int(botr[1])),(int(botl[0]),int(botl[1])),(0,255,0),2)
            cv2.line(cv_image,(int(botl[0]),int(botl[1])),(int(topl[0]),int(topl[1])),(0,255,0),2)
            cv2.rectangle(cv_image,(int(topl[0])-8,int(topl[1])-8),(int(topl[0]+8),int(topl[1]+8)),(0,0,255),2,)
            id=mark_id[i]
            id=id[0]
            id=str(id)
            ids= 'id='+id
            cv2.putText(cv_image,ids,(int(topl[0]/2+botr[0]/2),int(topl[1]/2+botr[1]/2)),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255))
            i+=1
    cv_image=cv2.resize(cv_image, (660, 440)) 
    cv2.imshow("cv_image",cv_image)
    cv2.waitKey(1)          

rospy.init_node('aruco_detect')
rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
rospy.spin()