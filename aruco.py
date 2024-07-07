import cv2
import cv2.aruco as aruco

cap=cv2.VideoCapture("aruco.mp4")
dic=aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
params=aruco.DetectorParameters()



while True:
    res,frame=cap.read()
    if not res:
        print("can't open file")
        break
    
    (corners,mark_id,rejected)=aruco.detectMarkers(frame,dic,parameters=params)
    
    if len(corners)>0:
        i=0
        for corn in corners:
            corn=corn.reshape(4,2)
            topl=corn[0]
            topr=corn[1]
            botr=corn[2]
            botl=corn[3]
            cv2.line(frame,[int(topl[0]),int(topl[1])],[int(topr[0]),int(topr[1])],(0,255,0),2)
            cv2.line(frame,[int(topr[0]),int(topr[1])],[int(botr[0]),int(botr[1])],(0,255,0),2)
            cv2.line(frame,[int(botr[0]),int(botr[1])],[int(botl[0]),int(botl[1])],(0,255,0),2)
            cv2.line(frame,[int(botl[0]),int(botl[1])],[int(topl[0]),int(topl[1])],(0,255,0),2)
            cv2.rectangle(frame,(int(topl[0])-4,int(topl[1])-4),(int(topl[0]+4),int(topl[1]+4)),(0,0,255),2,)
            id=mark_id[i]
            id=id[0]
            id=str(id)
            ids= 'id='+id
            cv2.putText(frame,ids,(int(topl[0]/2+botr[0]/2),int(topl[1]/2+botr[1]/2)),cv2.FONT_HERSHEY_DUPLEX,0.6,(255,0,0))
            i+=1
        
    
    
    cv2.imshow("frame",frame)
    cv2.waitKey(2)   
