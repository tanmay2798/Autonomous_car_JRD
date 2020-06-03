from __future__ import division

import cv2

import track
import detect
import detect2
import argparse
import sys
import time
import math
import numpy as np
#from statistics import mean 


parser = argparse.ArgumentParser()
parser.add_argument("--path", type=str, help="Video path")

def main(video_path,name):
    flag=True
    cap = cv2.VideoCapture(video_path)
    ret,frame = cap.read()   
    ticks = 0

    lt = track.LaneTracker(2, 0.1, 500)
    ld = detect.LaneDetector(180)
    ld2 = detect2.LaneDetector(180)
    height, width = frame.shape[:2]
    fps = int(8)    
    fourcc = cv2.VideoWriter_fourcc(*'MPEG')
    out = cv2.VideoWriter(name, fourcc, fps, (width,height))
    p=[(0,0,0,0),(0,0,0,0)]
    l2_old=0
    x_old=0
    m1=0
    m2=0
    m=0
    arr1=[]
    arr2=[]
    arr2d=[]
    value=0
    value_old=0
    i_2=0
    j_2=0
    for i in range(10):
        arr2d.append([])

    while cap.isOpened():
        precTick = ticks
        ticks = cv2.getTickCount()
        dt = (ticks - precTick) / cv2.getTickFrequency()
        #print(dt)
        ret, frame = cap.read()
        frame = frame[100:600,300:1100]
        predicted = lt.predict(dt)
        #time.sleep(0.02)
        f=frame[0:400]
       #hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)

	    # define range of white color in HSV
	    # change it according to your need !
	#lower_white = np.array([0,0,0], dtype=np.uint8)
	#upper_white = np.array([0,0,255], dtype=np.uint8)

	    # Threshold the HSV image to get only white colors
	#mask = cv2.inRange(hsv, lower_white, upper_white)
	    # Bitwise-AND mask and original image
	#res = cv2.bitwise_and(f,f, mask= mask)

	#cv2.imshow('frame',f)
	#cv2.imshow('mask',mask)
	#cv2.imshow('res',res)
        #cv2.imshow('ss',f)
        lanes = ld.detect(f)
        l2= ld2.detect(frame[100:600,200:500])

        if predicted is not None and l2 is not None:
            cv2.line(frame, (predicted[0][0], predicted[0][1]), (predicted[0][2], predicted[0][3]), (0, 0, 255), 5)
            cv2.line(frame, (predicted[1][0], predicted[1][1]), (predicted[1][2], predicted[1][3]), (0, 0, 255), 5)
            if flag:
                flag=False

                #cv2.line(frame, (((predicted[0][0]+predicted[1][0]))/2,((predicted[0][1]+predicted[1][1]))/2),(((predicted[0][2]+predicted[1][2]))/2,((predicted[0][3]+predicted[1][3]))/2), (0, 255, 0), 5)
                x1=(max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
                y=((predicted[1][1])+((predicted[0][3])))/2

                dif1 = (predicted[0][3][0]-predicted[0][1][0])/10
                dif2 = (predicted[1][1][0]-predicted[1][3][0])/10
                m=((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4
                m1=((predicted[0][1][0]- predicted[0][3][0])/(predicted[0][0][0]- predicted[0][2][0]))
                m2=((predicted[1][1][0]- predicted[1][3][0])/(predicted[1][0][0]- predicted[1][2][0]))

                arr=[]
                arr_angles = [] 
                for i in range(2,12):
                    arr.append([int(((i*dif1/m1+predicted[0][0][0])+(i*dif2/m2+predicted[1][2][0]))/2),int((predicted[0][1][0]+i*dif1+predicted[1][3][0]+i*dif2)/2)])

                if(len(arr2)<100):
                    arr2.append(l2+200)
                else:
                    arr2.pop(0)
                    arr2.append(l2+200)

                if(len(arr1)<200):
                    arr1.append(x1)
                else:
                    arr1.pop(0)
                    arr1.append(x1)

                for i in range(10):
                    if(len(arr2d[i])<200):
                        arr2d[i].append(arr[i][0])
                    else:
                        arr2d[i].pop(0)
                        arr2d[i].append(arr[i][0])                

                x2d=[]
                y2d=[]
                for i in range(10):
                    x2d.append(sum(arr2d[i])/len(arr2d[i]))
                    y2d.append(int((predicted[0][1][0]+i*dif1+predicted[1][3][0]+i*dif2)/2))
                    #arr_ang

                x=sum(arr1)/len(arr1)
                l2_avg=sum(arr2)/len(arr2)

               # print(arr2)
                #print(len(arr1))
                cv2.circle(frame, (x,((predicted[1][1])+((predicted[0][3])))/2),5,(255,0,0),-1)
                #cv2.rectangle(frame,(x+10,y+10), (x-10, y-10), (255,0,0),1)
                #cnt = np.float32([(x+10,y+10),(x+10,y-10),(x-10,y-10),(x-10,y+10)])
                #dist = cv2.pointPolygonTest(cnt,(int(l2_avg),int((predicted[1][1])+((predicted[0][3])))/2),False)
                #cv2.circle(frame, (((predicted[0][0]+predicted[1][2])+((predicted[0][2]+predicted[1][2])))/4,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4), 5,(0, 255, 0),-1)
                #cv2.circle(frame,(350,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4),5,(0,255,0),-1)
                cv2.circle(frame,(int(l2_avg),((predicted[1][1])+((predicted[0][3])))/2),5,(0,255,0),-1)
                m=((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4
                m1=((predicted[0][1][0]- predicted[0][3][0])/(predicted[0][0][0]- predicted[0][2][0]))
                m2=((predicted[1][1][0]- predicted[1][3][0])/(predicted[1][0][0]- predicted[1][2][0]))
                #print(m2)
                #dif = l2_avg+200 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
                for i in range(10):
                    dif = l2_avg-x2d[i]
                    dire = dif/(y2d[i]*(10-i))
                    arr_angles.append(round(-1*math.degrees(math.atan(dire)),2))
                    if(abs(value-value_old)>10):
                        value=value_old
                    if(round(-1*math.degrees(math.atan(dire)),2)<-10 or round(-1*math.degrees(math.atan(dire)),2)>10):
                        arr2d[i]=[]
                        arr1=[]
                
                
            for i in range (len(arr2d)-1):
                cv2.circle(frame, (int(x2d[i]), arr[i][1]) ,5,(255,0,0),-1)
                cv2.circle(frame,(int(l2_avg),arr[i][1]),5,(0,255,0),-1)
                #time.sleep(0.001)
            cv2.putText(frame , str(arr_angles[i_2]), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 2, cv2.LINE_AA)
            print(arr_angles[i_2],i_2)
            time.sleep(0.001)
            j_2=j_2+1
            #print(j_2)
            if j_2==10:
                i_2=i_2+1
                j_2=0
            if i_2==10:
                i_2=0
                flag=True


        lt.update(lanes)
        out.write(frame)
        cv2.imshow('', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    #args = parser.parse_args()
    main(sys.argv[1],sys.argv[2])
