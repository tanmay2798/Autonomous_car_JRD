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
    value=0
    value_old=0
    while cap.isOpened():
    	#time.sleep(0.5)
        precTick = ticks
        ticks = cv2.getTickCount()
        dt = (ticks - precTick) / cv2.getTickFrequency()
        #print(dt)
        ret, frame = cap.read()
        frame = frame[100:600,300:1100]
        predicted = lt.predict(dt)
        #time.sleep(0.02)
        f=frame[0:400]
        #cv2.imshow('ss',f)
        lanes = ld.detect(f)
        l2= ld2.detect(frame[100:600,200:500])
        
        #print(l2[2])
        #print('hikkkkkkkkkkkkkkkkkkkkkkkkkkkkk')
        #print(lanes)
        '''if lanes is not None:
            for l in lanes:
                if l is not None:
                    for x1,y1,x2,y2 in l:
                        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)'''

        if predicted is not None and l2 is not None:
            #print(abs(l2-l2_old))
            '''if(abs(l2-l2_old)<100):
                l2=l2_old
            else:
                l2_old=l2'''



            cv2.line(frame, (predicted[0][0], predicted[0][1]), (predicted[0][2], predicted[0][3]), (0, 0, 255), 5)
            cv2.line(frame, (predicted[1][0], predicted[1][1]), (predicted[1][2], predicted[1][3]), (0, 0, 255), 5)
            #cv2.line(frame, (((predicted[0][0]+predicted[1][0]))/2,((predicted[0][1]+predicted[1][1]))/2),(((predicted[0][2]+predicted[1][2]))/2,((predicted[0][3]+predicted[1][3]))/2), (0, 255, 0), 5)
            x1=(max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
            y=((predicted[1][1])+((predicted[0][3])))/2
            
            '''if(abs(x-x_old)<50):
            	x=x_old
            else:
            	x_old=x'''

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

            x=sum(arr1)/len(arr1)
            l2_avg=sum(arr2)/len(arr2)

           # print(arr2)
            #print(len(arr1))
            cv2.circle(frame, (x,((predicted[1][1])+((predicted[0][3])))/2),5,(255,0,0),-1)
            cv2.rectangle(frame,(x+10,y+10), (x-10, y-10), (255,0,0),1)
            cnt = np.float32([(x+10,y+10),(x+10,y-10),(x-10,y-10),(x-10,y+10)])
            dist = cv2.pointPolygonTest(cnt,(int(l2_avg),int((predicted[1][1])+((predicted[0][3])))/2),False)
            #cv2.circle(frame, (((predicted[0][0]+predicted[1][2])+((predicted[0][2]+predicted[1][2])))/4,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4), 5,(0, 255, 0),-1)
            #cv2.circle(frame,(350,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4),5,(0,255,0),-1)
            cv2.circle(frame,(int(l2_avg),((predicted[1][1])+((predicted[0][3])))/2),5,(0,255,0),-1)
            m=((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4
            m1=((predicted[0][1][0]- predicted[0][3][0])/(predicted[0][0][0]- predicted[0][2][0]))
            m2=((predicted[1][1][0]- predicted[1][3][0])/(predicted[1][0][0]- predicted[1][2][0]))
            #print(m2)
            #dif = l2_avg+200 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
            dif = l2_avg-x
            dire = dif/(((predicted[1][1])+((predicted[0][3])))/2)
            value = round(-1*math.degrees(math.atan(dire)),2)
        
    	    if(abs(value-value_old)>10):
    	        value=value_old
            # else:
            #     value_old=value

            if(round(-1*math.degrees(math.atan(dire)),2)<-10 or round(-1*math.degrees(math.atan(dire)),2)>10):
                arr1=[]

            if(dist==-1):
		       	print(value,value_old)
		       	cv2.putText(frame , str(value), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 2, cv2.LINE_AA)
		       
            #print(predicted[0][0][0])
            #print(math.degrees(math.atan((((predicted[0][0]+predicted[1][0])+((predicted[0][2]+predicted[1][2])))/4 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2)/(((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4))))
            #print(predicted)
        else:
             cv2.circle(frame,(l2_old,m),5,(0,0,255),-1)

        
        '''if predicted is not None and l2 is not None:
            #print(predicted)
            #cv2.circle(frame,(int(predicted[1][0][0]),predicted[1][1][0]),5,(0,255,255),-1)
            dif1 = (predicted[0][3][0]-predicted[0][1][0])/10
            dif2 = (predicted[1][1][0]-predicted[1][3][0])/10
            m=((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4
            m1=((predicted[0][1][0]- predicted[0][3][0])/(predicted[0][0][0]- predicted[0][2][0]))
            m2=((predicted[1][1][0]- predicted[1][3][0])/(predicted[1][0][0]- predicted[1][2][0]))
            #print((int(((0*dif1/m1+predicted[0][0])+(0*dif2/m2+predicted[1][0]))/2)))
            arr=[] 
            for i in range(0,10):
            	arr.append([int(((i*dif1/m1+predicted[0][0][0])+(i*dif2/m2+predicted[1][2][0]))/2),int((predicted[0][1][0]+i*dif1+predicted[1][3][0]+i*dif2)/2)])
            	#arr[i][1]=(int((predicted[0][1][0]+i*dif1+predicted[1][3][0]+i*dif2)/2))
            	#i=i+1                                           
            	#cv2.circle(frame,((int(((i*dif1/m1+predicted[0][0][0])+(i*dif2/m2+predicted[1][2][0]))/2)),(int((predicted[0][1][0]+i*dif1+predicted[1][3][0]+i*dif2)/2))),5,(255,255,0),-1)
            #print(arr)
            for i in range (len(arr)-1):
            	#print(i)
            	cv2.line(frame, (arr[i][0], arr[i][1]), (arr[i+1][0], arr[i+1][1]), (0, 255, 255), 5)'''

        lt.update(lanes)
        out.write(frame)
        cv2.imshow('', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    #args = parser.parse_args()
    main(sys.argv[1],sys.argv[2])
