from __future__ import division

import cv2

import track
import detect
import detect2
import argparse
import sys
import time
import math



'''
    02,03     10,11
    00,01     12,13
'''
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
    check=False
    dif_new=0
    i=0
    while cap.isOpened():
    	#time.sleep(0.5)
        precTick = ticks
        ticks = cv2.getTickCount()
        dt = (ticks - precTick) / cv2.getTickFrequency()
        #print(dt)
        ret, frame = cap.read()
        frame = frame[100:600,300:1100]
        cv2.imshow('ss',frame[100:600,200:500])
        predicted = lt.predict(dt)
        #time.sleep(0.02)
        f=frame[0:250]
        #cv2.imshow('ss',f)
        #cv2.imshow('a', frame[100:400])

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
            if(abs(l2-l2_old)<100):
                l2=l2_old
            else:
                l2_old=l2


            cv2.line(frame, (predicted[0][0], predicted[0][1]), (predicted[0][2], predicted[0][3]), (0, 0, 255), 5)
           # cv2.line(frame, (predicted[1][0], predicted[1][1]), (predicted[1][2], predicted[1][3]), (0, 0, 255), 5)
            #cv2.line(frame, (((predicted[0][0]+predicted[1][0]))/2,((predicted[0][1]+predicted[1][1]))/2),(((predicted[0][2]+predicted[1][2]))/2,((predicted[0][3]+predicted[1][3]))/2), (0, 255, 0), 5)
            #x=(max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
            x=(predicted[0][2]+predicted[0][0])/2
            y=(((predicted[0][3])+predicted[0][1])/2)
            '''if(abs(x-x_old)<50):
            	x=x_old
            else:
            	x_old=x'''


            cv2.circle(frame, (x,y),5,(255,0,0),-1)
            cv2.rectangle(frame,(x+15,y+15), (x-15, y-15), (255,0,0),1)
            #cv2.circle(frame, (((predicted[0][0]+predicted[1][2])+((predicted[0][2]+predicted[1][2])))/4,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4), 5,(0, 255, 0),-1)
            #cv2.circle(frame,(350,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4),5,(0,255,0),-1)
            cv2.circle(frame,(dif_new,y),5,(0,255,0),-1)
            m=((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4
            m1=((predicted[0][1][0]- predicted[0][3][0])/(predicted[0][0][0]- predicted[0][2][0]))
            m2=((predicted[1][1][0]- predicted[1][3][0])/(predicted[1][0][0]- predicted[1][2][0]))
            #print(m2)
            dif = l2+200 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
            dire = dif/(((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4)
            if(i%100==0):
                dif_new = x
                check=True

            dif_actual = x - dif_new
            dire_actual = dif_actual/((predicted[0][3])+((predicted[0][3]))/2)

            print(round(-1*math.degrees(math.atan(dire_actual)),2))
            #print(dif_new)
            #print(predicted[0][0][0])
            #print(math.degrees(math.atan((((predicted[0][0]+predicted[1][0])+((predicted[0][2]+predicted[1][2])))/4 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2)/(((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4))))
            #print(predicted)
        else:
             cv2.circle(frame,(l2_old+200,m),5,(0,0,255),-1)


        lt.update(lanes)
        out.write(frame)
        cv2.imshow('', frame)
        i=i+1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    #args = parser.parse_args()
    main(sys.argv[1],sys.argv[2])