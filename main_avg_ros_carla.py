from __future__ import division

import cv2
import track
import detect
import detect2
import argparse
import sys
import time
import math
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from carla_msgs.msg import CarlaEgoVehicleControl
#from statistics import mean 


parser = argparse.ArgumentParser()
parser.add_argument("--path", type=str, help="Video path")

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color",Image,self.callback)
        #self.image_pub = rospy.Publisher("angle",Twist,queue_size=1)

    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        except CvBridgeError as e:
            print(e)
        frame = mainp(frame)
        cv2.imshow('', frame)
        cv2.waitKey(3)


def mainp(frame):

    global ticks
    global lt
    global ld
    global l2_old
    global arr1
    global arr2
    global sum1
    global elapsedTime
    global diff
    global prev_err
    global kp
    global ki
    global kd
    #cap = cv2.VideoCapture(video_path)
    #ret,frame = cap.read()   

    
    y=0
    #time.sleep(0.025)
    precTick = ticks
    ticks = cv2.getTickCount()
    dt = (ticks - precTick) / cv2.getTickFrequency()
    #print(dt)
    #ret, frame = cap.read()
    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl)
    msg = CarlaEgoVehicleControl()
    msg.throttle = 0.22
    frame = frame[200:600]
    predicted = lt.predict(dt)
    #time.sleep(0.02)
    f=frame
    #cv2.imshow("s",f)
    #cv2.imshow('aa',frame)
    #cv2.imshow('ss',f)
    lanes = ld.detect(f)
    #l2= ld2.detect(frame[100:600,200:500])
    l2=200

    if predicted is not None and l2 is not None:

        cv2.line(frame, (predicted[0][0], predicted[0][1]), (predicted[0][2], predicted[0][3]), (0, 0, 255), 5)
        cv2.line(frame, (predicted[1][0], predicted[1][1]), (predicted[1][2], predicted[1][3]), (0, 0, 255), 5)
        #cv2.line(frame, (((predicted[0][0]+predicted[1][0]))/2,((predicted[0][1]+predicted[1][1]))/2),(((predicted[0][2]+predicted[1][2]))/2,((predicted[0][3]+predicted[1][3]))/2), (0, 255, 0), 5)
        x1=(max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
        y=((predicted[1][1])+((predicted[0][3])))/2
	#cv2.circle(frame, (int(x1),int(y)),5,(255,255,0),-1)
        
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
      #  print(int(x))
        cv2.circle(frame, (int(x),int(((predicted[1][1])+((predicted[0][3])))/2)),5,(255,0,0),-1)
        cv2.rectangle(frame,(int(x)+10,int(y)+10), (int(x)-10, int(y)-10), (255,0,0),1)
        cnt = np.float32([(int(x)+10,int(y)+10),(int(x)+10,int(y)-10),(int(x)-10,int(y)-10),(int(x)-10,int(y)+10)])
        #cv2.polylines(frame, cnt, True, 255, 3, cv2.LINE_AA) 
        #print(cnt)
        dist = cv2.pointPolygonTest(cnt,(int(l2_avg),int((predicted[1][1])+((predicted[0][3])))/2),False)
        print(dist)
        #cv2.circle(frame, (((predicted[0][0]+predicted[1][2])+((predicted[0][2]+predicted[1][2])))/4,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4), 5,(0, 255, 0),-1)
        #cv2.circle(frame,(350,((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4),5,(0,255,0),-1)
        cv2.circle(frame,(int(l2_avg),int(((predicted[1][1])+((predicted[0][3])))/2)),5,(0,255,0),-1)
        #m=((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4
        #m1=((predicted[0][1][0]- predicted[0][3][0])/(predicted[0][0][0]- predicted[0][2][0]))
        #m2=((predicted[1][1][0]- predicted[1][3][0])/(predicted[1][0][0]- predicted[1][2][0]))
        #print(m2)
        #dif = l2_avg+200 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2
        dif = l2_avg-x
        dire = dif/(((predicted[1][1])+((predicted[0][3])))/2)
        if(round(-1*math.degrees(math.atan(dire)),2)<-10 or round(-1*math.degrees(math.atan(dire)),2)>10):
            arr1=[]


        #if(dist==-1):
        deg = pid(round(-1*math.degrees(math.atan(dire)),2)/30)
        #sum1=sum1+
        #print(round(-1*math.degrees(math.atan(dire)),2))
        #msg.steer = round(-1*math.degrees(math.atan(dire)),2)/30
        print(deg)
        msg.steer = round(deg,2)
        pub.publish(msg)
        cv2.putText(frame , str(round(deg,2)), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 2, cv2.LINE_AA)
        ''''else:
            print(0)
            msg.steer = 0
            pub.publish(msg)
            cv2.putText(frame , str(0), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 2, cv2.LINE_AA)'''

        #print(predicted[0][0][0])
        #print(math.degrees(math.atan((((predicted[0][0]+predicted[1][0])+((predicted[0][2]+predicted[1][2])))/4 - (max(predicted[0][0],predicted[0][2])+min(predicted[1][0],predicted[1][2]))/2)/(((predicted[0][1]+predicted[1][1])+((predicted[0][3]+predicted[1][3])))/4))))
        #print(predicted)
    else:
         cv2.circle(frame,(l2_old,y),5,(0,0,255),-1)

    lt.update(lanes)
    #cv2.imshow('', frame)
    return frame

def pid(dif):
	deltaT = (millis() - elapsed_time)
	elapsed_time=millis()
	sum1 = sum1 + dif * deltaT;
	diff = (dif - prev_err) / deltaT;
	prev_err = dif
	output = (kp * dif + ki * sum1 + kd * diff);
	return output


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    #args = parser.parse_args()
    ticks = 0
    rospy.init_node('image_converter', anonymous=True)
    lt = track.LaneTracker(2, 0.1, 500)
    ld = detect.LaneDetector(180)
    ld2 = detect2.LaneDetector(180)
    l2_old=0
    x_old=0
    arr1=[]
    arr2=[]
    sum1=0
    diff=0
    elapsed_time=0
    prev_err = 0
    kp=1
    ki=1
    kd=1
    main(sys.argv)