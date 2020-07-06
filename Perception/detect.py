#!/usr/bin/env python

from __future__ import division
import numpy as np
import cv2
import math
import time
import freenect 
import rospy
from geometry_msgs.msg import Pose

def detect():
    pub = rospy.Publisher('detection', Pose, queue_size=10)
    rospy.init_node('detect', anonymous=True)
    old_real_values = [0.0,0.0,0.0]
    start_time = 0
    end_time = 0
    while not rospy.is_shutdown():
        frame = get_video()
        detection = False
        detection, detected_frame, new_center, end_time = detect_ball_in_a_frame(frame)
        elapsed_time = end_time - start_time
        cv2.imshow('Detection', detected_frame)
        x=new_center[0]
        y=new_center[1]
        real_values = get_depth(y,x)
        all_values = old_real_values ,real_values

        if  (real_values[0] > 0) and (detection == True) and (real_values[0] < 250) and (old_real_values[0] - real_values[0] < 60):
            data_to_send = Pose()  # the data to be sent, initialise the array
            data_to_send.position.x = all_values[0][0]
            data_to_send.position.y = all_values[0][1]
            data_to_send.position.z = all_values[0][2]
            data_to_send.orientation.x = all_values[1][0]
            data_to_send.orientation.y = all_values[1][1]
            data_to_send.orientation.z = all_values[1][2]
            data_to_send.orientation.w = elapsed_time
            pub.publish(data_to_send)
            print(data_to_send)

            old_real_values = real_values
            start_time = end_time
        else:
            print("no contours detected")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.waitKey(0)
    cv2.destroyAllWindows()


#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array

def distance3D(old_center, new_center):
    distance = math.sqrt((new_center[0] - old_center[0])**2 + (new_center[1] - old_center[1])**2 + (new_center[2] - old_center[2])**2)
    return distance

#function to get depth image from kinect
def get_depth(y,x):
    array,_ = freenect.sync_get_depth()
    #array = array.astype(np.uint8)

    depth_value = array[y,x]
    #distance_cm = 100/(-0.00307 * depth_value + 3.33)
    distance_meter = 0.1236 * math.tan((depth_value / 2842.5) + 1.1863)         
    w = 640
    h = 480
    #minDistance = -10
    #scaleFactor = .0021
    z = distance_meter * 100
    #x = (x - w / 2) * (z + minDistance) * scaleFactor
    #y = (y - h / 2) * (z + minDistance) * scaleFactor
    x = (2 * math.tan(29 * 3.14159265359 / 180) * z) * ((x - w/2) / 640)
    y = (2 * math.tan(22.5 * 3.14 / 180) * z) * ((y - h/2) / 480)
    real_values = [z,-x,-y]
    return real_values
     

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask

def getContours(binary_image):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    _, contours, hierarchy = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sec = time.time()
    
    return contours, sec

def findLargest(contours):
    area_max = 0
    for c in contours:
        area = cv2.contourArea(c)
        if (area_max < area and area > 50):
            area_max = area
    return area_max

def draw_ball_contour(binary_image, rgb_image, contours, area_max):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    tolerance = 50
    center = [0,0]
    detection = False
    for c in contours:
        cx, cy = get_contour_center(c)
        area = cv2.contourArea(c)
        min_val = area_max - tolerance
        max_val = area_max + tolerance
        if (area < max_val and area > min_val and area > 100):
            #perimeter= cv2.arcLength(c, True)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            
            cv2.circle(rgb_image, (cx, cy),(int)(radius),(0,0,255),3)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            cv2.circle(rgb_image, (cx,cy),5,(150,150,255),-1)
            
            center = [cx, cy]
            detection = True
    return detection, rgb_image, center
        
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
    #print ("number of contours: {}".format(len(contours)))
    #cv2.imshow("RGB Image Contours",rgb_image)
    #cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):
    yellowLower =(25, 50, 50)
    #yellowLower =(50, 50, 50)
    yellowUpper = (60, 255, 255)
    #yellowUpper = (100, 255, 100)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours, sec = getContours(binary_image_mask)
    area_max = findLargest(contours)
    detection, rgb_image, center = draw_ball_contour(binary_image_mask, rgb_image,contours, area_max)
    return detection, rgb_image, center, sec

if __name__ == '__main__':
    try:
        detect()
        
    except rospy.ROSInterruptException:
        pass
