#!/usr/bin/env python

from __future__ import division
import numpy as np
import cv2
import math
import time
import math
import rospy
from geometry_msgs.msg import Pose
import pyrealsense2 as rs

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 90)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

# Start streaming
profile = pipeline.start(config)

def detect():
    pub = rospy.Publisher('detection', Pose, queue_size=10)
    rospy.init_node('detect', anonymous=True)
    old_real_values = [0.0,0.0,0.0]
    start_time = 0
    end_time = 0
    while not rospy.is_shutdown():
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        detection = False
        detection, detected_frame, new_center, end_time = detect_ball_in_a_frame(frame)
        elapsed_time = end_time - start_time
        cv2.imshow('Detection', detected_frame)
        x=new_center[0]
        y=new_center[1]
        cv2.circle(frame, (x, y), 2, (255,0,0), thickness=2)

        depth = frames.get_depth_frame()
        
        real_values = get_depth(y,x,depth, frame, frames)
        all_values = old_real_values ,real_values

        if  (real_values[0] > 0) and (detection == True) and (real_values[0] < 250):
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

def distance3D(old_center, new_center):
    distance = math.sqrt((new_center[0] - old_center[0])**2 + (new_center[1] - old_center[1])**2 + (new_center[2] - old_center[2])**2)
    return distance

#function to get depth image from kinect
def get_depth(y,x,depth,frame,frames):
    colorizer = rs.colorizer()
    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frames = align.process(frames)

    # Update color and depth frames:
    aligned_depth_frame = frames.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    height, width = frame.shape[:2]
    expected = int(300)
    aspect = int(width / height)
    resized_image = cv2.resize(frame, (int(round(expected * aspect)), expected))
    crop_start = int(round(expected * (aspect - 1) / 2))
    crop_img = resized_image[0:expected, crop_start:crop_start+expected]
    scale = height / expected
    
    depth = np.asanyarray(aligned_depth_frame.get_data())

    cv2.circle(colorized_depth, (x, y), 2, (255,255,255), thickness=2)
    cv2.imshow("depth", colorized_depth)

    z = aligned_depth_frame.get_distance(x,y)
    w = 640
    h = 480
    x = (2 * math.tan(29 * 3.14159265359 / 180) * z) * ((x - w/2) / 640)
    y = (2 * math.tan(22.5 * 3.14 / 180) * z) * ((y - h/2) / 480)
    real_values = [z*100,-x*100,-y*100]
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
        if (area_max < area):
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
            (x,y,w,h)=cv2.boundingRect(c)
            # ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            
            cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(255,0,0),2)
            # cv2.circle(rgb_image, (cx, cy),(int)(radius),(0,0,255),3)
            # cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            # cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            # cv2.circle(rgb_image, (cx,cy),5,(150,150,255),-1)
            # cx = int((x + w) / 2)
            # cy = int((y + h) / 2)
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
    yellowLower =(29, 30, 107)
    #yellowLower =(50, 50, 50)
    yellowUpper = (89, 221, 255)
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
