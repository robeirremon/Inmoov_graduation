from __future__ import division
import numpy as np
import cv2
import math

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
    contours, hierarchy = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours

def findLargest(contours):
    area_max = 0
    for c in contours:
        area = cv2.contourArea(c)
        if (area_max < area and area > 100):
            area_max = area
    return area_max

def draw_ball_contour(binary_image, rgb_image, contours, area_max):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    tolerance = 50
    center = [0,0]
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
        else:
            detection = False
            
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
    yellowUpper = (60, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    area_max = findLargest(contours)
    detection, rgb_image, center = draw_ball_contour(binary_image_mask, rgb_image,contours, area_max)
    return detection, rgb_image, center

class Frame_Angles:

    # ------------------------------
    # User Instructions
    # ------------------------------

    # Set the pixel width and height.
    # Set the angle width (and angle height if it is disproportional).
    # These can be set during init, or afterwards.

    # Run build_frame.

    # Use angles_from_center(self,x,y,top_left=True,degrees=True) to get x,y angles from center.
    # If top_left is True, input x,y pixels are measured from the top left of frame.
    # If top_left is False, input x,y pixels are measured from the center of the frame.
    # If degrees is True, returned angles are in degrees, otherwise radians.
    # The returned x,y angles are always from the frame center, negative is left,down and positive is right,up.

    # Use pixels_from_center(self,x,y,degrees=True) to convert angle x,y to pixel x,y (always from center).
    # This is the reverse of angles_from_center.
    # If degrees is True, input x,y should be in degrees, otherwise radians.

    # Use frame_add_crosshairs(frame) to add crosshairs to a frame.
    # Use frame_add_degrees(frame) to add 10 degree lines to a frame (matches target).
    # Use frame_make_target(openfile=True) to make an SVG image target and open it (matches frame with degrees).

    # ------------------------------
    # User Variables
    # ------------------------------

    pixel_width = 640
    pixel_height = 480

    angle_width = 60
    angle_height = None
    
    # ------------------------------
    # System Variables
    # ------------------------------

    x_origin = None
    y_origin = None

    x_adjacent = None
    x_adjacent = None

    # ------------------------------
    # Init Functions
    # ------------------------------

    def __init__(self,pixel_width=None,pixel_height=None,angle_width=None,angle_height=None):

        # full frame dimensions in pixels
        if type(pixel_width) in (int,float):
            self.pixel_width = int(pixel_width)
        if type(pixel_height) in (int,float):
            self.pixel_height = int(pixel_height)

        # full frame dimensions in degrees
        if type(angle_width) in (int,float):
            self.angle_width = float(angle_width)
        if type(angle_height) in (int,float):
            self.angle_height = float(angle_height)

        # do initial setup
        self.build_frame()

    def build_frame(self):

        # this assumes correct values for pixel_width, pixel_height, and angle_width

        # fix angle height
        if not self.angle_height:
            self.angle_height = self.angle_width*(self.pixel_height/self.pixel_width)

        # center point (also max pixel distance from origin)
        self.x_origin = int(self.pixel_width/2)
        self.y_origin = int(self.pixel_height/2)

        # theoretical distance in pixels from camera to frame
        # this is the adjacent-side length in tangent calculations
        # the pixel x,y inputs is the opposite-side lengths
        self.x_adjacent = self.x_origin / math.tan(math.radians(self.angle_width/2))
        self.y_adjacent = self.y_origin / math.tan(math.radians(self.angle_height/2))

    # ------------------------------
    # Pixels-to-Angles Functions
    # ------------------------------

    def angles(self,x,y):

        return self.angles_from_center(x,y)

    def angles_from_center(self,x,y,top_left=True,degrees=True):

        # x = pixels right from left edge of frame
        # y = pixels down from top edge of frame
        # if not top_left, assume x,y are from frame center
        # if not degrees, return radians

        if top_left:
            x = x - self.x_origin
            y = self.y_origin - y

        xtan = x/self.x_adjacent
        ytan = y/self.y_adjacent

        xrad = math.atan(xtan)
        yrad = math.atan(ytan)

        if not degrees:
            return xrad,yrad

        return math.degrees(xrad),math.degrees(yrad)

    def pixels_from_center(self,x,y,degrees=True):

        # this is the reverse of angles_from_center

        # x = horizontal angle from center
        # y = vertical angle from center
        # if not degrees, angles are radians

        if degrees:
            x = math.radians(x)
            y = math.radians(y)

        return int(self.x_adjacent*math.tan(x)),int(self.y_adjacent*math.tan(y))

    # ------------------------------
    # 3D Functions
    # ------------------------------

    def distance(self,*coordinates):
        return self.distance_from_origin(*coordinates)

    def distance_from_origin(self,*coordinates):
        return math.sqrt(sum([x**2 for x in coordinates]))
    
    def intersection(self,pdistance,langle,rangle,degrees=False):

        # return (X,Y) of target from left-camera-center

        # pdistance is the measure from left-camera-center to right-camera-center (point-to-point, or point distance)
        # langle is the left-camera  angle to object measured from center frame (up/right positive)
        # rangle is the right-camera angle to object measured from center frame (up/right positive)
        # left-camera-center is origin (0,0) for return (X,Y)
        # X is measured along the baseline from left-camera-center to right-camera-center
        # Y is measured from the baseline

        # fix degrees
        if degrees:
            langle = math.radians(langle)
            rangle = math.radians(rangle)

        # fix angle orientation (from center frame)
        # here langle is measured from right baseline
        # here rangle is measured from left  baseline
        langle = math.pi/2 - langle
        rangle = math.pi/2 + rangle

        # all calculations using tangent
        ltan = math.tan(langle)
        rtan = math.tan(rangle)

        # get Y value
        # use the idea that pdistance = ( Y/ltan + Y/rtan )
        Y = pdistance / ( 1/ltan + 1/rtan )

        # get X measure from left-camera-center using Y
        X = Y/ltan

        # done
        return X,Y

    def location(self,pdistance,lcamera,rcamera,center=False,degrees=True):

        # return (X,Y,Z,D) of target from left-camera-center (or baseline midpoint if center-True)

        # pdistance is the measure from left-camera-center to right-camera-center (point-to-point, or point distance)
        # lcamera = left-camera-center (Xangle-to-target,Yangle-to-target)
        # rcamera = right-camera-center (Xangle-to-target,Yangle-to-target)
        # left-camera-center is origin (0,0) for return (X,Y)
        # X is measured along the baseline from left-camera-center to right-camera-center
        # Y is measured from the baseline
        # Z is measured vertically from left-camera-center (should be same as right-camera-center)
        # D is distance from left-camera-center (based on pdistance units)

        # separate values
        lxangle,lyangle = lcamera
        rxangle,ryangle = rcamera

        # yangle should be the same for both cameras (if aligned correctly)
        yangle = (lyangle+ryangle)/2

        # fix degrees
        if degrees:
            lxangle = math.radians(lxangle)
            rxangle = math.radians(rxangle)
            yangle  = math.radians( yangle)

        # get X,Z (remember Y for the intersection is Z frame)
        X,Z = self.intersection(pdistance,lxangle,rxangle,degrees=False)

        # get Y
        # using yangle and 2D distance to target
        Y = math.tan(yangle) * self.distance_from_origin(X,Z)

        # baseline-center instead of left-camera-center
        if center:
            X -= pdistance/2

        # get 3D distance
        D = self.distance_from_origin(X,Y,Z)

        # done
        return X,Y,Z,D

def main():
    # cameras variables
    pixel_width = 640
    pixel_height = 480
    angle_width = 64.4
    angle_height = 36.2
    camera_separation = 6.8
    old_depth = 0
    n = 0
   
    # cameras are the same, so only 1 needed
    angler = Frame_Angles(pixel_width,pixel_height,angle_width,angle_height)
    angler.build_frame()


    klen  = 3 # length of target queues, positive target frames required to reset set X,Y,Z,D
    
    # target queues
    x1k,y1k,x2k,y2k = [],[],[],[]
    x1m,y1m,x2m,y2m = 0,0,0,0
    
    # last positive target
    # from camera baseline midpoint
    X,Y,Z,D = 0,0,0,0
        
    video_capture = cv2.VideoCapture(0)
    video_capture2 = cv2.VideoCapture(1)
    #video_capture = cv2.VideoCapture('video/tennis-ball-video.mp4')

    while(True):
        if  (video_capture.grab() and video_capture2.grab()) :
            flag, frame = video_capture.retrieve()
            flag2, frame2 = video_capture2.retrieve()
            if not (flag and flag2):
                print("Error")
            else:
                detection, detected_frame, new_center = detect_ball_in_a_frame(frame) 
                x1=new_center[0]
                y1=new_center[1]
                detection2, detected_frame2, new_center2 = detect_ball_in_a_frame(frame2)
                x2=new_center2[0]
                y2=new_center2[1]
                
            
        # ret, frame = video_capture.read()
        # detection, detected_frame, new_center = detect_ball_in_a_frame(frame)
        # cv2.imshow('Detection', detected_frame)
        
        # ret2, frame2 = video_capture2.read()
        # detection2, detected_frame2, new_center2 = detect_ball_in_a_frame(frame)
        # cv2.imshow('Detection2', detected_frame2)
        # update queues
        x1k.append(x1)
        y1k.append(y1)
        x2k.append(x2)
        y2k.append(y2)
        # check 3: queues full
        if len(x1k) >= klen:
            # trim
            x1k = x1k[-klen:]
            y1k = y1k[-klen:]
            x2k = x2k[-klen:]
            y2k = y2k[-klen:]

            # mean values
            x1m = sum(x1k)/klen
            y1m = sum(y1k)/klen
            x2m = sum(x2k)/klen
            y2m = sum(y2k)/klen
                                    
            # get angles from camera centers
            xlangle,ylangle = angler.angles_from_center(x1m,y1m,top_left=True,degrees=True)
            xrangle,yrangle = angler.angles_from_center(x2m,y2m,top_left=True,degrees=True)
                        
            # triangulate
            X,Y,Z,new_depth = angler.location(camera_separation,(xlangle,ylangle),(xrangle,yrangle),center=True,degrees=True)
            text = 'X: {:3.1f}\nY: {:3.1f}\nZ: {:3.1f}\nD: {:3.1f}'.format(X,Y,Z,D)
            lineloc = 0
            lineheight = 30
            for t in text.split('\n'):
                lineloc += lineheight
                cv2.putText(detected_frame,
                            t,
                            (10,lineloc), # location
                            cv2.FONT_HERSHEY_PLAIN, # font
                            #cv2.FONT_HERSHEY_SIMPLEX, # font
                            1.5, # size
                            (0,255,0), # color
                            1, # line width
                            cv2.LINE_AA, #
                            False) 
            
            cv2.imshow('Detection', detected_frame)
            cv2.imshow('Detection2', detected_frame2)
            

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()