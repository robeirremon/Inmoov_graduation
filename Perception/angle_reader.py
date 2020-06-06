#!/usr/bin/env python

import rospy
from numpy import interp
from std_msgs.msg import Float32




def Omoplate_mapping( value):
	# name of publisher, and topic and type of msg 
	pubomoplate = rospy.publisher('lOmoplateAng',Float32,queue_size=10)
	
	#loop
	while not rospy.is_shutdown():
		angle_omoplate = value
		pubomoplate.publish(angle_omoplate)
        	rate.sleep()

def Shoulder_mapping( value):
	pubshoulder = rospy.publisher('lShoulderAng',Float32,queue_size=10)

	#loop
	while not rospy.is_shutdown():
		angle_shoulder = numpy.interp(value,[-25,155],[0,180])
		pubshoulder.publish(angle_shoulder)
        	rate.sleep()



def Rotate_mapping( value):
	pubrotate = rospy.publisher('lRotateAng',Float32,queue_size=10)

	#loop
	while not rospy.is_shutdown():
		angle_rotate = numpy.interp(value,[-50,90],[40,180])
		pubrotate.publish(angle_rotate)
        	rate.sleep()


def Bicebs_mapping( value):
	pubbicebs = rospy.publisher('lBicebsAng',Float32,queue_size=10)

	#loop
	while not rospy.is_shutdown():
		angle_bicebs = value
		pubbicebs.publish(angle_bicebs)
        	rate.sleep()


def Wrist_mapping( value):
	pubwrist = rospy.publisher('WristAng',Float32,queue_size=10)
	
	#loop
	while not rospy.is_shutdown():
		angle_wrist = numpy.interp(value,[-25,155],[0,180])
		pubwrist.publish(angle_wrist)
        	rate.sleep()


def angle_reader():
	#intialization
	rospy.init_node('angle_reader', anonymous = True )

	#subscribers
	rospy.Subscriber("leftBicebsAngle", Float32, Bicebs_mapping)
	rospy.Subscriber("leftWristAngle", Float32, Wrist_mapping)
	rospy.Subscriber("leftOmoplateAngle", Float32, Omoplate_mapping)
	rospy.Subscriber("leftRotateAngle", Float32, Rotate_mapping)
	rospy.Subscriber("leftShoulderAngle", Float32, Shoulder_mapping)
	
	# spin() simply keeps python from exiting until this node is stopped
  	rospy.spin()


if __name__ == '__main__':
    
	angle_reader()
