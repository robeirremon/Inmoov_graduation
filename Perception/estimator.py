#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray

global positions = [[0,0,0],[0,0,0]]

pixelsPerMeter = 980.0
FPS = 30.0

# Euler's method will proceed by timeStepSize / timeStepPrecision at a time
timeStepSize = 0.5 / FPS
timeStepPrecision = 1.0

# Number of Euler's method steps to take
eulerSteps = 15

# Gravitational acceleration is in units of pixels per second squared
gSeconds = 9.81 * pixelsPerMeter
# Per-timestep gravitational acceleration (pixels per timestep squared)
gTimesteps = gSeconds * (timeStepSize**2)

def estimateVelocity(old_pos, new_pos):
    velocity = [(new_pos[0] - old_pos[0]), (new_pos[1] - old_pos[1]), (old_pos[2] - old_pos[2])]
    return velocity

def eulerExtrapolate(position, velocity, acceleration, timeDelta):
    position[0] += velocity[0] * timeDelta
    position[1] += velocity[1] * timeDelta
    position[2] += velocity[2] * timeDelta

    velocity[0] += acceleration[0] * timeDelta
    velocity[1] += acceleration[1] * timeDelta
    velocity[2] += acceleration[2] * timeDelta

    return (position, velocity)

def getTrajectory(initialPosition, initialVelocity, acceleration, timeDelta, numTrajPoints):
    positions = []
    
    position = list(initialPosition)
    velocity = list(initialVelocity)
    for i in range(numTrajPoints):
        position, velocity = eulerExtrapolate(position, velocity, acceleration, 1)
        positions.append(position[:])
    return positions

def estimate():
    pub = rospy.Publisher('estimation', UInt16MultiArray, queue_size = 10)
    
    rospy.init_node('estimate', anonymous = True)
    
    rospy.Subscriber('detection', UInt16MultiArray, callback)
    
    data_to_send = UInt16MultiArray()  # the data to be sent, initialise the array
    data_to_send.data = positions # assign the array with the value you want to send
    
    pub.publish(data_to_send)
    
    rospy.spin()


def callback(data):
    velocity = estimateVelocity((data.data[0], data.data[1], data.data[2]), (data.data[3], data.data[4], data.data[5]))
    positions = getTrajectory((data.data[3], data.data[4], data.data[5]), velocity, (0, gTimesteps, 0), timeStepSize, eulerSteps)
    

if __name__ == '__main__':
    estimate()
    
    
    
    
    
    
    
    
    
    
    
    
    
    