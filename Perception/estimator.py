#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

x = [ 15.0  , 45.0  ]
y = [ 5.0   , 35.0  ]
z = [ 105.0 , 135.0 ]

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
    rospy.init_node('estimate', anonymous = True)
    
    rospy.Subscriber('detection', Pose, callback)
    
    rospy.spin()


def callback(data):
    pub = rospy.Publisher('estimation', Pose, queue_size = 10)
    
    data.position.x += 4.03
    data.position.y += 0.3
    data.position.z += 138.53
    data.orientation.x += 4.03
    data.orientation.y += 0.3
    data.orientation.z += 138.53

    velocity = estimateVelocity((data.position.x, data.position.y, data.position.z), (data.orientation.x, data.orientation.y, data.orientation.z))
    positions = getTrajectory((data.orientation.x, data.orientation.y, data.orientation.z), velocity, (0, gTimesteps, 0), timeStepSize, eulerSteps)
    data_to_send = Point()  # the data to be sent, initialise the array
    
    for i in positions:
        if ((i[0] > x[0]) and (i[0] < x[1]) and (i[1] > y[0]) and (i[1] < y[1]) and (i[2] > z[0]) and (i[2] < z[1])):
            data_to_send = positions[i] 
            pub.publish(data_to_send)
            print(data_to_send)

if __name__ == '__main__':
    estimate()
    
    
    
    
    
    
    
    
    
    
    
    
    
    