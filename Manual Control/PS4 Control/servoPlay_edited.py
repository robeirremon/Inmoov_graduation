# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

from adafruit_servokit import ServoKit
import board
import busio
import time
from approxeng.input.selectbinder import ControllerResource


def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]


# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...

print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))

print("Initializing ServoKit")
kit = ServoKit(channels=16, i2c=i2c_bus0)

# kit[0] is the bottom servo
# kit[1] is the top servo
print("Done initializing")

# sweep = range(0,180)
# for degree in sweep :
#     kit.servo[0].angle=degree
#     # kit.servo[1].angle=degree
#     # time.sleep(0.01)

# time.sleep(0.5)
# sweep = range(180,0, -1)
# for degree in sweep :
#     kit.servo[0].angle=degree

while True:
    try:
        with ControllerResource() as joystick:
            print('Found a joystick and connected')
            print(type(joystick).__name__)
            while joystick.connected:
                lx, ly, rx, ry = joystick['lx','ly','rx','ry']
                
                if (lx >=-1 and lx<=0):               
                    lx_angle = scale(lx, (-1,0), (50,120))  # omoplate rest 120 (50-120)
                    kit.servo[0].angle=lx_angle
                
                if (ly >=-1 and ly<=0):               
                    ly_angle = scale(ly, (-1,0), (0,30))  # shoulder rest 30  (0-30)
                    kit.servo[1].angle=ly_angle
                
                if (ly >=0 and ly<=1):               
                    ly_angle = scale(ly, (0,1), (30,180))  # shoulder rest 30  (30-180)
                    kit.servo[1].angle=ly_angle
                
                if (rx >=-1 and rx<=0):               
                    rx_angle = scale(rx, (-1,0), (40,90))  # rotate rest 90 (40-90)
                    kit.servo[2].angle=rx_angle
                
                if (rx >=0 and rx<1):               
                    rx_angle = scale(rx, (0,1), (90,180))  # rotate rest 90 (40-90)
                    kit.servo[2].angle=rx_angle
                
                if (ry >=0 and ry<=1):               
                    ry_angle = scale(ry, (0,1), (0,50))  # biceps rest 0   (0-50)
                    kit.servo[3].angle=ry_angle
                                

                # lx_angle = (lx+1)/2*180         #
                # ly_angle = (ly+1)/2*180         #
                # rx_angle = (rx+1)/2*180         #
                # ry_angle = (ry+1)/2*180         #
                
                    
                
                # axis_list = [ 'lx', 'ry' ]
                # for axis_name in axis_list:
                #     # desired_angle is in degrees
                #     joystick_value = joystick[axis_name]
                #     # The joystick value goes from -1.0 ... 1.0 (a range of 2)
                #     # Normalize within a range of 180 degrees
                #     desired_angle = (joystick_value+1)/2*180
                    
                #     if  axis_name == 'lx' :
                #         kit.servo[0].angle=desired_angle
                #         # print(axis_name, joystick[axis_name])
                        
                #     if axis_name == 'ry' :
                #          kit.servo[1].angle=joystick[axis_name]
            print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        time.sleep(1.0)
        
                