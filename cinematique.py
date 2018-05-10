import itertools
import numpy as np
import time
import math

import pypot.dynamixel


def degToRad(deg):
    return math.pi * deg / 180

def radToDeg(rad):
    return 180 * rad / math.pi

#lower_limit = -180
#upper_limit = 180

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the second available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

found_ids = dxl_io.scan()
print('Found ids:', found_ids)

ids = found_ids[:5]

dxl_io.enable_torque(ids)

#dxl_io.set_angle_limit(dict(zip(ids, itertools.repeat((lower_limit, upper_limit)))))

print('get angle limit', dxl_io.get_angle_limit(ids))



print('get present position', dxl_io.get_present_position(ids))

xJ = 3 
yJ = 0
zJ = 0

l1 = 1
l2 = 1
l3 = 1
l4 = 1
l5 = 1

alpha = degToRad(135)

if yJ == 0:
    theta1 = math.pi / 2
    
else:
    tantheta1 = xJ / yJ

    if tantheta1 >= 1:
        tantheta1 = 1
    elif tantheta1 <= -1:
        tantheta1 = -1
        
    theta1 = np.arctan2(xJ, yJ)

sintheta2 = (yJ - np.cos(alpha - math.pi)*(l3 + l4 + l5)) / l2

if sintheta2 >= 1:
    sintheta2 = 1
elif sintheta2 <= -1:
    sintheta2 = -1

theta2 = np.arcsin(sintheta2)

if theta2 < 0:
    theta2 = -theta2

theta3 = alpha - theta2

print("theta1 = {}".format(theta1))
print("theta2 = {}".format(theta2))
print("theta3 = {}".format(theta3))

print("theta1 = {}".format(radToDeg(theta1)))
print("theta2 = {}".format(radToDeg(theta2)))
print("theta3 = {}".format(radToDeg(theta3)))

theta1 = -theta1
theta2 = -theta2
theta3 = -theta3 + math.pi /2

dxl_io.set_goal_position({ids[0]: radToDeg(theta1)})

dxl_io.set_goal_position({ids[1]: radToDeg(theta2)})

dxl_io.set_goal_position({ids[2]: radToDeg(theta3)})

print('get present position', dxl_io.get_present_position(ids))
