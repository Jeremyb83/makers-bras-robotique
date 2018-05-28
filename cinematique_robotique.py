import itertools
import numpy as np
import time
import math

import cv2
import cv2.aruco as aruco
import yaml
import scanner

import pypot.dynamixel


def degToRad(deg):
    return math.pi * deg / 180

def radToDeg(rad):
    return 180 * rad / math.pi

def searchObject(): 
    #startT = time.time()
    #currentT = time.time()
    v = 60
    dxl_io.set_moving_speed({ids[0]: v, ids[1]: v, ids[2]: v, ids[3]: v, ids[4]: v})

    dxl_io.set_goal_position({ids[1]: 20})
    for k in range(-20, -60, -10):
        dxl_io.set_goal_position({ids[2]: k})
        for i in range(-150, 120, 40):
            if((k/10)%2 == 1):
                dxl_io.set_goal_position({ids[0]: i})
            else:
                dxl_io.set_goal_position({ids[0]: -i-(150-120)}) 
            time.sleep(0.9)
        
        
    
def moveTo(xJ, yJ, zJ):
    l1 = 44
    l2 = 53
    l3 = 47
    l4 = 43
    l5 = 141
    
    if yJ == 0:
        theta1 = math.pi / 2
        
    else:
        tantheta1 = xJ / yJ

        if tantheta1 >= 1:
            tantheta1 = 1
        elif tantheta1 <= -1:
            tantheta1 = -1
            
        theta1 = np.arctan2(xJ, yJ)

    xC = 0
    yC = 0
    zC = l1
    CJ = np.sqrt(xJ**2 + yJ**2 + (zJ - zC)**2)

    cosAlpha = (-(l3+l4+l5)**2 + l2**2 + CJ**2)/float(2*l2*CJ)
    if cosAlpha >= 1:
        cosAlpha = 1
    elif cosAlpha <= -1:
        cosAlpha = -1
    
    alpha = np.arccos(cosAlpha)

    sinBeta = (zJ - zC)/float(CJ)
    if sinBeta >= 1:
        sinBeta = 1
    elif sinBeta <= -1:
        sinBeta = -1
        
    beta = np.arcsin(sinBeta)

    print("alpha = {}".format(radToDeg(alpha)))
    print("beta = {}".format(radToDeg(beta)))

    theta2 = math.pi - alpha - beta

    cosTheta3 = (-CJ**2 + (l3+l4+l5)**2 + l2**2)/float(2*(l3+l4+l5)*l2)
    if cosTheta3 >= 1:
        cosTheta3 = 1
    elif cosTheta3 <= -1:
        cosTheta3 = -1
                                                  
    theta3 = np.arccos(cosTheta3)


    print("theta1 = {}".format(theta1))
    print("theta2 = {}".format(theta2))
    print("theta3 = {}".format(theta3))

    print("theta1 = {}".format(radToDeg(theta1)))
    print("theta2 = {}".format(radToDeg(theta2)))
    print("theta3 = {}".format(radToDeg(theta3)))

    theta1 = -theta1
    theta2 = -theta2 + math.pi /2
    theta3 = theta3 - math.pi /2

    print("theta1 = {}".format(radToDeg(theta1)))
    print("theta2 = {}".format(radToDeg(theta2)))
    print("theta3 = {}".format(radToDeg(theta3)))

    dxl_io.set_goal_position({ids[0]: radToDeg(theta1)})

    dxl_io.set_goal_position({ids[1]: radToDeg(theta2)})

    dxl_io.set_goal_position({ids[2]: radToDeg(theta3)})

    print('get present position', dxl_io.get_present_position(ids))


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

v = 50
dxl_io.set_moving_speed({ids[0]: v, ids[1]: v, ids[2]: v, ids[3]: v, ids[4]: v})

print('get present position', dxl_io.get_present_position(ids))

xJ = 284
yJ = 0
zJ = 44

cap = cv2.VideoCapture(0)

skip_lines = 6
data = None
with open('microsoftWebcamMatrix.yml') as infile:
    for i in range(skip_lines):
        _ = infile.readline()
    data = yaml.load(infile)
    mtx, dist = [data[i] for i in ('Camera_Matrix','Distortion_Coefficients')]
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
while(True):
    ret, frame = cap.read()
    if(not ret):
        print("ERROR : Can't read camera")
    coord = scanner.findMarker(frame, aruco_dict, parameters, mtx, dist, 0.048, True)
    if coord is not None:
        moveTo(coord[0]*10**3, coord[1]*10**3, coord[2]*10**3)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


#searchObject()
