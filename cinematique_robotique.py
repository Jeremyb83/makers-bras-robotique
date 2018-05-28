# -*- coding: utf-8 -*-
import itertools
import numpy as np
import time
import math

import cv2
import cv2.aruco as aruco
import yaml
import scanner

import pypot.dynamixel
from picamera import PiCamera
from picamera.array import PiRGBArray



def degToRad(deg):
    return math.pi * deg / 180

def radToDeg(rad):
    return 180 * rad / math.pi

def searchObject(i,sens): 
    #startT = time.time()
    #currentT = time.time()
    dxl_io.set_goal_position({ids[1]: 20})
    dxl_io.set_goal_position({ids[2]: -60})
    if sens ==1:
        dxl_io.set_goal_position({ids[0]: i})
    else:
        dxl_io.set_goal_position({ids[0]: -i-(150-120)})
    #time.sleep(0.2)
        
        
    
def moveTo(xcam, ycam, zcam):
    l1 = 44
    l2 = 53
    l3 = 47
    l4 = 43
    l5 = 141
    lcam = 290
    thetaCam = degToRad(45)
    print("ycam = {}".format((ycam)))
    print("zcam = {}".format((zcam)))
    print("thetaCam = {}".format((thetaCam)))
    print("np.cos(thetaCam) = {}".format((np.cos(thetaCam))))
    print("np.sin(thetaCam) = {}".format((np.sin(thetaCam))))
    #changement repère camera:
    xJ = xcam
    yJ = np.cos(thetaCam)*ycam + np.sin(thetaCam)*zcam
    zJ = np.cos(thetaCam)*zcam - np.sin(thetaCam)*ycam - lcam

    print("yJ = {}".format(radToDeg(yJ)))
    print("zJ = {}".format(radToDeg(zJ)))

    
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
    #seuil = 10
    print('get present position', dxl_io.get_present_position(ids))
    #if not ((dxl_io.get_present_position({ids[0]})  > (radToDeg(theta1)-seuil)) and (dxl_io.get_present_position({ids[0]})  < (radToDeg(theta1)+seuil))):
    #dxl_io.set_goal_position({ids[0]: radToDeg(theta1)})

    seuil = 20
    if(xcam < 0 - seuil or xcam > 0 + seuil):
        dxl_io.set_goal_position({ids[0]: radToDeg(theta1)})
        print("PASSSSSSSSSSSSSSSSSSSSSSSSSOKKKKKKKKKKK")
    else:
        print("okkkkkkkkkkkkkkkkkkkkkkkkk")
        dxl_io.set_goal_position({ids[1]: radToDeg(theta2)})

        dxl_io.set_goal_position({ids[2]: radToDeg(theta3)})
        
def catch():
    dxl_io.set_goal_position({ids[4]: 150})

    time.sleep(2)

    dxl_io.set_goal_position({ids[2]: 0})
    dxl_io.set_goal_position({ids[1]: 0})

    time.sleep(2)
    dxl_io.set_goal_position({ids[4]: 100})

        

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

v = 70
dxl_io.set_moving_speed({ids[0]: v, ids[1]: v, ids[2]: v, ids[3]: v, ids[4]: v})

print('get present position', dxl_io.get_present_position(ids))

xJ = 284
yJ = 0
zJ = 44


resolution = (1280, 720)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=resolution)
 
# allow the camera to warmup
time.sleep(0.1)

skip_lines = 6
data = None
with open('piCamMatrix.yml') as infile:
    for i in range(skip_lines):
        _ = infile.readline()
    data = yaml.load(infile)
    mtx, dist = [data[i] for i in ('Camera_Matrix','Distortion_Coefficients')]
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    

debut = time.time()
i = -150
sens = 1
lost = 30
stable = 0
# capture frames from the camera
dxl_io.set_goal_position({ids[4]: 100})
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    coord = scanner.findMarker(image, aruco_dict, parameters, mtx, dist, 0.048, True)
    duree = time.time() - debut
    if coord is not None and duree > 0.5 :
        debut = time.time()
        moveTo(coord[0]*10**3, coord[1]*10**3, coord[2]*10**3)
        lost = 0
        stable += 1
    else:
        lost+=1
        stable = 0
        if lost > 30:
            print("searchObject")
            if coord is None and i < 120:
                searchObject(i, sens)
                i+=5
            else:
                i = -150
                sens = (sens+1)%2
    if stable == 5:
        print("catchhhh")
        catch()
                
    rawCapture.truncate(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
 
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)


# When everything done, release the capture
#cap.release()
cv2.destroyAllWindows()


#searchObject()
