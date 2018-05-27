import numpy as np
import cv2
import cv2.aruco as aruco
import yaml
import scanner

cap = cv2.VideoCapture(1)

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
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
