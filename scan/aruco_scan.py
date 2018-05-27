import numpy as np
import cv2
import cv2.aruco as aruco
import yaml

cap = cv2.VideoCapture(1)
while(True):
    skip_lines = 6
    data = None
    with open('microsoftWebcamMatrix.yml') as infile:
        for i in range(skip_lines):
            _ = infile.readline()
        data = yaml.load(infile)
    mtx, dist = [data[i] for i in ('Camera_Matrix','Distortion_Coefficients')]
    ret, frame = cap.read()

    img = frame
    # img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    if len(corners) > 0:
        cam_mat = np.array([mtx['data'][0:3],mtx['data'][3:6],mtx['data'][6:9]], dtype="float32")
        dist_coef = np.array(dist['data'], dtype="float32")
        rvec, tvec, obj_pts = aruco.estimatePoseSingleMarkers(corners, 0.048, cam_mat, dist_coef)
        for i in range(len(ids)):
            aruco.drawAxis(img, cam_mat, dist_coef, rvec[i], tvec[i], 0.1)
        rmat = cv2.Rodrigues(rvec)

        rmat = np.matrix(rmat[0])
        obj_pts =  np.matrix(obj_pts[0])
        tvec = np.matrix(tvec[0])
        coord_3d = rmat * obj_pts.transpose() + tvec.transpose()
        print "Coord :"
        print (coord_3d.item(0), coord_3d.item(1), coord_3d.item(2))
        print "coord_3d"
        print type(coord_3d)
    cv2.imshow('frame / press q to quit',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
