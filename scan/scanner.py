# coding=utf-8
import numpy as np
import cv2
import cv2.aruco as aruco
import yaml

def findMarker(frame, aruco_dict, parameters, mtx, dist, markerLength, debug = False):
    '''
    Détecte et localise le marker dans une image
    Paramètres :
        frame : OpenCV Image - image à analyser
        aruco_dict : Aruco Dictionary - Dictionnaire des markers à rechercher
        parameters : Aruco Detector Parameters - Obtenu grace à aruco.DetectorParameters_create()
        mtx : Matrix 3x3 - Matrice de la caméra
        dist : Array 5x1 - Coefficient de distorsion de la caméra
        markerLength : float - Coté (en m) d'un marker
        debug : bool - afficher l'image analyser

    Retourne :
        - Si un market est détecté dans <frame>, les coordonnées (x, y, z) du marker
        - None, sinon

    '''
    img = frame
    # img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    if len(corners) > 0:
        cam_mat = np.array([mtx['data'][0:3],mtx['data'][3:6],mtx['data'][6:9]], dtype="float32")
        dist_coef = np.array(dist['data'], dtype="float32")
        rvec, tvec, obj_pts = aruco.estimatePoseSingleMarkers(corners, markerLength, cam_mat, dist_coef)
        for i in range(len(ids)):
            rmat = cv2.Rodrigues(rvec)

            rmat = np.matrix(rmat[0])
            obj_pts =  np.matrix(obj_pts[0])
            tvec = np.matrix(tvec[0])
            coord_3d = rmat * obj_pts.transpose() + tvec.transpose()
            if debug:
                aruco.drawAxis(img, cam_mat, dist_coef, rvec[i], tvec[i], 0.1)
                print "Coord (x, y, z) :"
                print (coord_3d.item(0), coord_3d.item(1), coord_3d.item(2))
                cv2.imshow('frame / press q to quit',img)
            # Il n'y a qu'un element dans le tableau
            return (coord_3d.item(0), coord_3d.item(1), coord_3d.item(2))
