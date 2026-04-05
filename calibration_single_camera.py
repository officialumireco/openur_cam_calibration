'''
/*******************************************************************
 * Project Name: Stereo camera 
 * File Name: objectfinder_multiprocess_main.py
 * Description: Open source library using OPENCV for marine snow detection
 * Author: Umi-Reco 
 * License: MIT (see LICENSE file for details)
 ******************************************************************/
''' 

import numpy as np
import cv2
import os
import json


'''

Sample of configuration JSON

dict_cfg = {
    "processing":{
        "id":"dataset",
        "sync":{
            "sync":true,
            "tsec_offset_lcam":0,
            "tsec_offset_rcam":7.073733333333333,
            "start_frame_lcam":0,
            "start_frame_rcam":212
        },
        "calib_board":{
            "width":100,
            "height":60,
            "dia_circle":24,        
            "dia_circle_centre":30,        
            "num_rows":13,
            "num_cols":19
        },
        "tsec_start":30,
        "tsec_duration":12,
        "force_extract":false,
        "force_clean":false
    },
    "paths":{
        "path_video_lcam":"left_camera.mov",
        "path_video_rcam":"right_camera.mov",
        "dir_database":"/Stereo"
    }           
}

'''


def _make_blob_detector():

    # Setup SimpleBlobDetector parameters.
    blobParams = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    blobParams.minThreshold = 150
    blobParams.maxThreshold = 200

    # Filter by Area.
    blobParams.filterByArea = True
    blobParams.minArea = 100     # minArea may be adjusted to suit for your experiment
    blobParams.maxArea = 4000   # maxArea may be adjusted to suit for your experiment

    # Filter by Circularity
    blobParams.filterByCircularity = True
    blobParams.minCircularity = 0.1

    # Filter by Convexity
    blobParams.filterByConvexity = True
    blobParams.minConvexity = 0.1

    # Filter by Inertia
    blobParams.filterByInertia = False
    blobParams.minInertiaRatio = 0.01

    blobParams.filterByColor = True
    blobParams.blobColor = 0

    # Create a detector with the parameters
    blobDetector = cv2.SimpleBlobDetector_create(blobParams)

    return(blobDetector)

def vpr_get_calibrate(dict_cfg):

    board = dict_cfg['processing']['calib_board']
    chessboardSize = (board['num_cols'],board['num_rows'])

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

    objp = objp * 20

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    img_points_lcam = [] # 2d points in image plane.
    img_points_rcam = [] # 2d points in image plane.

    dir_images          = dict_cfg['paths']['dir_images']
    dir_images_detected = dict_cfg['paths']['dir_detected']
    dir_params          = dict_cfg['paths']['dir_detected']
    
    os.makedirs(dict_cfg['paths']['dir_detected'],exist_ok=True)

    l_images = os.listdir(dir_images)
    l_images = [s for s in l_images if '.DS' not in s]

    # Making a custom circle detector
    myblobDetector = _make_blob_detector() 

    for c_img in l_images:

        img             = cv2.imread(os.path.join(dir_images,c_img))
        print('INFO: Reading image > ' + c_img)
        img_gs          = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_gs          = cv2.bitwise_not(img_gs)
        ret_val, img_gs = cv2.threshold(img_gs, 240, 255, cv2.THRESH_BINARY)

        path_out        = os.path.join(dir_images_detected,'detected_' + c_img)
        path_blob       = os.path.join(dir_images_detected,'blob_' + c_img)
        keypoints       = myblobDetector.detect(img_gs)

        for x in range(0,len(keypoints)):
            imgarr_rgb=cv2.circle(img_gs, (int(keypoints[x].pt[0]),int(keypoints[x].pt[1])), radius=int(3 + (keypoints[x].size)/2), color=(0,0,0), thickness=-1)

        cv2.imwrite(path_blob, imgarr_rgb)

        ret, corners = cv2.findCirclesGrid(img_gs, (board['num_cols'],board['num_rows']),None,flags=cv2.CALIB_CB_SYMMETRIC_GRID,blobDetector=myblobDetector)
        
        if ret is True:

            objpoints.append(objp)
            corners_lcam = cv2.cornerSubPix(img_gs, corners, (11,11), (-1,-1), criteria)
            img_points_lcam.append(corners_lcam)
            img_out_lcam  = cv2.drawChessboardCorners(img, (board['num_cols'],board['num_rows']), corners, ret)
            cv2.imwrite(path_out, img_out_lcam)
        
        else:
            print('Not detected')
            cv2.imwrite(path_out, img_gs)

    #ret, mtx, dist, rvecs, tvecs
    retL, cameraMatrixL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, img_points_lcam, img_gs.shape[::-1], None, None)
    heightL, widthL, channelsL = img.shape
    newCameraMatrixL, roi_L = cv2.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

    # Reprojection Error
    mean_error = 0

    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecsL[i], tvecsL[i], newCameraMatrixL, distL)
        error = cv2.norm(img_points_lcam[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print("INFO: Total error: {}".format(mean_error/len(objpoints)))
  
    # Wriring to file
    path_xml = os.path.join(dir_params,'stereoMap.xml')
    cv_file = cv2.FileStorage(path_xml, cv2.FILE_STORAGE_WRITE)
    print('INFO: Saving Stereo Parameters > ' + path_xml)

    cv_file.write('K',cameraMatrixL)
    cv_file.write('D',distL)
    #cv_file.write('q', Q)

    cv_file.release()
    
    return dict_cfg

#--------- MAIN ------------------
#Edit the config file and run in python

dict_cfg = {
    "processing":{
        "id":"BOSS-A",
        "calib_board":{
            "width":100,
            "height":100,
            "dia_circle":24,        
            "dia_circle_centre":30,        
            "num_rows":7, #number of rows in the board
            "num_cols":7  #number of columns in the board
        },
        "clean":False
    },
    "paths":{
        "dir_images":"/images", #images takes from the camera for calibration
        "dir_detected":"/process" #images saved by program after blog detection and circle pattern detection
    }           
}

vpr_get_calibrate(dict_cfg)