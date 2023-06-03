
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import matplotlib
import math

#--- Define Tag
id_to_find  = 19
#marker_size  = 10 #- [cm]
marker_size_L  = 18 #- [cm]
marker_size_M  = 10 #- [cm]
marker_size_S  = 5 #- [cm]
id_to_find_list = [81, 19, 273, 26,   305, 319, 290, 298,  390,354,441,333]
correction_vectorT = np.array ([[34,-34 ,-34 ,+34 ,   0,-20 ,0 ,-20,   0,-7.5,0,7.5],
        [-34,-34,34,34,   -20,0 , +20,0,  -7.5,0,+7.5,0],
        [0, 0 , 0 , 0 ,     0 , 0, 0 , 0, 0, 0, 0, 0]]).T
corrected_pos=[]
SHOW_FRAME  = False
#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def mag(x):
    return math.sqrt(sum(kk**2 for kk in x))

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def pose_estimation (frame, ids,id_to_find,corners,marker_size, camera_matrix, camera_distortion,j):
    global  pos_camera , i
    pos_camera = []
    print('marker size', marker_size)
    counter = 0
    for i in range(len(ids)):
        if ids[i] == id_to_find:
            #print('Desired Aruco found',i, "the aruco found is", id_to_find)
            # -- ret = [rvec, tvec, ?]
            # -- array of rotation and position of each marker in camera frame
            # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, camera_distortion)

            # -- Unpack the output, get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # aruco.drawDetectedMarkers(frame, corners,ids, (0,0,255))
            # aruco.drawDetectedMarkers(frame, corners[i],ids[i],(255,255,0))
            corners_desired = corners[counter]
            aruco.drawDetectedMarkers(frame, corners)
            counter += 1

            # print ("ids=",ids[i])
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            # print('DETECTED')
            # print(corners)
            # -- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

            # -- Now get Position andd attitude f the camera respect to the marker
            pos_camera = -R_tc * np.matrix(tvec).T
            #print("camera position is :", pos_camera + correction_vectorT[j].T)
            #correction_vector = np.transpose(correction_vectorT[j])


    return pos_camera, i, marker_size

def update_fps_read():
    global t_read, fps_read
    t           = time.time()
    fps_read    = 1.0/(t - t_read)
    t_read      = t

def update_fps_detect():
    global t_detect, fps_detect
    t           = time.time()
    fps_detect  = 1.0/(t - t_detect)
    t_detect      = t
    #calib_path = "C:/Users/Oussama/Desktop/projet python"
    #camera_matrix = np.loadtxt('C:/Users\Oussama/Desktop/projet python/cameraMatrix.txt', delimiter=',')
    #camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

camera_matrix   = np.loadtxt('C:/Users/Oussama/Desktop/projet python/cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt('C:/Users/Oussama/Desktop/projet python/cameraDistortion.txt', delimiter=',')
#print(camera_matrix)
#print(camera_distortion)

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0


aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()


#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#-- Set the camera size as the one it was calibrated with
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

t_read      = time.time()
t_detect    = t_read
fps_read    = 0.0
fps_detect  = 0.0
magnitude =130
while True:
    print("*******************************************************************")
    print("*******************************************************************")

    #-- Read the camera frame
    ret, frame = cap.read()
    #print ("id to find is =", id_to_find_list[0])

    update_fps_read()
    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #-- remember, OpenCV stores color images in Blue, Green, Red
    for j in range(len(id_to_find_list)):
        pos_camera = []
        #-- Find all the aruco markers in the image
        id_to_find = id_to_find_list[j]
        if j == 0 or j == 1 or j == 2 or j == 3:
            marker_size = marker_size_L
        if j == 4 or j == 5 or j == 6 or j == 7:
            marker_size = marker_size_M
        if j == 8 or j == 9 or j == 10 or j == 11:
            marker_size = marker_size_S
        else:
            marker_size = marker_size_L
        print("**** For the Marker,",j, "referenced as ", id_to_find,"the size is", marker_size)
        #corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, ids= id_to_find, parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        #print (corners)
        res = cv2.aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        if len(res[0]) > 0:# and ids == id_to_find:
            print("Detected Markers number   =", len(ids),"identification is", ids )
            # id_to_find=int(id_to_find_list[0])
            pose_estimation (frame, ids,id_to_find,corners,marker_size, camera_matrix, camera_distortion,j)
            # print('Desired Aruco found', i, "the aruco found is", id_to_find_list[0])
            update_fps_detect()

            #print ("correction",  50 * correction + pos_camera)
            print("Marker pos", pos_camera)
            #print("correction is for", j, "marker position is", pos_camera, "with size", marker_size)
            if j == 0 and len(pos_camera) > 0 and magnitude > 120:
                corrected_pos = pos_camera + [[-34],
                                              [+34],
                                              [0]]
                pos= np.array (corrected_pos)
                magnitude =  np.linalg.norm(pos)
                print("magnitude",magnitude)
            if j == 1 and len(pos_camera) > 0 and magnitude > 120:
                corrected_pos = pos_camera + [[34],
                                              [34],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 2 and len(pos_camera) > 0 and magnitude > 120:
                corrected_pos = pos_camera + [[34],
                                              [-34],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 3 and len(pos_camera) > 0 and magnitude > 120:
                corrected_pos = pos_camera + [[-34],
                                              [-34],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)

            if j == 4 and len(pos_camera) > 0 and magnitude < 120:
                corrected_pos = pos_camera + [[0],
                                              [20],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 5 and len(pos_camera) > 0 and magnitude < 120:
                corrected_pos = pos_camera + [[20],
                                              [0],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 6 and len(pos_camera) > 0 and magnitude < 120:
                corrected_pos = pos_camera + [[0],
                                              [-20],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 7 and len(pos_camera) > 0:
                corrected_pos = pos_camera + [[-20],
                                              [0],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 8 and not pos_camera == [] and magnitude < 80:
                corrected_pos = pos_camera + [[0],
                                              [7.5],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 9 and len(pos_camera) > 0 and magnitude < 80:
                corrected_pos = pos_camera + [[7.5],
                                              [0],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 10 and len(pos_camera) > 0 and magnitude < 80:
                corrected_pos = pos_camera + [[0],
                                              [-7.5],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)
            if j == 11 and len(pos_camera) > 0 and magnitude < 80:
                corrected_pos = pos_camera + [[-7.5],
                                              [0],
                                              [0]]
                pos = np.array(corrected_pos)
                magnitude = np.linalg.norm(pos)
                print("magnitude", magnitude)

            if len (corrected_pos) > 0:
                print ("corrected_pos",corrected_pos)
                print("------------------------------")

        else:
            #print "Nothing detected - fps = %.0f"%fps_read
            print("Nothing detected, try to search more " )
        cv2.imshow('frame', frame)
    if cv2.waitKey(5) & 0xFF == ord('d'):
        break
cap.release()
cv2.destroyAllWindows()


