import cv2
from cv2 import aruco
import numpy as np
import math
import pandas as pd

# create the input file name
file_name = '370'
# input image
img = cv2.imread(file_name + '.jpg')

# the size of ArUco Markers
marker_size = 3.6  # cm

# Marker IDs
top_right_id = 1 # 12
top_left_id = 0 # 11
bottom_left_id = 2 # 10
bottom_right_id = 3 # 13
# end_effector_id = 608
object_id = 495
palm_id = 608
# center_id = 0
finger_1_prox_id = 331
finger_1_dist_id = 189
finger_2_prox_id = 190
finger_2_dist_id = 411

# Load the dictionary of the ArUco Markers
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
# a = aruco.D
parameters = aruco.DetectorParameters_create()

# --- Get the camera calibration path
calib_path = ""
# we need to get the camera and distortino matrix every time
camera_matrix = np.load(calib_path + 'camera_mtx.npy')
camera_distortion = np.load(calib_path + 'dist_mtx.npy')

# Detect the markers.
corners, ids, rejected = aruco.detectMarkers(image=img, dictionary=aruco_dict, parameters=parameters,
                                             cameraMatrix=camera_matrix, distCoeff=camera_distortion)


# calculate the orientation of ArUco Markers
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


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


# Read the location and orientation of the ArUco Markers
def read_inf(ids, i, data, rvecs, tvecs):
    # make a dictionary of the ArUco Markers
    id = [608, 2, 0, 1, 3, 495, 331, 189, 190, 411]
    location = ['palm', 'bottom_left', 'top_left', 'top_right', 'bottom_right', 'object',
                'finger_1_prox', 'finger_1_dist', 'finger_2_prox', 'finger_2_dist']
    dict = {}
    for n in range(len(id)):
        dict[id[n]] = location[n]

    name = dict[ids[i, 0]]

    # --- 180 deg rotation matrix around the x axis
    R_flip = np.zeros((3, 3), dtype=np.float32)
    R_flip[0, 0] = 1.0
    R_flip[1, 1] = -1.0
    R_flip[2, 2] = -1.0

    # transformation matrix (from camera frame to world frame)
    # change unit from cm to m 
    # Re-calculate value every time 
    tranf = np.eye(4)
    tranf[1, 1] = -1
    tranf[2, 2] = -1
    # the error in x, y, z axis
    tranf[0, 3] = 0
    tranf[1, 3] = -0.002
    tranf[2, 3] = 0.8795

    np.savetxt('tranf_world_to_camera.csv', tranf, delimiter=',')

    # make the location matrix of ArUco Marker
    pos = np.insert(tvecs[i], [3], 100, axis=None) / 100

    # get the world frame location (Unit: Meter)
    trans = np.dot(tranf, pos)
    # get the output of the location
    str_position = name + " Position x=%4.3f  y=%4.3f  z=%4.3f" % (
        trans[0], trans[1], trans[2])

    # -- Obtain the rotation matrix tag->camera
    R_ct = np.matrix(cv2.Rodrigues(rvecs[i])[0])
    R_tc = R_ct.T

    # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

    str_attitude = name + " Euler Angle x=%4.1f  y=%4.1f  z=%4.1f" % (
        math.degrees(roll_marker), math.degrees(pitch_marker),
        math.degrees(yaw_marker))

    # draw axis
    aruco.drawAxis(img, camera_matrix, camera_distortion, rvecs[i], tvecs[i], 1.5)

    data_save = [name, trans[0], trans[1], trans[2], math.degrees(roll_marker), math.degrees(pitch_marker),
                 math.degrees(yaw_marker)]

    data.append(data_save)

    return str_position, str_attitude, data


# setting the position of text box
font = cv2.FONT_HERSHEY_PLAIN
# get the rotation and translation vectors
rvecs, tvecs, _objPonits = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

# create a empty list to save position and orientation
data = []

# identify each ArUco Marker
for i in range(ids.size):

    # Do the output for each ArUco Marker
    """if ids[i] == end_effector_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 25), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.putText(img, str_attitude, (0, 50), font, 1, (0, 0, 255), 1, cv2.LINE_AA)"""

    if ids[i] == palm_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 75), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.putText(img, str_attitude, (0, 100), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

    """if ids[i] == center_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 125), font, 1, (0, 255, 0), 1, cv2.LINE_AA)"""

    if ids[i] == top_right_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == top_left_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 175), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == bottom_left_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 200), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == bottom_right_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 225), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == finger_1_prox_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 250), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == finger_1_dist_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 275), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == finger_2_prox_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 300), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
    if ids[i] == finger_2_dist_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 325), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

    if ids[i] == object_id:
        [str_position, str_attitude, data] = read_inf(ids, i, data, rvecs, tvecs)

        cv2.putText(img, str_position, (0, 350), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

# save csv file
column_list = ['location', 'x-axis', 'y-axis', 'z-axis', 'x-degree', 'y-degree', 'z-degree']
data_file = np.array(data)
data_file = pd.DataFrame(data=data_file, columns=column_list)
save_csv_file_name = 'result/ data_file_' + file_name + '.csv'
data_file.to_csv(save_csv_file_name, index=False)

# produce the result image
out = aruco.drawDetectedMarkers(img, corners)
save_img_file_name = 'result/ result_' + file_name + '.png'
cv2.imwrite(save_img_file_name, out)
cv2.imshow("out", out)
cv2.waitKey(0)
cv2.destroyAllWindows()
