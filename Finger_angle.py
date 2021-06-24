import math

import numpy as np
import pandas as pd
import sympy as sy


def read_file(path):
    """
    read the data file and return the name, position, degree
    """
    data = pd.read_csv(path)
    name = np.array(data[['location']])
    position = np.array(data[['x-axis', 'y-axis', 'z-axis']])
    angle = np.array(data[['x-degree', 'y-degree', 'z-degree']])
    return name, position, angle


def read_aruco_angle(name, angle):
    """
    According to the angle from aruco marker to get the finger angle
    """
    data_name = name
    data_ang = angle
    # get the finger 1 proximal angle
    finger_1_prox_index = np.where(data_name == 'finger_1_prox')[0][0]
    finger_1_prox = data_ang[finger_1_prox_index, -1]
    # get the finger 1 distal angle
    finger_1_dist_index = np.where(data_name == 'finger_1_dist')[0][0]
    finger_1_dist = data_ang[finger_1_dist_index, -1]
    # get the finger 2 proximal angle
    finger_2_prox_index = np.where(data_name == 'finger_2_prox')[0][0]
    finger_2_prox = data_ang[finger_2_prox_index, -1]
    # get the finger 2 distal angle
    finger_2_dist_index = np.where(data_name == 'finger_2_dist')[0][0]
    finger_2_dist = data_ang[finger_2_dist_index, -1]
    # get the palm angle
    palm_index = np.where(data_name == 'palm')[0][0]
    palm = data_ang[palm_index, -1]

    # get the angle of finger 1 proximal and distal
    diff_finger_1_p_palm = finger_1_prox - palm
    diff_finger_1_d_palm = finger_1_dist - palm
    finger_1_prox_angle = 90 + diff_finger_1_p_palm
    finger_1_dist_angle = 180 + (diff_finger_1_d_palm - diff_finger_1_p_palm)

    # get the angle of finger 2 proximal and distal
    diff_finger_2_p_palm = (finger_2_prox - 180) - palm
    diff_finger_2_d_palm = (finger_2_dist - 180) - palm
    finger_2_prox_angle = 90 - diff_finger_2_p_palm
    finger_2_dist_angle = 180 - (diff_finger_2_d_palm - diff_finger_2_p_palm)

    return finger_1_prox_angle, finger_1_dist_angle, finger_2_prox_angle, finger_2_dist_angle


def read_aruco_position(name, position, angle):
    # initial para
    finger_1_prox_width = (0.037 - (0.036 / 2)) / 2
    finger_1_dist_width = (0.032 - (0.036 / 2)) / 2
    finger_2_prox_width = (0.037 - (0.036 / 2)) / 2
    finger_2_dist_width = (0.035 - (0.036 / 2)) / 2

    distance_move_x_axis_1 = finger_1_prox_width + 0.023
    distance_move_x_axis_2 = finger_2_prox_width + 0.031
    distance_move_y_axis_1 = 0.045
    distance_move_y_axis_2 = 0.045

    # get the palm angle
    palm_index = np.where(name == 'palm')[0][0]
    palm = abs(angle[palm_index, -1]) * np.pi / 180

    # rotate matrix
    if angle[palm_index, -1] > 0:
        rotate_mat = np.array([[np.cos(-palm), -np.sin(-palm), 0], [np.sin(-palm), np.cos(-palm), 0], [0, 0, 1]])
    else:
        rotate_mat = np.array([[np.cos(palm), -np.sin(palm), 0], [np.sin(palm), np.cos(palm), 0], [0, 0, 1]])

    # read the position of each joint
    finger_1_prox_index = np.where(name == 'finger_1_prox')[0][0]
    finger_1_prox_pos = np.array([position[finger_1_prox_index, :]])
    finger_1_prox_pos = np.reshape(finger_1_prox_pos, (3, 1))
    finger_1_dist_index = np.where(name == 'finger_1_dist')[0][0]
    finger_1_dist_pos = position[finger_1_dist_index, :]
    finger_1_dist_pos = np.reshape(finger_1_dist_pos, (3, 1))
    finger_2_prox_index = np.where(name == 'finger_2_prox')[0][0]
    finger_2_prox_pos = position[finger_2_prox_index, :]
    finger_2_prox_pos = np.reshape(finger_2_prox_pos, (3, 1))
    finger_2_dist_index = np.where(name == 'finger_2_dist')[0][0]
    finger_2_dist_pos = position[finger_2_dist_index, :]
    finger_2_dist_pos = np.reshape(finger_2_dist_pos, (3, 1))
    palm_pos = position[palm_index, :]
    palm_pos = np.reshape(palm_pos, (3, 1))

    # After translate and rotate to the original point
    finger_1_prox_pos_new = rotate_mat.dot(finger_1_prox_pos - palm_pos)
    finger_1_dist_pos_new = rotate_mat.dot(finger_1_dist_pos - palm_pos)
    finger_2_prox_pos_new = rotate_mat.dot(finger_2_prox_pos - palm_pos)
    finger_2_dist_pos_new = rotate_mat.dot(finger_2_dist_pos - palm_pos)

    palm_pos_1_p_new = np.array([[-distance_move_x_axis_1], [distance_move_y_axis_1], [0]])
    palm_pos_2_p_new = np.array([[distance_move_x_axis_2], [distance_move_y_axis_2], [0]])

    x1p, y1p = sy.symbols('x1p y1p')
    dis_aruco_mid_finger1_prox = finger_1_prox_width + 0.036 / 2
    dis_mid_finger1_prox_plam = 0.019
    eq = [(x1p - finger_1_prox_pos_new[0, 0]) ** 2 + (
            y1p - finger_1_prox_pos_new[1, 0]) ** 2 - dis_aruco_mid_finger1_prox ** 2,
          (x1p - palm_pos_1_p_new[0, 0]) ** 2 + (y1p - palm_pos_1_p_new[1, 0]) ** 2 - dis_mid_finger1_prox_plam ** 2]
    result = sy.nonlinsolve(eq, [x1p, y1p])
    for position in result:
        # a = type(position[0]) == sy.Float
        if isinstance(position[0], sy.Float) and isinstance(position[1], sy.Float):
            if position[1] > 0.045:
                if position[0] >= finger_1_prox_pos_new[0, 0]:
                    finger_1_prox_pos_new = np.array([[position[0]], [position[1]], [0]])

    x2p, y2p = sy.symbols('x2p y2p')
    dis_aruco_mid_finger2_prox = finger_2_prox_width + 0.036 / 2
    dis_mid_finger2_prox_plam = 0.015
    eq = [(x2p - finger_2_prox_pos_new[0, 0]) ** 2 + (
            y2p - finger_2_prox_pos_new[1, 0]) ** 2 - dis_aruco_mid_finger2_prox ** 2,
          (x2p - palm_pos_2_p_new[0, 0]) ** 2 + (y2p - palm_pos_2_p_new[1, 0]) ** 2 - dis_mid_finger2_prox_plam ** 2]
    result = sy.nonlinsolve(eq, [x2p, y2p])
    for position in result:
        if isinstance(position[0], sy.Float) and isinstance(position[1], sy.Float):
            if position[1] > 0.045:
                if position[0] <= finger_2_prox_pos_new[0, 0]:
                    finger_2_prox_pos_new = np.array([[position[0]], [position[1]], [0]])

    diff_finger_1_p_palm = finger_1_prox_pos_new - palm_pos_1_p_new
    diff_finger_2_p_palm = finger_2_prox_pos_new - palm_pos_2_p_new

    if diff_finger_1_p_palm[0, 0] < 0:
        finger_1_prox_angle = 180 - math.degrees(
            math.atan(abs(diff_finger_1_p_palm[1, 0]) / abs(diff_finger_1_p_palm[0, 0])))
    else:
        finger_1_prox_angle = math.degrees(math.atan(abs(diff_finger_1_p_palm[1, 0]) / abs(diff_finger_1_p_palm[0, 0])))
    if diff_finger_2_p_palm[0, 0] < 0:
        finger_2_prox_angle = math.degrees(math.atan(abs(diff_finger_2_p_palm[1, 0]) / abs(diff_finger_2_p_palm[0, 0])))
    else:
        finger_2_prox_angle = 180 - math.degrees(
            math.atan(abs(diff_finger_2_p_palm[1, 0]) / abs(diff_finger_2_p_palm[0, 0])))

    finger_1_d_shift = np.array(
        [[0.045 * np.cos(finger_1_prox_angle * np.pi / 180)], [0.045 * np.sin(finger_1_prox_angle * np.pi / 180)], [0]])
    finger_2_d_shift = np.array(
        [[0.045 * np.cos(finger_2_prox_angle * np.pi / 180)], [0.045 * np.sin(finger_2_prox_angle * np.pi / 180)], [0]])

    palm_pos_1_d_new = palm_pos_1_p_new + finger_1_d_shift
    palm_pos_2_d_new = palm_pos_2_p_new + finger_2_d_shift

    x1d, y1d = sy.symbols('x1d y1d')
    dis_aruco_mid_finger1_dist = finger_1_dist_width + 0.036 / 2
    dis_mid_finger1_dist_plam = 0.02
    eq = [(x1d - finger_1_dist_pos_new[0, 0]) ** 2 + (
            y1d - finger_1_dist_pos_new[1, 0]) ** 2 - dis_aruco_mid_finger1_dist ** 2,
          (x1d - palm_pos_1_d_new[0, 0]) ** 2 + (y1d - palm_pos_1_d_new[1, 0]) ** 2 - dis_mid_finger1_dist_plam ** 2]
    result = sy.nonlinsolve(eq, [x1d, y1d])
    for position in result:
        if isinstance(position[0], sy.Float) and isinstance(position[1], sy.Float):
            if position[1] > 0.045:
                if position[0] >= finger_1_dist_pos_new[0, 0]:
                    finger_1_dist_pos_new = np.array([[position[0]], [position[1]], [0]])

    x2d, y2d = sy.symbols('x2d y2d')
    dis_aruco_mid_finger2_dist = finger_2_dist_width + 0.036 / 2
    dis_mid_finger2_dist_plam = 0.025
    eq = [(x2d - finger_2_dist_pos_new[0, 0]) ** 2 + (
            y2d - finger_2_dist_pos_new[1, 0]) ** 2 - dis_aruco_mid_finger2_dist ** 2,
          (x2d - palm_pos_2_d_new[0, 0]) ** 2 + (y2d - palm_pos_2_d_new[1, 0]) ** 2 - dis_mid_finger2_dist_plam ** 2]
    result = sy.nonlinsolve(eq, [x2d, y2d])
    for position in result:
        if isinstance(position[0], sy.Float) and isinstance(position[1], sy.Float):
            if position[1] > 0.045:
                if position[0] <= finger_2_dist_pos_new[0, 0]:
                    finger_2_dist_pos_new = np.array([[position[0]], [position[1]], [0]])

    diff_finger_1_d_palm = finger_1_dist_pos_new - palm_pos_1_d_new
    diff_finger_2_d_palm = finger_2_dist_pos_new - palm_pos_2_d_new

    if diff_finger_1_d_palm[0, 0] < 0:
        finger_1_dist_angle = 180 - math.degrees(
            math.atan(abs(diff_finger_1_d_palm[1, 0]) / abs(diff_finger_1_d_palm[0, 0]))) + (180 - finger_1_prox_angle)
    else:
        finger_1_dist_angle = math.degrees(
            math.atan(abs(diff_finger_1_d_palm[1, 0]) / abs(diff_finger_1_d_palm[0, 0]))) + (180 - finger_1_prox_angle)
    if diff_finger_2_d_palm[0, 0] < 0:
        finger_2_dist_angle = math.degrees(
            math.atan(abs(diff_finger_2_d_palm[1, 0]) / abs(diff_finger_2_d_palm[0, 0]))) + (180 - finger_2_prox_angle)
    else:
        finger_2_dist_angle = 180 - math.degrees(
            math.atan(abs(diff_finger_2_d_palm[1, 0]) / abs(diff_finger_2_d_palm[0, 0]))) + (180 - finger_2_prox_angle)

    return finger_1_prox_angle, finger_1_dist_angle, finger_2_prox_angle, finger_2_dist_angle


def get_finger_pos(name, position, angle):
    data_name = name
    data_ang = angle
    # get the finger 1 proximal angle
    finger_1_prox_index = np.where(data_name == 'finger_1_prox')[0][0]
    finger_1_prox = data_ang[finger_1_prox_index]
    # get the finger 1 distal angle
    finger_1_dist_index = np.where(data_name == 'finger_1_dist')[0][0]
    finger_1_dist = data_ang[finger_1_dist_index]
    # get the finger 2 proximal angle
    finger_2_prox_index = np.where(data_name == 'finger_2_prox')[0][0]
    finger_2_prox = data_ang[finger_2_prox_index]
    # get the finger 2 distal angle
    finger_2_dist_index = np.where(data_name == 'finger_2_dist')[0][0]
    finger_2_dist = data_ang[finger_2_dist_index]

    # read the position of each joint
    finger_1_prox_index = np.where(name == 'finger_1_prox')[0][0]
    finger_1_prox_pos = np.array([position[finger_1_prox_index, :]])
    finger_1_prox_pos = np.reshape(finger_1_prox_pos, (3, 1))
    finger_1_dist_index = np.where(name == 'finger_1_dist')[0][0]
    finger_1_dist_pos = position[finger_1_dist_index, :]
    finger_1_dist_pos = np.reshape(finger_1_dist_pos, (3, 1))
    finger_2_prox_index = np.where(name == 'finger_2_prox')[0][0]
    finger_2_prox_pos = position[finger_2_prox_index, :]
    finger_2_prox_pos = np.reshape(finger_2_prox_pos, (3, 1))
    finger_2_dist_index = np.where(name == 'finger_2_dist')[0][0]
    finger_2_dist_pos = position[finger_2_dist_index, :]
    finger_2_dist_pos = np.reshape(finger_2_dist_pos, (3, 1))

    # initial Transform matrix
    original_pos = np.array([0, 0, 0, 1]).T
    original_pos = np.reshape(original_pos, (4, 1))
    
    # from finger_dist_1 to the marker
    fd1_mat = np.array([[1, 0, 0, -0.031896],
                        [0, 1, 0, -0.041946],
                        [0, 0, 1, 0.0063680],
                        [0, 0, 0, 1]])

    # from finger_dist_2 to the marker
    fd2_mat = np.array([[1, 0, 0, 0.0311410],
                        [0, 1, 0, -0.040089],
                        [0, 0, 1, -0.014486],
                        [0, 0, 0, 1]])

    # from finger_prox_1 to the marker
    fp1_mat = np.array([[1, 0, 0, -0.003713],
                        [0, 1, 0, -0.043195],
                        [1, 0, 0, 0.0094430],
                        [0, 0, 0, 1]])
    
    # from finger_prox_2 to the marker
    fp2_mat = np.array([[1, 0, 0, 0.0038120],
                        [0, 1, 0, -0.043246],
                        [0, 0, 1, -0.009320],
                        [0, 0, 0, 1]])

    # get the Transform matrix from finger to world

    finger_1_dist_real_pos = Transform_matrix(finger_1_dist, finger_1_dist_pos) @ fd1_mat @ original_pos
    finger_1_prox_real_pos = Transform_matrix(finger_1_prox, finger_1_prox_pos) @ fp1_mat @ original_pos
    finger_2_dist_real_pos = Transform_matrix(finger_2_dist, finger_2_dist_pos) @ fd2_mat @ original_pos
    finger_2_prox_real_pos = Transform_matrix(finger_2_prox, finger_2_prox_pos) @ fp2_mat @ original_pos

    return finger_1_dist_real_pos, finger_1_prox_real_pos, finger_2_dist_real_pos, finger_2_prox_real_pos, finger_1_dist, finger_1_prox, finger_2_dist, finger_2_prox


def Transform_matrix(angle, position):
    ax = angle[0]
    ay = angle[1]
    az = angle[2]
    x = position[0]
    y = position[1]
    z = position[2]
    Rx_mat = np.array([[1, 0, 0],
                       [0, np.cos(np.deg2rad(ax)), -np.sin(np.deg2rad(ax))],
                       [0, np.sin(np.deg2rad(ax)), np.cos(np.deg2rad(ax))]])

    Ry_mat = np.array([[np.cos(np.deg2rad(ay)), 0, np.sin(np.deg2rad(ay))],
                       [0, 1, 0],
                       [-np.sin(np.deg2rad(ay)), 0, np.cos(np.deg2rad(ay))]])

    Rz_mat = np.array([[np.cos(np.deg2rad(az)), -np.sin(np.deg2rad(az)), 0],
                       [np.sin(np.deg2rad(az)), np.cos(np.deg2rad(az)), 0],
                       [0, 0, 1]])

    R_mat = Rz_mat # @ Ry_mat @ Rx_mat

    Transform_mat = np.zeros(shape=(4, 4))
    Transform_mat[0][0] = R_mat[0][0]
    Transform_mat[0][1] = R_mat[0][1]
    Transform_mat[0][2] = R_mat[0][2]
    Transform_mat[0][3] = x

    Transform_mat[1][0] = R_mat[1][0]
    Transform_mat[1][1] = R_mat[1][1]
    Transform_mat[1][2] = R_mat[1][2]
    Transform_mat[1][3] = y

    Transform_mat[2][0] = R_mat[2][0]
    Transform_mat[2][1] = R_mat[2][1]
    Transform_mat[2][2] = R_mat[2][2]
    Transform_mat[2][3] = z

    Transform_mat[3][3] = 1

    return Transform_mat


if __name__ == '__main__':
    # get the date
    data_name, data_pos, data_ang = read_file('data_file_frame0000.csv')
    # calculate the finger real pos
    f1dp, f1pp, f2dp, f2pp, f1da, f1pa, f2da, f2pa = get_finger_pos(data_name, data_pos, data_ang)
    data = np.array([['finger_1_dist', f1dp[0][0], f1dp[1][0], f1dp[2][0], f1da[0], f1da[1], f1da[2]],
                     ['finger_1_prox', f1pp[0][0], f1pp[1][0], f1pp[2][0], f1pa[0], f1pa[1], f1pa[2]],
                     ['finger_2_dist', f2dp[0][0], f2dp[1][0], f2dp[2][0], f2da[0], f2da[1], f2da[2]],
                     ['finger_2_prox', f2pp[0][0], f2pp[1][0], f2pp[2][0], f2pa[0], f2pa[1], f2pa[2]]])
    column_list = ['location', 'x-axis', 'y-axis', 'z-axis', 'x-degree', 'y-degree', 'z-degree']
    data_file = pd.DataFrame(data=data, columns=column_list)
    save_csv_file_name = 'data_file0000_finger_pos.csv'
    data_file.to_csv(save_csv_file_name, index=False)

    # calculate the distance between object and each finger
    # get the position of object
    object_index = np.where(data_name == 'object')[0][0]
    object_pos = np.array([data_pos[object_index, :]])
    object_pos = np.reshape(object_pos, (3, 1))

    distance_f1d = np.sqrt(np.power((f1dp[0] - object_pos[0]), 2) + np.power((f1dp[1] - object_pos[1]), 2)) - 0.025
    distance_f1p = np.sqrt(np.power((f1pp[0] - object_pos[0]), 2) + np.power((f1pp[1] - object_pos[1]), 2)) - 0.025
    distance_f2d = np.sqrt(np.power((f2dp[0] - object_pos[0]), 2) + np.power((f2dp[1] - object_pos[1]), 2)) - 0.025
    distance_f2p = np.sqrt(np.power((f2pp[0] - object_pos[0]), 2) + np.power((f2pp[1] - object_pos[1]), 2)) - 0.025
    distance_data = np.array([['finger_1_dist_distance', distance_f1d[0]],
                              ['finger_1_prox_distance', distance_f1p[0]],
                              ['finger_2_dist_distance', distance_f2d[0]],
                              ['finger_2_prox_distance', distance_f2p[0]]])
    name_list = ['location', 'distance']
    distance_file = pd.DataFrame(data=distance_data, columns=name_list)
    save_distance_csv_file_name = 'data_file0000_distance.csv'
    distance_file.to_csv(save_distance_csv_file_name, index=False)

    # method one
    finger_1a_prox_angle, finger_1a_dist_angle, finger_2a_prox_angle, finger_2a_dist_angle = read_aruco_angle(data_name,
                                                                                                              data_ang)
    angle_data = np.array([['finger_1_dist_angle', finger_1a_dist_angle],
                              ['finger_1_prox_angle', finger_1a_prox_angle],
                              ['finger_2_dist_angle', finger_2a_dist_angle],
                              ['finger_2_prox_angle', finger_2a_prox_angle]])
    angle_name_list = ['location', 'angle']
    angle_file = pd.DataFrame(data=angle_data, columns=angle_name_list)
    save_angle_csv_file_name = 'data_file0000_angle.csv'
    angle_file.to_csv(save_angle_csv_file_name, index=False)

"""    # method two
    finger_1p_prox_angle, finger_1p_dist_angle, finger_2p_prox_angle, finger_2p_dist_angle = read_aruco_position(
        data_name, data_pos, data_ang)

    print('Finger_1_prox: {:.4}'.format(finger_1a_prox_angle))
    print('Finger_1_prox: {:.4}'.format(finger_1p_prox_angle))
    print('Finger_1_prox: {:.4}\n'.format((finger_1p_prox_angle + finger_1a_prox_angle) / 2))

    print('Finger_1_dist: {:.4}'.format(finger_1a_dist_angle))
    print('Finger_1_dist: {:.4}'.format(finger_1p_dist_angle))
    print('Finger_1_dist: {:.4}\n'.format((finger_1p_dist_angle + finger_1a_dist_angle) / 2))

    print('Finger_2_prox: {:.4}'.format(finger_2a_prox_angle))
    print('Finger_2_prox: {:.4}'.format(finger_2p_prox_angle))
    print('Finger_2_prox: {:.4}\n'.format((finger_2p_prox_angle + finger_2a_prox_angle) / 2))

    print('Finger_2_dist: {:.4}'.format(finger_2a_dist_angle))
    print('Finger_2_dist: {:.4}'.format(finger_2p_dist_angle))
    print('Finger_2_dist: {:.4}'.format((finger_2p_dist_angle + finger_2a_dist_angle) / 2))"""

