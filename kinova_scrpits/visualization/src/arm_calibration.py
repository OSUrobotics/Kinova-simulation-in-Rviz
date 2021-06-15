#! usr/bin/env python3


import numpy as np
import csv
import sys
import os


directory = os.path.dirname(os.path.realpath(__file__))

transforms = {}
tran_mat = np.zeros((4, 4))
# print(tran_mat)
trans_mat = []
ee_loc_inWorld = []
test_trans = [0, 0, 0, 0]

trans_O_B = np.array([[1, 0, 0, 0],
                      [0, 1, 0, -0.32],
                      [0, 0, 1, -0.02],
                      [0, 0, 0, 1]])


def transform_cal(transform_mat, transform_o_b):
    """Takes in the transform matrix from the arm(end effector to the base) and the initial "guess" transform matrix.
    returns the location of the end effector in the world frame."""

    loc_inWorld = transform_o_b @ transform_mat @ np.transpose([0, 0, 0, 1])  # Puts the end effector location in the world frame

    return loc_inWorld


for k in range(1, 5):

    with open(directory + '/data/TransformMatrix_' + str(k) + '.0.csv', newline='') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                tran_mat[j][i] = float(col)

        # print(tran_mat)
        transforms[str(k)] = tran_mat
    trans_mat.append(transform_cal(tran_mat, trans_O_B))  # calls the function to get the end effector location in world frame


########################################################################################################################
# Generate the updated translation portion of the transformation matrix
########################################################################################################################

ee_loc_inWorld = trans_mat

x = 0
y = 0
z = 0

for i in range(len(ee_loc_inWorld)):
    # sums the x, y, and z portions
    # If the points give the real location they should average to 0(the origin)

    x += ee_loc_inWorld[i][0]
    y += ee_loc_inWorld[i][1]
    z += ee_loc_inWorld[i][2]

# Averages the x, y, z points giving the needed offset for the translation portion of the transform matrix
dx = x / len(ee_loc_inWorld)
dy = y / len(ee_loc_inWorld)
dz = z / len(ee_loc_inWorld)

trans_O_B_update = trans_O_B

# Updates the translation portion of the transform matrix from the robot base frame to the world frame
trans_O_B_update[0][-1] += -dx
trans_O_B_update[1][-1] += -dy
# Currently not updating the z portion due to not having accurate z points.
# trans_O_B_update[2][-1] += -dz

# print('dx: {0}'.format(dx))
# print('dy: {0}'.format(dy))
# print('dz: {0}'.format(dz))
# print(trans_O_B_update)

########################################################################################################################
# Generating the rotation matrix portion of the transformation matrix
########################################################################################################################

# Averages the vectors between the top two points and the bottom two points, giving the x-vector
v_12 = ee_loc_inWorld[0][:-1] - ee_loc_inWorld[1][:-1]
v_43 = ee_loc_inWorld[3][:-1] - ee_loc_inWorld[2][:-1]
x_vec = (v_12 + v_43) / 2

# Averages the vectors between the two points on each side, giving the y-vector
v_14 = ee_loc_inWorld[0][:-1] - ee_loc_inWorld[3][:-1]
v_23 = ee_loc_inWorld[1][:-1] - ee_loc_inWorld[2][:-1]
y_vec = (v_14 + v_23) / 2

# Force the y-vector to be orthogonal to the x-vector, creating the yprime-vector
yp_vec = y_vec - np.dot(x_vec, y_vec) * x_vec

# print(yp_vec)
# print(x_vec)

# Cross the x and yprime-vectors to get the z-vector that is orthogonal to the other two
z_vec = np.cross(x_vec, yp_vec)

# Generate an identity matrix and replace the respective row with the x, yprime, and z-vectors after normalizing them
rot_mat = np.identity(4)
rot_mat[0][:-1] = x_vec / np.linalg.norm(x_vec)
rot_mat[1][:-1] = yp_vec / np.linalg.norm(yp_vec)
rot_mat[2][:-1] = z_vec / np.linalg.norm(z_vec)

# print(rot_mat)

########################################################################################################################
# saving the translation and rotation portions of the new calibrated transformation matrix
########################################################################################################################

file = open(directory + "/TranslationMatrix.csv", "w")
wr = csv.writer(file, dialect='excel')

for i in range(len(trans_O_B_update)):
    wr.writerow(trans_O_B_update[i])

file.close()

file = open(directory + "/RotationMatrix.csv", "w")
wr = csv.writer(file, dialect='excel')

for i in range(len(rot_mat)):
    wr.writerow(rot_mat[i])

file.close()
