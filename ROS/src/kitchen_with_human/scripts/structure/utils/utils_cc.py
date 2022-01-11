import rospy
import math 
import fcl 
import numpy as np
from tf import transformations

def collision_checker(obj, links):
    x_obj, y_obj, z_obj = obj[0], obj[1], obj[2]
    roll_obj, pitch_obj, yaw_obj = obj[3], obj[4], obj[5] 
    l_obj, w_obj, h_obj = obj[6], obj[7], obj[8]
    qx_obj, qy_obj, qz_obj, qw_obj = transformations.quaternion_from_euler(roll_obj, pitch_obj, yaw_obj)
    q_obj = np.array([qx_obj, qy_obj, qz_obj, qw_obj])
    T_obj = np.array([x_obj, y_obj, z_obj])
    obj = fcl.Box(l_obj, w_obj, h_obj)
    tf_obj = fcl.Transform(q_obj, T_obj) 
    col_obj = fcl.CollisionObject(obj, tf_obj)

    num_obj = len(links)
    in_collision = np.ones(num_obj)
    col_request = fcl.CollisionRequest()
    col_result = fcl.CollisionResult()

    for i in range(len(links)):
        x_link, y_link, z_link = links[i][0], links[i][1], links[i][2] #translation
        roll, pitch, yaw = links[i][3], links[i][4], links[i][5] #orientation 
        l_link , w_link , h_link = links[i][6], links[i][7], links[i][8] #scale 
        qx_link, qy_link, qz_link, qw_link = transformations.quaternion_from_euler(roll, pitch, yaw)
        q_link = np.array([qx_link, qy_link, qz_link, qw_link]) 
        T_link = np.array([x_link, y_link, z_link])
        link = fcl.Box(l_link, w_link, h_link)
        tf_link = fcl.Transform(q_link, T_link)
        col_link = fcl.CollisionObject(link, tf_link)
        ret = fcl.collide(col_obj, col_link, col_request, col_result)
        if ret == 0:
            in_collision[i] = 0

    if sum(in_collision) == 0:
        return False
    else: 
        return True


def cap_collision_checker(obj, links):
    x_obj, y_obj, z_obj = obj[0], obj[1], obj[2]
    roll_obj, pitch_obj, yaw_obj = obj[3], obj[4], obj[5] 
    l_obj, w_obj, h_obj = obj[6], obj[7], obj[8]
    qx_obj, qy_obj, qz_obj, qw_obj = transformations.quaternion_from_euler(roll_obj, pitch_obj, yaw_obj)
    q_obj = np.array([qx_obj, qy_obj, qz_obj, qw_obj])
    T_obj = np.array([x_obj, y_obj, z_obj])
    obj = fcl.Box(l_obj, w_obj, h_obj)
    tf_obj = fcl.Transform(q_obj, T_obj) 
    col_obj = fcl.CollisionObject(obj, tf_obj)

    num_obj = len(links)
    in_collision = np.ones(num_obj)
    col_request = fcl.CollisionRequest()
    col_result = fcl.CollisionResult()

    for i in range(len(links)):
        x_link, y_link, z_link = links[i][0], links[i][1], links[i][2] #translation
        roll, pitch, yaw = links[i][3], links[i][4], links[i][5] #orientation 
        height_link , radius_link = links[i][6], links[i][7]#height, radius
        qx_link, qy_link, qz_link, qw_link = transformations.quaternion_from_euler(roll, pitch, yaw)
        q_link = np.array([qx_link, qy_link, qz_link, qw_link]) 
        T_link = np.array([x_link, y_link, z_link])
        cap_link = fcl.Capsule(radius_link, height_link)
        tf_link = fcl.Transform(q_link, T_link)
        col_link = fcl.CollisionObject(cap_link, tf_link)
        ret = fcl.collide(col_obj, col_link, col_request, col_result)
        if ret == 0:
            in_collision[i] = 0

    if sum(in_collision) == 0:
        return False
    else: 
        return True


def decompose_homogeneous_matrix(matrix):
    x, y, z = matrix[:-1, -1]
    R = matrix[:-1, :-1]
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], (math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([x, y, z, roll, pitch, yaw])



def cc_color(markers, link):
    markers[link].color.r = 1.0
    markers[link].color.g = 0.0 
    markers[link].color.b = 0.0 


