import numpy as np
from utils_fk import *
from utils_ik import *

D2R = np.math.pi/180 

def init_jc_structure(robot_jc):
    del robot_jc[:]
    return robot_jc 

def init_lc_structure(robot_lc):
    del robot_lc[:]
    return robot_lc 
    
def update_q_chain(robot_jc, q_list, ctrl_joint):
    for idx in range(len(robot_jc)):
        if idx < ctrl_joint:
            robot_jc[idx].q = q_list[idx]

def get_scale(robot_lc):
    link_lst = []
    arr = []
    for link in robot_lc: 
        for lc in link.scale.split():
            lc = float(lc)
            arr.append(lc)
        link_lst.append(arr)
        arr = []
    return link_lst

def get_viz_ingredients(p_list, rpy_list, mesh_path_list, scale_lst, color_lst):
    viz_links = []
    for p, rpy, mesh_path, scale, color in zip(p_list, rpy_list, mesh_path_list, scale_lst, color_lst):
        viz_links.append([p[0], p[1], p[2], rpy[0], rpy[1], rpy[2], mesh_path, scale[0], scale[1], scale[2], color[0], color[1], color[2], color[3]])
    return viz_links

def get_viz_box_ingredients(p_list, rpy_list, scale_list):
    viz_links = [] 
    for p, rpy, scale in zip(p_list, rpy_list, scale_list):
        viz_links.append([p[0], p[1], p[2], rpy[0], rpy[1], rpy[2], scale[0], scale[1], scale[2]])
    return viz_links 

def get_viz_cylinder_ingredients(p_list, rpy_list, radius_list, height_list):
    viz_links = [] 
    for p, rpy, radius, height in zip(p_list, rpy_list, radius_list, height_list):
        viz_links.append([p[0], p[1], p[2], rpy[0], rpy[1], rpy[2], radius, radius, height])
    return viz_links 

def get_viz_test_ingredients(mesh_path_list, scale_lst):
    viz_links = []
    for idx, (mesh_path, scale) in enumerate(zip(mesh_path_list, scale_lst)):
        viz_links.append([0, 0, idx*0.2, 0, 0, 0, mesh_path, scale[0], scale[1], scale[2]])
    return viz_links

def get_coll_ingredients(link_p_list, rpy_list, scale_list):
    coll_links = []
    for link_p, rpy, scale in zip(link_p_list, rpy_list, scale_list):
        coll_links.append([link_p[0], link_p[1], link_p[2], rpy[0], rpy[1], rpy[2], scale[0], scale[1], scale[2]])
    return coll_links

def get_cap_coll_ingredients(link_p_list, rpy_list, height_list, radius_list):
    coll_links = []
    for link_p, rpy, height, radius in zip(link_p_list, rpy_list, height_list, radius_list):
        coll_links.append([link_p[0], link_p[1], link_p[2], rpy[0], rpy[1], rpy[2], height, radius])
    return coll_links

def get_cap_fcl_ingredients(name_list, link_p_list, rpy_list, height_list, radius_list): 
    fcl_links = [{"name":name, "type":"capsule", "info":[link_p[0], link_p[1], link_p[2], 
                rpy[0], rpy[1], rpy[2], height, radius]} 
                for name, link_p, rpy, height, radius 
                in zip(name_list, link_p_list, rpy_list, height_list, radius_list)]
    return fcl_links

def get_box_fcl_ingredients(name_list, link_p_list, rpy_list, size_list): 
    fcl_links = [] 
    for name, link_p, rpy, size in zip(name_list, link_p_list, rpy_list, size_list):
        fcl_links.append([name, link_p[0], link_p[1], link_p[2], rpy[0], rpy[1], rpy[2], size[0], size[1], size[2]])
    return fcl_links

def get_cylinder_ingredients(link_p_list, rpy_list, height_list, radius_list):
    coll_links = []
    for link_p, rpy, height, radius in zip(link_p_list, rpy_list, height_list, radius_list):
        coll_links.append([link_p[0], link_p[1], link_p[2], rpy[0], rpy[1], rpy[2], radius, radius, height])
    return coll_links

def get_cap_R_chain(robot_lc):
    n_link = len(robot_lc)
    cap_R_list = []
    for idx in range(n_link):
        cap_R_list.append(robot_lc[idx].cap.R) 
    return cap_R_list 


def get_color_from_urdf(color_value):
    color_value_list = []
    for color in color_value.split():
        color_value_list.append(float(color))
    return color_value_list

def get_link_color(robot_lc):
    n_link = len(robot_lc)
    color_list = []
    for idx in range(n_link):
        color_list.append(robot_lc[idx].color)
    return color_list

def get_mesh_chain(robot_lc):
    n_link = len(robot_lc)
    mesh_path_list = []
    for idx in range(n_link):
        mesh_path_list.append(robot_lc[idx].mesh_path) 
    return mesh_path_list 

def get_mother_id_chain(robot_jc):
    n_jc = len(robot_jc)
    id_list = []
    for idx in range(n_jc):
        id_list.append(robot_jc[idx].mother)
    return id_list

def get_scale_chain(robot_lc):
    n_link = len(robot_lc)
    scale_list = [] 
    for idx in range(n_link):
        scale_list.append(robot_lc[idx].scale)
    return scale_list 

def get_axis_chain(robot_jc):
    n_link = len(robot_jc)
    axis_list =[]
    for idx in range(n_link):
        axis_list.append(robot_jc[idx].a)
    return axis_list 

def get_R_offset_chain(robot_jc):
    n_link = len(robot_jc)
    R_offset_list = []
    for idx in range(n_link):
        R_offset_list.append(robot_jc[idx].R_offset)
    return R_offset_list

def get_q_chain(robot_jc):
    n_joint = len(robot_jc)
    q_list = np.zeros((n_joint,1))
    for idx in range(n_joint):
        q_list[idx] = robot_jc[idx].q 
    return q_list 


def get_p_chain(robot_jc):
    n_joint = len(robot_jc)
    p_list = np.zeros((n_joint,3))
    for idx in range(n_joint):
        p_list[idx] = robot_jc[idx].p.T
    return p_list 

def get_cap_p_chain(robot_lc):
    n_joint = len(robot_lc)
    p_list = np.zeros((n_joint,3))
    for idx in range(n_joint):
        p_list[idx] = robot_lc[idx].cap.p.T
    return p_list     

def get_center_p_chain(robot_lc):
    n_link = len(robot_lc)#TODO:End joint is None, Should fix it later. 
    p_list = np.zeros((n_link,3))
    for idx in range(n_link):
        p_list[idx] = robot_lc[idx].cap.center_p.T
    return p_list   

def get_name_chain(robot_lc): 
    n_link = len(robot_lc)
    name_list = [] 
    for idx in range(n_link): 
        name_list.append(robot_lc[idx].name)
    return name_list 

def get_cap_R_chain(robot_lc):
    n_joint = len(robot_lc) 
    R_list = [] 
    for idx in range(n_joint):
        R_list.append(robot_lc[idx].cap.R)
    return R_list

def get_R_chain(robot_jc):
    n_joint = len(robot_jc)
    R_list = [] 
    for idx in range(n_joint):
        R_list.append(robot_jc[idx].R)
    return R_list

def get_cap_radius(robot_lc):
    n_link = len(robot_lc)
    cap_radius = []
    for idx in range(n_link):
        cap_radius.append(robot_lc[idx].cap.radius)
    return cap_radius

def get_height_chain(robot_lc):
    height_lst = []
    n_link = len(robot_lc)
    for idx in range(n_link):  
        height_lst.append(robot_lc[idx].cap.height)
    return height_lst

def get_cap_size_chain(robot_lc):
    size_lst = []
    n_link = len(robot_lc)
    for idx in range(n_link):  
        size_lst.append(robot_lc[idx].cap.size)
    return size_lst

def get_rpy_from_R_mat(R_list):
    rpy_list = []
    for idx in range(len(R_list)):
        rpy = decompose_rotation_matrix(R_list[idx])
        rpy_list.append(rpy)
    return rpy_list 

def get_rev_joi_chain(robot_jc, ctrl_num):
    n_joint = len(robot_jc)
    revolute_type = [] 
    for idx in range(n_joint):
        if robot_jc[idx].type == 'revolute' and idx <=ctrl_num: # This is essenstial condition, to remove gripper joint in IK slover
            revolute_type.append(robot_jc[idx].id)
    return revolute_type 


def add_capsule_between_joint(robot_jc):
    cap_height_list = []
    cap_p_list = []
    for jc in robot_jc:
        if jc.mother == 0:
            pass
        else:
            idx_from = jc.mother 
            idx_to = jc.id 
            jc_mother = get_jc_by_id(robot_jc, idx_from)
            p_fr = jc_mother.p
            p_to = jc.p 
            R_fr = jc_mother.R
            v_fr2to = p_to - p_fr 
            cap_height = np.linalg.norm(v_fr2to)
            cap_p = (p_fr + p_to)/2
            # diff_unit = v_fr2to/cap_height
            cap_height_list.append(cap_height)
            cap_p_list.append(cap_p)

    #if cap_height > 0:
    #    z_offset = 0.5*cap_height 
    #    ez = np.matmul(R_fr.T, v_fr2to)
    #    ex = np.random.randn(3,1)
    #    ey = -np.random.randn(3,1)
        #Q, _ = gram_schmidt([ez, ey, ex])
        #cap_R = [Q[:,2], Q[:,1], Q[:,0]]
    #else:
    #    z_offset = 0
    #    cap_R =np.eye(3,3)
    #    cap_height = 0.01 
    return cap_height_list, cap_p_list #, cap_R


def gram_schmidt(V):
    n = len(V)
    V = np.array(V)
    R = np.zeros((n,n))
    R[0, 0] = np.linalg.norm(V[:, 0])
    Q = np.zeros((n,n))
    Q[:, 0] = (V[:, 0]*R[0,0]).squeeze()
    for i in range(n):
        k = i+2
        R[0:k-1, k] = np.matmul(Q[:, 0:k-1].T, V[:, k])
        Q[:, k] = V[:, k]-np.matmul(Q[:, 0:k-1], R[0:k-1, k])
        R[k,k] = np.linalg.norm(Q[:,k])
        Q[:, k] = np.divide(Q[:, k], R[k,k])
    return Q, R 

    

def get_link_center_p(id, robot_jc):
    for jc in robot_jc: 
        if id == 1: 
            pass 
        if id == jc.id:
            joint_id = jc.id 
            idx_to = joint_id-1 
            idx_fr = idx_to + 1
            if idx_fr == len(robot_jc):
                break 
            center_p = (robot_jc[idx_to].p +robot_jc[idx_fr].p)/2. 
            return center_p
    return np.array([0, 0, 0])

def get_name_from_urdf(_filename):
    if "panda" in _filename: 
        return "panda"
    if "ur5e" in _filename: 
        return "ur5e"
    if "iiwa14" in _filename:
        return "iiwa14"   



# def get_link_height(id, robot_jc):
#     height_lst = [] 
#     for jc in robot_jc: 
#         if id == jc.id:
#             print(jc.name)
#             joint_id = jc.id 
#             idx_to = joint_id-1 
#             idx_fr = idx_to + 1
#             if idx_fr == len(robot_jc):
#                 break    
#             height_numpy = robot_jc[idx_fr].p - robot_jc[idx_to].p
#             height = np.linalg.norm(height_numpy)
#             print(height)
#             return height 

# def get_link_center_p(robot_jc, robot_lc):
#     link_center_p_list = []
#     for jc in robot_jc:
#         if 2<jc.id<8: #Forearm_link 
#             idx_to = jc.id +1
#             jc_child = get_jc_by_id(robot_jc, idx_to)
#             p_fr = jc.p
#             p_to = jc_child.p 
#             v_fr2to = (p_to + p_fr)/2 
#             link_center_p_list.append(v_fr2to)
#         else:
#             for lc in robot_lc:
#                 if jc.name == lc.joint_name:
#                     link_center_p_list.append(jc.p+lc.capsule.p)
#     return link_center_p_list

def get_b_chain(robot_jc):
    n_joint = len(robot_jc)
    b_list = np.zeros((n_joint,3))
    for idx in range(n_joint):
            b_list[idx] = robot_jc[idx].b.T
    return b_list 


def get_trans_mat_from_jc(robot_jc):
    b_list = get_b_chain(robot_jc) 
    trans_list = []
    for b in b_list:
        trans = Translation(b[0], b[1], b[2])
        trans_list.append(trans)
    return trans_list 


def get_rot_mat_from_jc(R_offset_list, axis_list, q_list): #4*4
    R_mat_list = []
    for idx, (R_offset, axis) in enumerate(zip(R_offset_list, axis_list)):
        if (axis == column_v(0,0,0)).all():
            R = Rotation_E()
            R = R.dot(R_offset)
            R_mat_list.append(R)

        elif (axis == column_v(0,0,1)).all():
            R = Rotation_Z(0)
            if idx<7:
                R=Rotation_Z(q_list[idx])
            R = R.dot(R_offset)
            R_mat_list.append(R)

        elif (axis == column_v(0,1,0)).all():
            R = Rotation_Y(0) 
            if idx<7:
                R=Rotation_Y(q_list[idx])
            R = R.dot(R_offset)
            R_mat_list.append(R)


        elif (axis == column_v(1,0,0)).all():
            R = Rotation_X(0) 
            if idx<7:
                R=Rotation_X(q_list[idx])
            R = R.dot(R_offset)
            R_mat_list.append(R)

    return R_mat_list 


def make_HT_dot(HT_mother, HT_child):
    HT = HT_mother.dot(HT_child)
    return HT 


def get_HT_mat_from_jc(robot_jc, q_list):
    HT_list = []
    id_list = get_mother_id_chain(robot_jc)
    trans_mat_list = get_trans_mat_from_jc(robot_jc) 
    axis_list = get_axis_chain(robot_jc)
    R_offset_list = get_R_offset_chain(robot_jc)
    R_mat_list = get_rot_mat_from_jc(R_offset_list, axis_list, q_list)
    for trans_mat, R_mat, mother_id in zip(trans_mat_list, R_mat_list, id_list):
        HT = HT_matrix(R_mat, trans_mat)
        HT_with_id = {'HT':HT, 'mother_id': mother_id}
        HT_list.append(HT_with_id)
    return HT_list 


def make_fk_chain(HT_list, robot_jc): 
    HT_dot_with_id = []
    HT_dot_with_id.append({'HT_dot':HT_list[0]['HT'].dot(HT_list[1]['HT']), 'id': robot_jc[0].id}) 
    for idx, HT in enumerate(HT_list):
        for HT_dot in HT_dot_with_id:
            if HT_dot['id'] == HT['mother_id']:
                HT_dot_with_id.append({'HT_dot': make_HT_dot(HT_dot['HT_dot'], HT['HT']), 'id': idx+1})
            else:
                pass
    return HT_dot_with_id