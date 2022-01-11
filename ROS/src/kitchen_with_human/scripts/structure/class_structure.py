import numpy as np
from numpy.lib.shape_base import _put_along_axis_dispatcher 
from utils.utils_fk import *
from utils.utils_ik import *
from utils.utils_structure import *
from class_parser import Parser
from config import Config
import copy 

class CAPSULE():
    def __init__(self, scale=[], cap_radius = float(0), cap_height = float(0), cap_p= column_v(0,0,0), 
                cap_R=rot_e(), cap_center_p = column_v(0, 0, 0), cap_size=column_v(0,0,0), cap_t_offset=np.eye(4)):
        self.scale = scale 
        self.radius = cap_radius
        self.height  = cap_height
        self.p = cap_p
        self.R= cap_R
        self.center_p = cap_center_p
        self.size = cap_size
        self.T_offset= cap_t_offset

class JOINT():
    def __init__(self, name=str('joint'), id=int(0), mother=0, child=[0], q= float(0), a= column_v(1,1,1), b=column_v(0,0,0),
                 p=column_v(0,0,0), R=rot_e(), R_offset = Rotation_E(), dq = float(0), v=0, w = 0, type=''):
        self.name = name 
        self.id = id 
        self.mother = mother
        self.child = child 
        self.q = q
        self.a = a 
        self.b = b 
        self.p = p 
        self.R = R 
        self.R_offset =R_offset
        self.dq = dq 
        self.v = v 
        self.w = w
        self.type = type

class LINK(CAPSULE):
    def __init__(self, name=str('link'), joint_id=int(0), joint_name = str('mother_joint'), 
                P_offset=column_v(0,0,0), R_offset=rot_e(), cap_center_p = column_v(0, 0, 0),
                scale= [] , cap_radius = float(0), cap_height = float(0), cap_t_offset =  np.eye(4),
                mesh_path=str('path'), cap_p = column_v(0,0,0), cap_R=rot_e(), 
                color="0.2 0.2 0.2 1", collision_type="mesh", cap_size=column_v(0,0,0)):
        CAPSULE.__init__(self, scale, cap_radius, cap_height, cap_p, cap_R, cap_center_p, cap_size, cap_t_offset)
        self.name           = name 
        self.joint_id       = joint_id 
        self.joint_name     = joint_name 
        self.P_offset       = P_offset
        self.R_offset       = R_offset 
        self.mesh_path      = mesh_path 
        self.color          = color 
        self.collision_type = collision_type

        # Capsule 
        self.cap = CAPSULE()
        self.cap.scale = scale
        self.cap.radius = cap_radius 
        self.cap.height = cap_height
        self.cap.p = cap_p
        self.cap.R = cap_R
        self.cap.T_offset = cap_t_offset #In case of mesh type
        self.cap.center_p = cap_center_p #In case of mesh type
        self.cap.size = cap_size #In case of box type
        
class CHAIN:
    def __init__(self, file_name , verbose=False):
        self.robot = get_name_from_urdf(file_name)
        self.parser = Parser(file_name)
        self.config = Config(self.robot)
        self.joint = []
        self.link  = [] 
        self.verbose    = verbose

    def fk_chain(self, idx_to):
        if idx_to == 0:
            return None 
        idx_fr = self.joint[idx_to-1].mother 
        if idx_fr != 0:
            joint_fr = self.joint[idx_fr-1]
            joint_to = self.joint[idx_to-1]
            joint_to.p = (np.matmul(joint_fr.R, joint_to.b) + joint_fr.p).astype(float)
            if (joint_to.a == column_v(1,1,1)).all():#(joint_to.a == column_v(0,0,0)).all:
                joint_to.R = np.matmul(joint_fr.R, joint_to.R_offset)
            else:   
                joint_fr_after = np.matmul(joint_fr.R, joint_to.R_offset)
                joint_to.R = np.matmul(joint_fr_after, rodrigues(joint_to.a, joint_to.q)).astype(float)

        for child_idx in self.joint[idx_to-1].child:
            self.fk_chain(child_idx)

    def fk_link_chain(self, idx_to):
        if idx_to == 0:
            # For the base link center_p
            joint_to = self.joint[idx_to]
            link_to = self.link[idx_to]
            p = joint_to.b
            r = joint_to.R
            T_capsule = np.matmul(pr2t(p, r) ,link_to.T_offset)
            link_to.cap.center_p = t2p(T_capsule)
            return None 
        idx_fr = self.joint[idx_to-1].mother 
        if idx_fr != 0:
            link_fr = self.link[idx_fr-1]
            link_to = self.link[idx_to-1]
            joint_fr = self.joint[idx_fr-1]
            joint_to = self.joint[idx_to-1]
            joint_to.p = (np.matmul(joint_fr.R, joint_to.b) + joint_fr.p).astype(float)
            link_to.cap.p = joint_to.p + link_fr.P_offset 

            #link_to.cap.center_p = (np.matmul(joint_fr.R, joint_to.b) + link_fr.cap.center_p).astype(float)

            if (joint_to.a == column_v(1,1,1)).all():#(joint_to.a == column_v(0,0,0)).all:
                joint_to.R = np.matmul(joint_fr.R, joint_to.R_offset)
            else:   
                joint_fr_after = np.matmul(joint_fr.R, joint_to.R_offset)
                joint_to.R = np.matmul(joint_fr_after, rodrigues(joint_to.a, joint_to.q)).astype(float)

            T_capsule = np.matmul(pr2t(joint_to.p, joint_to.R) ,link_to.T_offset)
            link_to.cap.center_p = t2p(T_capsule)
            link_to.cap.R = t2r(T_capsule)

        for child_idx in self.joint[idx_to-1].child:
            self.fk_link_chain(child_idx)


    # Inverse Kinematics LM
    def aug_inverse_kinmatics_LM(self, variables):
        ctrl_num = variables["joint_ctrl_num"]
        # Get revolute_joint for IK 
        rev_joi = get_rev_joi_chain(self.joint, ctrl_num)
        J, err, we = get_aug_ik_ingredients(self.joint, variables)
        self.fk_chain(1)
        self.fk_link_chain(1)

        Ek = np.matmul(err.T, we) 
        Ek = np.matmul(Ek, err) 
        while True: 
            dq, _ = damped_ls(J, err)
            move_joints(self.joint, rev_joi, dq)
            self.fk_chain(1)
            J, err, _ = get_aug_ik_ingredients(self.joint, variables) #TODO: Change it to Origin version 
            Ek2 = np.matmul(err.T, we)
            Ek2 = np.matmul(Ek2, err)
            if Ek2 < 1e-12:
                break
            elif Ek2 < Ek:
                Ek = Ek2 
            else: 
                move_joints(self.joint, rev_joi, -dq) #revert
                self.fk_chain(1)
                self.fk_link_chain(1)
                break 

    def get_q_from_ik(self, variable):
        self.fk_chain(1)
        self.aug_inverse_kinmatics_LM(variable)
        q_list = get_q_chain(self.joint)
        return q_list 

    """ CONNECT JOINT&LINK TREE """
    def add_joi_to_robot(self):
        # Other joints     
        for joint in self.parser.joint:
            self.joint.append(JOINT(name=joint['name'], 
                                        id=joint['id'], 
                                        mother = self.parser.get_mother(joint['parent']), 
                                        child=self.parser.get_child_joint_tree(joint["child"]), q=0, 
                                        a=column(joint['axis']), 
                                        b=column(joint['xyz']), 
                                        p=column_v(0, 0, 0), 
                                        R=np.eye(3), 
                                        R_offset=make_rotation(rad=joint['rpy']),
                                        type=joint['type']))    
        # CONNECT ALL JOINTS TO THE ROBOT 
        self.fk_chain(1)
        """ Verbose Function """
        if self.verbose:
            for i in self.joint: 
                print("Name: {}, ID: {}, Mother_joint: {}, Child_joint: {}, Type: {}, Axis: {}, B_offset:{}".
                    format(i.name.ljust(35), str(i.id).ljust(2), str(i.mother).ljust(2), 
                           str(i.child).ljust(11), i.type.ljust(10), str(i.a.T).ljust(16), 
                           str(i.b.T).ljust(5)))
                print("Current Position: {}\nCurrent Rotation: \n{}\n".format(i.p.T, i.R))         

    def add_link_to_robot(self):
        """ CONNECT LINK TREE """
        for link, capsule in zip(self.parser.link, self.config.capsule):
            self.link.append(LINK(name=link['name'], 
                                        joint_id = self.parser.get_link_joint_id(link['name']), 
                                        joint_name=self.parser.get_link_joint_name(link['name']), 
                                        P_offset = column(link["P_offset"]), 
                                        R_offset = make_rotation(rad=link["R_offset"]), 
                                        scale = link['scale'], 
                                        mesh_path = link["filepath"], 
                                        collision_type = link["collision_type"], 
                                        cap_p    = column_v(0.0, 0.0, 0.0), 
                                        cap_R      = np.eye(3),
                                        cap_radius = capsule["radius"],
                                        cap_size  = capsule["size"],
                                        cap_t_offset = capsule["T_offset"],
                                        cap_center_p = column_v(0.0, 0.0, 0.0),
                                        cap_height   = capsule["height"],
                                        color = get_color_from_urdf(link['color'])))
        # CONNECT ALL LINKS TO THE ROBOT 
        # self.fk_link_chain(1) #TODO: UR5e doen't work
        if self.verbose:
            print('len_link', len(self.link))
            for i in self.link:
                print("Name: {}, Connected Joint ID: {}, Connected Joint name: {}, \nMesh Path: {}, \
                    \nCollision Type: {}, Radius: {}, Heihgt: {}, Cap_Position: {}, \nCenter Position: {}\n".
                    format(i.name.ljust(30), str(i.joint_id).ljust(2), i.joint_name.ljust(30), 
                           i.mesh_path, i.collision_type, str(i.radius).ljust(10), str(i.height).ljust(10), 
                           i.cap.p.T, i.cap.center_p))



