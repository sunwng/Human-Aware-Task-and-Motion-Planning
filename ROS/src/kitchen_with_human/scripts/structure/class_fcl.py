import fcl 
import numpy as np 
from tf.transformations import quaternion_from_euler as rot2quat
import math 

def make_box(name, pos, rot, size): 
    return {"name":name, "type":"box", "info":pos+rot+size}

""" Flexible Collision Library """
class PyFCL:
    def __init__(self, _num_max_contacts = 100, _verbose = True):
        self.geom_lst = []
        self.objs_lst = []
        self.name_lst = [] 
        self.verbose  = _verbose

    # capsule type: [position[0], position[1], position[2], rpy[0], rpy[1], rpy[2], height, radius]
    # box type: [position[0], position[1], position[2], rpy[0], rpy[1], rpy[2], size[0], size[1], size[2]]
    def fclize_obj(self, _name, _type, obj_info): 
        obj_name, obj_type = str(_name), str(_type) 
        if obj_type == "capsule":
            obj_pos, obj_rpy, obj_height, obj_radius = obj_info[:3], obj_info[3:6], obj_info[6], obj_info[7]
            obj_q = rot2quat(obj_rpy[0], obj_rpy[1], obj_rpy[2]) 
            obj_t = np.array(obj_pos)
            obj_g = fcl.Capsule(obj_radius, obj_height)
            obj_t = fcl.Transform(obj_q, obj_t)
            obj_o = fcl.CollisionObject(obj_g, obj_t)
 
        if obj_type == "box": 
            obj_pos, obj_rpy, obj_size = obj_info[:3], obj_info[3:6], obj_info[6:] 
            obj_q = rot2quat(obj_rpy[0], obj_rpy[1], obj_rpy[2]) 
            obj_t = np.array(obj_pos)            
            obj_g = fcl.Box(obj_size[0], obj_size[1], obj_size[2])
            obj_t = fcl.Transform(obj_q, obj_t)
            obj_o = fcl.CollisionObject(obj_g, obj_t)
        self.geom_lst.append(obj_g)
        self.objs_lst.append(obj_o)
        self.name_lst.append(obj_name)
        return obj_o 

    def one2many_cc(self, obj, obj_list):
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(obj_list)
        manager.setup()
        req = fcl.CollisionRequest()
        cdata = fcl.CollisionData(request = req)
        manager.collide(obj, cdata, fcl.defaultCollisionCallback)
        collision = cdata.result.is_collision
        if self.verbose: 
            objs_in_collision = set() 
            # Create map from geometry IDs to objects
            geom_id_to_obj  = {id(geom) : obj for geom, obj in zip(self.geom_lst, self.objs_lst)}
            # Create map from geometry IDs to string names
            geom_id_to_name = {id(geom) : name for geom, name in zip(self.geom_lst, self.name_lst)}
            for contact in cdata.result.contacts:
                coll_geom_0 = contact.o1
                coll_geom_1 = contact.o2

                # Get their names
                coll_names = [geom_id_to_name[id(coll_geom_0)], geom_id_to_name[id(coll_geom_1)]]
                coll_names = tuple(sorted(coll_names))
                objs_in_collision.add(coll_names)

            for coll_pair in objs_in_collision:
                pass
                print('*WARNING* {} in collision with object {}!'.format(coll_pair[0], coll_pair[1]))
        if collision: 
            return True 
        else: 
            return False         
    
    def many2many_cc(self, obj_list1, obj_list2): 
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager2 = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(obj_list1)
        manager2.registerObjects(obj_list2)
        manager.setup()
        req = fcl.CollisionRequest()
        cdata = fcl.CollisionData(request = req)
        manager.collide(manager2, cdata, fcl.defaultCollisionCallback)
        collision = cdata.result.is_collision
        if self.verbose: 
            objs_in_collision = set() 
            # Create map from geometry IDs to objects
            geom_id_to_obj  = {id(geom) : obj for geom, obj in zip(self.geom_lst, self.objs_lst)}
            # Create map from geometry IDs to string names
            geom_id_to_name = {id(geom) : name for geom, name in zip(self.geom_lst, self.name_lst)}
            for contact in cdata.result.contacts:
                coll_geom_0 = contact.o1
                coll_geom_1 = contact.o2

                # Get their names
                coll_names = [geom_id_to_name[id(coll_geom_0)], geom_id_to_name[id(coll_geom_1)]]
                coll_names = tuple(sorted(coll_names))
                objs_in_collision.add(coll_names)

            for coll_pair in objs_in_collision:
                pass
                print('*WARNING* {} in collision with object {}!'.format(coll_pair[0], coll_pair[1]))
        if collision: 
            return True 
        else: 
            return False         

    def one2one_collision_check(self, obj_info, link_list, type):
        link_ocap_lst = [] 
        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()
        obj_type = obj_info[0]
        obj_name = obj_info[1]
        obj_float_info = [float(param) for param in obj_info[2:]]
        obj_position = np.array([obj_float_info[0], obj_float_info[1], obj_float_info[2]])
        qx_obj, qy_obj, qz_obj, qw_obj= rot2quat(obj_float_info[3], obj_float_info[4], obj_float_info[5])
        obj_quaternion = np.array([qx_obj, qy_obj, qz_obj, qw_obj])

        if obj_type == "box": 
            obj_gb = fcl.Box(obj_float_info[6], obj_float_info[7], obj_float_info[8])
            obj_tb = fcl.Transform(obj_quaternion, obj_position)
            obj_ob = fcl.CollisionObject(obj_gb, obj_tb)
        self.geom_lst.append(obj_gb)
        self.objs_lst.append(obj_ob)
        self.name_lst.append(obj_name)

        if type == "capsule":
            for idx, link_info in enumerate(link_list): 
                link_name = str(link_info[0])
                link_float_info = [float(param) for param in link_info[1:]]
                link_position = np.array([link_float_info[0], link_float_info[1], link_float_info[2]])
                qx_link, qy_link, qz_link, qw_link = rot2quat(link_float_info[3], link_float_info[4], link_float_info[5])
                link_quaternion = np.array([qx_link, qy_link, qz_link, qw_link])  
                link_height, link_rad = link_float_info[6], link_float_info[7]
                link_gcap = fcl.Capsule(link_rad, link_height)
                link_tcap = fcl.Transform(link_quaternion, link_position)
                link_ocap = fcl.CollisionObject(link_gcap, link_tcap)
                self.geom_lst.append(link_gcap)
                self.objs_lst.append(link_ocap)
                self.name_lst.append(link_name)
                if link_height ==0 or link_rad ==0:
                    pass
                else:
                    link_ocap_lst.append(link_ocap)
                
        if type == "box": 
            for idx, link_info in enumerate(link_list): 
                link_name = str(link_info[0])
                link_float_info = [float(param) for param in link_info[1:]]
                link_position = np.array([link_float_info[0], link_float_info[1], link_float_info[2]])
                qx_link, qy_link, qz_link, qw_link = rot2quat(link_float_info[3], link_float_info[4], link_float_info[5])
                link_quaternion = np.array([qx_link, qy_link, qz_link, qw_link])  
                link_x, link_y, link_z = link_float_info[6], link_float_info[7], link_float_info[8]
                link_gcap = fcl.Box(link_x, link_y, link_z)
                link_tcap = fcl.Transform(link_quaternion, link_position)
                link_ocap = fcl.CollisionObject(link_gcap, link_tcap)
                self.geom_lst.append(link_gcap)
                self.objs_lst.append(link_ocap)
                self.name_lst.append(link_name)
                link_ocap_lst.append(link_ocap)   
        in_collision = np.ones(len(link_ocap))

        ret = fcl.collide(obj_ob, link_ocap, req, res)
        if ret == 0:
            in_collision[idx] = 0

        if sum(in_collision) == 0:
            return False
        else: 
            return True

if __name__ == "__main__": 
    fclize = PyFCL() 
    objinfo1 = [1, 2, 3, 0, 0, 0, 1, 1]
    type1 = 'capsule'
    name1 = 'obs1'

    objinfo2 = [1, 2, 3, 0, 0, 0, 1, 1, 1]
    type2 = 'box'
    name2 = 'obs2'

    objinfo3 = [1, 2, 3, 0, 0, 0, 1, 1, 1]
    type3 = 'box'
    name3 = 'obs3'

    objinfo4 = [1, 2, 3, 0, 0, 0, 1, 1, 1]
    type4 = 'box'
    name4 = 'obs4'

    a = fclize.fclize_obj(name1, type1, objinfo1)
    b = fclize.fclize_obj(name2, type2, objinfo2)
    c = fclize.fclize_obj(name3, type3, objinfo3)
    d = fclize.fclize_obj(name4, type4, objinfo4)

    obj_list = [a,b]
    obj_list2 =[c,d]
    ret = fclize.one2many_cc(c, obj_list)
    ret2 = fclize.many2many_cc(obj_list, obj_list2)
    print(ret)
    print(ret2)