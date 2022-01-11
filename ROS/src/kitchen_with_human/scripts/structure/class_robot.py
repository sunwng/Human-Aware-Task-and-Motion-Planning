import rospy
from class_parser import *
from class_structure import *
from class_fcl import *
from utils.utils_rviz import *
from utils.utils_ik import *
from utils.utils_cc import *

def euclidean_dist(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2)
                          for i in range(len(point1))]))

class ROBOT:
    def __init__(self, _filename):
        rospy.init_node("Run_Robot")
        self.robotname      = get_name_from_urdf(_filename)

        self.pub_opt_path = rospy.Publisher('viz_optimal_path', MarkerArray, queue_size=10)
        self.pub_opt_line = rospy.Publisher('viz_optimal_line', Marker, queue_size=10)

        self.pub_obs      = rospy.Publisher('viz_obstacle', MarkerArray, queue_size=10)
        self.pub_obj      = rospy.Publisher('viz_objects', MarkerArray, queue_size=10)
        self.pub_mesh     = rospy.Publisher('viz_mesh', MarkerArray, queue_size=10)

        self.ee_pos_lst = []
        self.ee_pos_trg_lst = []

        self.chain          = CHAIN(_filename)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()
        self.joint_tree     = self.chain.joint
        self.link_tree      = self.chain.link
        self.parser         = Parser(_filename)   
        self.fcl            = PyFCL()
        self.config         = Config(self.robotname)
        self.joint_limits_low = self.config.joint_limits_low
        self.joint_limits_high = self.config.joint_limits_high
        self.start_joints    = self.config.start_joints
        self.ctrl_joint_num = self.config.ctrl_joint_num


    def publish_opt_path(self, q_list, r,g,b):
        update_q_chain(self.joint_tree, q_list, self.ctrl_joint_num)
        self.chain.fk_chain(1)
        ee_pos = self.chain.joint[8].p
        self.ee_pos_lst.append(ee_pos)

        if self.ee_pos_lst != []:
            # print('start_position',euclidean_dist(ee_pos, self.ee_pos_lst[-1]))
            if euclidean_dist(ee_pos, self.ee_pos_lst[-1]) > 0.01:
                self.ee_pos_lst = []

        p_list     = get_p_chain(self.joint_tree)
        R_list     = get_R_chain(self.joint_tree)
        rpy_list   = get_rpy_from_R_mat(R_list)
        mesh_list  = get_mesh_chain(self.link_tree)
        scale_list = get_scale(self.link_tree)
        color_list = get_link_color(self.link_tree)
        viz_links = get_viz_ingredients(p_list, rpy_list, mesh_list, scale_list, color_list)
        viz_opt_robot = publish_viz_robot(viz_links)
        viz_lines  = publish_viz_line(self.ee_pos_lst, r, g, b)
        self.pub_opt_path.publish(viz_opt_robot)
        if self.ee_pos_lst != []:
            self.pub_opt_line.publish(viz_lines)
    
    def publish_box_mesh(self, q_list):
        update_q_chain(self.joint_tree, q_list, self.ctrl_joint_num)
        self.chain.fk_link_chain(1)
        link_p_list = get_center_p_chain(self.link_tree)
        cap_R_list = get_cap_R_chain(self.link_tree)
        rpy_list = get_rpy_from_R_mat(cap_R_list)
        size_list = get_cap_size_chain(self.link_tree) 
        viz_box = get_viz_box_ingredients(link_p_list, rpy_list, size_list)
        viz_box_mesh = publish_box_mesh(viz_box)     
        self.pub_mesh.publish(viz_box_mesh)

    def publish_obj(self, objs):
        viz_obj = publish_viz_object(objs)
        self.pub_obj.publish(viz_obj)

    def publish_obs(self, obs):
        viz_obs = publish_viz_obs(obs)
        self.pub_obs.publish(viz_obs)

    def publish_sphere_mesh(self, q_list):
        update_q_chain(self.joint_tree, q_list, self.ctrl_joint_num)
        self.chain.fk_link_chain(1)
        link_p_list = get_center_p_chain(self.link_tree)
        cap_R_list = get_cap_R_chain(self.link_tree)
        rpy_list = get_rpy_from_R_mat(cap_R_list)
        height_list = get_height_chain(self.link_tree) #add_capsule_between_joint(self.joint_tree)
        radius_list = get_cap_radius(self.link_tree)
        viz_sphere = get_viz_cylinder_ingredients(link_p_list, rpy_list, radius_list, height_list)
        viz_sphere_mesh = publish_sphere_mesh(viz_sphere)
        self.pub_mesh.publish(viz_sphere_mesh)

    def is_collision(self, q_list, objs):
        update_q_chain(self.joint_tree, q_list, self.ctrl_joint_num)
        self.chain.fk_chain(1)
        self.chain.fk_link_chain(1)        
        cap_R_list = get_cap_R_chain(self.link_tree)
        rpy_list = get_rpy_from_R_mat(cap_R_list)
        link_p_list = get_center_p_chain(self.link_tree)
        name_list = get_name_chain(self.link_tree)
        size_list = get_cap_size_chain(self.link_tree)
        scale_list = get_scale_chain(self.link_tree)                  
        height_list = get_height_chain(self.link_tree)
        radius_list = get_cap_radius(self.link_tree)
        """ Caspsule """
        fcl_links = get_cap_fcl_ingredients(name_list, link_p_list, rpy_list, height_list, radius_list)
        fclized_link_lst = [self.fcl.fclize_obj(link["name"], link["type"], link["info"]) for link in fcl_links]
        fclized_obs_lst  = [self.fcl.fclize_obj(obs["name"], obs["type"], obs["info"]) for obs in objs]
        """ COLLISION CHECK """
        coll_req = self.fcl.many2many_cc(fclized_link_lst, fclized_obs_lst)
        if coll_req:
            # print("Collsion with Objects") 
            return True
        else: 
            # print("Collision Free")
            return False 