import numpy as np
from time import time 
import math 
from class_kdtree import KD_TREE

def euclidean_dist(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2)
                          for i in range(len(point1))]))

class SimpleTree: 
    def __init__(self, dim):
        self._parents_map = {}
        self._kd = KD_TREE(dim)

    def add_new_node(self, point, parent=None):
        node_id = self._kd.insert(point)
        self._parents_map[node_id] = parent
        return node_id 
    
    def get_parent(self, child_id): 
        return self._parents_map[child_id]
    
    def get_point(self, node_id):
        return self._kd.get_node(node_id).point 

    def get_num_nodes(self):
        return len(self._parents_map)

    def get_nearest_node(self, point):
        return self._kd.find_nearest_point(point)

    def get_near_neighbor_node(self, query, radius): 
        return self._kd.find_points_within_radius(query, radius)

    def connect_path2root(self, leaf_node_id):
        path = []
        node_id = leaf_node_id
        while node_id is not None:
            path.append(self.get_point(node_id))
            node_id = self.get_parent(node_id)
        return path

    def get_total_nodes(self): 
        total_path = []
        total_length = self.get_num_nodes()
        print("Total length: {}".format(total_length))
        for node_id in range(total_length):
            total_path.append(self.get_point(node_id))
        return total_path 
""" Rapidly-exploring Radom Tree* """
class RRT_STAR: 
    def __init__(self, _robot, _is_in_collision): 
        self.robot = _robot 
        self.is_in_collision = _is_in_collision
        self.q_step_size = 0.02
        self.max_n_nodes = int(1e5)
        self.target_p = 0.5
        self.radius = 0.1
        self.cost = {}

    def get_distance(self, p):
        dist = 0
        prev = p[0]
        for q in p[1:]:
            dist += np.linalg.norm(q - prev)
            prev = q
        return dist


    def get_near_neighbor(self): 
        pass
    
    def get_cost(self, nearest_node_id, q_nearest, q_new):
        cost = self.cost[nearest_node_id] + euclidean_dist(q_new, q_nearest)
        return cost 

    def get_minimum_cost(self, tree, neighbor_nodes, q_new, cost, nearest_node_id):
        for node_id in neighbor_nodes: 
            q_neighbor = tree.get_point(node_id)
            cost_new = self.get_cost(node_id, q_neighbor, q_new)
            if cost_new < cost and self.q_path_collision_check(q_new, q_neighbor):
                cost = cost_new
                nearest_node_id = node_id  
        return cost, nearest_node_id

    def rewire(self, tree, neighbor_nodes, q_new, q_new_id):
        for node_id in neighbor_nodes: 
            q_node = tree.get_point(node_id)
            is_collision_free = self.q_path_collision_check(q_new, q_node)
            cost_new = self.get_cost(node_id, q_node, q_new)

            if is_collision_free==True and cost_new < self.cost[node_id]:
                self.cost[node_id] = cost_new
                tree._kd.node_list[node_id] = q_new_id

    def q_path_collision_check(self, q0, q1):
        qs = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self.q_step_size))
        for q in qs:
            if self.is_in_collision(q):
                return False
        return True 

    def random_joints(self): 
        valid_q_list = [] 
        for i in range(self.robot.ctrl_joint_num):
            valid_q = np.random.uniform(self.robot.joint_limits_low[i], self.robot.joint_limits_high[i])
            # valid_q = np.random.random(1) * (self.robot.joint_limits_high[i] - self.robot.joint_limits_low[i]) + self.robot.joint_limits_low[i] 
            valid_q_list.append(valid_q)
        valid_q_list = np.array([valid_q_list])
        valid_q_list = valid_q_list.reshape([self.robot.ctrl_joint_num])
        return valid_q_list
    
    def extend(self, tree, q_near, q_near_id, q_target): 
        self.cost[0] = 0
        q_new = q_old = q_near
        q_new_id = q_old_id = q_near_id
        while True:
            # If q_new is same as q_target, It's enough to finisth this loop
            if (q_target == q_new).all():
                return q_new, q_new_id

            # If q_old is more closer from q_target than q_new, Change the node to q_old 
            if np.linalg.norm(q_target - q_new) > np.linalg.norm(q_old - q_target):
                return q_old, q_old_id

            q_old = q_new
            q_old_id = q_new_id
            q_new = q_new + min(self.q_step_size, np.linalg.norm(q_target - q_new)) \
                       * (q_target - q_new) / np.linalg.norm(q_target - q_new)
            # If the path is collision free, add the node into the tree   
            if not self.is_in_collision(q_new) and self.q_path_collision_check(q_old, q_new):
                q_new_id_ = tree.add_new_node(q_new, q_old_id)
                neighbor_node_list = tree.get_near_neighbor_node(q_new, self.radius)

                min_cost = self.get_cost(q_old_id, q_old, q_new)
                min_cost, q_new_id = self.get_minimum_cost(tree, neighbor_node_list, q_new, min_cost, q_new_id_)
                self.cost[q_new_id_] = min_cost

            else:
                return q_old, q_old_id
    
    def plan(self, q_start, q_target): 
        tree = SimpleTree(len(q_start)) 
        tree.add_new_node(q_start)
        s = time() 
        for n_nodes_sampled in range(self.max_n_nodes): 
            if n_nodes_sampled % 10 == 0: 
                print('RRT STAR: Sampled {} nodes'.format(n_nodes_sampled))
            q_rand = self.random_joints()
            node_id_nearest = tree.get_nearest_node(q_rand)[0]
            q_near_start = tree.get_point(node_id_nearest)  
            if np.random.random(1) < self.target_p: 
                q_sample = q_target # Make sure to reach target point. 
            else: 
                q_sample = q_rand   # Random Sampling in feasible joint range.
            q_new, q_target_reach_id = self.extend(tree, q_near_start, node_id_nearest, q_sample)
            neighbor_node_list = tree.get_near_neighbor_node(q_new, self.radius) # Get near neighbor nodes for comparison of cost 
            self.rewire(tree, neighbor_node_list, q_new, q_target_reach_id) # Cost Comparison + Rewire 

            if np.linalg.norm(q_new - q_target) < self.q_step_size:
                reached_target = True
                break 
        print('RRT STAR: Sampled {} nodes in {:.2f}s'.format(n_nodes_sampled, time() - s))

        path = []
        if reached_target:
            backward_path = [q_target]
            node_id = q_target_reach_id
            while node_id is not None:
                backward_path.append(tree.get_point(node_id))
                node_id = tree.get_parent(node_id)
            path = backward_path[::-1]
            print('RRT: Found a path! Path length is {}.'.format(len(path)))
        else:
            print('RRT: Was not able to find a path!')
        total_path = tree.get_total_nodes()
        return path, total_path
