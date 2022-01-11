import math
import numpy as np
from collections import deque, namedtuple

Node= namedtuple('Node', 'point region axis active left right data')

def interval_condition(value, pre, post, dist):
    return pre -dist < value < post + dist 

def euclidean_dist(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2)
                          for i in range(len(point1))]))


class KD_TREE:
    def __init__(self, k=2, capacity=100000, limits = None):
        self.node_list = [None] * capacity  #node list  
        self.size = 0
        self.next_identifier = 0
        self.k = k  
        self.region = limits if limits is not None \
            else[[-float('inf'), float('inf')]] * k 
    

    def get_node(self, node_id):
        return self.node_list[node_id]
    

    def insert(self, point, data = None):
        if self.size == 0:
            if self.region is None:
                self.region = [[-math.inf, math.inf]] * self.k
            axis = 0
            return self.new_node(point, self.region, axis, data)

        current_id = 0 
        while True:
            parent_node = self.node_list[current_id] #parent node 
            axis = parent_node.axis
            if point[axis] < parent_node.point[axis]:
                next_id, left = parent_node.left, True 
            else:
                next_id, left = parent_node.right, False
            
            if next_id is None:
                break
                
            current_id = next_id

        region = parent_node.region[:]
        region[axis] = parent_node.region[axis][:]
        limit = parent_node.point[axis]

        if left:
            self.node_list[current_id] = parent_node._replace(left=self.size)
            region[axis][1] = limit
        else:
            self.node_list[current_id] = parent_node._replace(right=self.size)
            region[axis][0] = limit
        return self.new_node(point, region, (axis +1) % self.k, data)


    def new_node(self, point, region, axis, data):
        node = Node(point, region, axis, True, None, None, data)

        node_id = self.next_identifier 
        self.node_list[node_id] = node

        self.size += 1
        self.next_identifier += 1
        return node_id 
    

    def find_nearest_point(self, query, dist_fun = euclidean_dist):
        def get_properties(node_id):
            return self.node_list[node_id][:6]
        return nearest_point(query, 0 , get_properties, dist_fun)


    def find_points_within_radius(self, query, radius, dist_fun = euclidean_dist):
        def get_properties(node_id):
            return self.node_list[node_id][:6]
        return neighbor_within_radius(query, radius, 0, get_properties, dist_fun)


def nearest_point(query, root_id, get_properties, dist_fun = euclidean_dist):
    k=len(query)
    dist = float('inf')
    nearest_node_id = None 
    stack_node = deque([root_id])
    stack_look = deque()
    while stack_node or stack_look:
        if stack_node:
            node_id = stack_node.pop()
            look_node = False

        else:
            node_id = stack_look.pop()
            look_node = True 

        point, region, axis, active, left, right = get_properties(node_id)

        if look_node:
            insdie_region = True 
            for i in range(k):
                insdie_region &= interval_condition(query[i], region[i][0], region[i][1], dist)
            if not insdie_region:
                continue 

        if active:
            node_distance = dist_fun(query, point)
            if nearest_node_id is None or dist > node_distance:
                nearest_node_id = node_id 
                dist = node_distance
        
        if query[axis] < point[axis]:
            side_node = left
            side_look = right 
        else: 
            side_node = right
            side_look = left

        if side_node is not None:
            stack_node.append(side_node)
        if side_look is not None:
            stack_look.append(side_look)

    return nearest_node_id, dist 

def neighbor_within_radius(query, radius, root_id, get_properties, dist_fun = euclidean_dist):
    k=len(query)
    neighbors = []
    stack_node = deque([root_id])
    stack_look = deque()

    while stack_node or stack_look:
        if stack_node:
            node_id = stack_node.pop()
            look_node = False

        else:
            node_id = stack_look.pop()
            look_node = True 

        point, region, axis, active, left, right = get_properties(node_id)

        if look_node:
            insdie_region = True 
            for i in range(k):
                insdie_region &= interval_condition(query[i], region[i][0], region[i][1], radius)

            if not insdie_region:
                continue 

        if active:
            node_distance = dist_fun(query, point)
            if node_distance <= radius and node_distance != 0:
                neighbors.append((-node_distance, node_id))
        
        if query[axis] < point[axis]:
            side_node = left
            side_look = right 
        else: 
            side_node = right
            side_look = left

        if side_node is not None:
            stack_node.append(side_node)

        if side_look is not None:
            stack_look.append(side_look)
    return [item[1] for item in neighbors]