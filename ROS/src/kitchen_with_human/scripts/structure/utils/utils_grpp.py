
import numpy as np 
import math 

def reshape_path(total_path, trajlen, ctrl_num):
    total_repath = []
    for path in total_path: 
        for i in range(ctrl_num):
            idx = trajlen*i
            next_idx = trajlen+idx  
            newpath = np.array([path[idx:next_idx]])
            if i == 0 : 
                repath = newpath 
            else:
                repath = np.append(repath, newpath, axis=0)
            trans_repath = repath.T
        total_repath.append(trans_repath)
    return total_repath 


def generate_traj(H, joint_start, joint_target, nsamples, lentraj, hgrp_params):
    _lentraj=lentraj
    _hyp_mean=hgrp_params
    _hyp_var=hgrp_params
    _hyp_hvar=hgrp_params
    DoFrandompath = []
    for idx, (start, target) in enumerate(zip(joint_start, joint_target)):
        H.set_x_anchor(_x_anchor=np.array([[start],[target]]).reshape(-1,1), _SAMEINTV=True, _EPSRU=True, _teps=0.01) #np.zeros(2).reshape(-1,1)
        H.compute_grp(_len=_lentraj,_hyp_mean=_hyp_mean,_hyp_var=_hyp_var,_hyp_hvar=_hyp_hvar)
        randompaths, _ = H.get_samplepaths(_npath=nsamples)
        meanpath = H.get_meanpaths()
        if idx == 0: 
            DoFmeanpath = meanpath
            DoFrandompath = randompaths

        else: 
            DoFmeanpath = np.hstack([DoFmeanpath, meanpath])
            DoFrandompath = np.hstack([DoFrandompath, randompaths])
    return DoFrandompath, DoFmeanpath

def is_in_collision(robot, joints, obs_info_lst):
    if robot.is_collision(joints, obs_info_lst):
        return True 
    return False 


def path_collcheck(path, obs_lst):
    if not obs_lst:
        pass
    else:
        for joint in path[0]: 
            optimial_joints = joint
            is_coll = is_in_collision(optimial_joints, obs_lst)
            if is_coll:
                return True 
    return False  

def distance(p1, p2):
    return math.sqrt((p1-p2)**2)

def get_shortestPath(paths):
    lines = [] 
    for path in paths: 
        joints = path.T
        line = 0
        for joint in joints: 
            for i in range(len(joint)):
                if i == (len(joint)-2): 
                    break 
                points = joint[i:i+2]
                line+=distance(points[0], points[1])
        lines.append(line)
        min_line = min(lines)
        min_id = lines.index(min_line)
    return paths[min_id], min_id 

def get_lineLength(path):
    joints = path.T
    line = 0
    for joint in joints: 
        for i in range(len(joint)):
            if i == (len(joint)-2): 
                break 
            points = joint[i:i+2]
            line+=distance(points[0], points[1])
    return line