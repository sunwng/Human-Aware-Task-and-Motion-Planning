import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
from numpy.core.fromnumeric import reshape 

def plot_1dgrp(y_samplepahts, meanpath, ntraj, id):
    x_samples = np.linspace(0, 1, int(ntraj)).reshape((-1,1))
    plt.figure(figsize=(10,4))
    for y_samples in y_samplepahts:
        plt.plot(x_samples, y_samples)
    plt.title("Sampeld Trajectory: Joint{}".format(id))
    # plt.scatter(x_anchors, anchors, c='r', s=60)
    # plt.plot(x_samples, meanpath)
    plt.show()

def plot_fulldof(samplepahts, meanpath, ntraj, id):
    x_samples = np.linspace(1, ntraj, int(ntraj)).reshape((-1,1))
    reshape_sample = samplepahts[0]
    reshape_sample = reshape_sample.T
    reshape_mean   = meanpath.T 
    plt.figure(figsize=(15,10))
    for idx, (sample, mean) in enumerate(zip(reshape_sample, reshape_mean)):
        plt.plot(x_samples,sample, label="SampledJoi{}".format(idx+1))
        plt.scatter(x_samples, mean, s=15, label="MeanJoi{}".format(idx+1))
    plt.legend(loc="upper left", fontsize=7)
    plt.title("#{}.Sampeld Trajectory".format(id), fontsize=20)
    plt.ylim(-np.pi, np.pi)
    plt.xlim(1,ntraj)
    plt.xlabel("Step", fontsize=15)
    plt.ylabel("Radian", fontsize=15)
    # plt.scatter(x_anchors, anchors, c='r', s=60)
    # plt.plot(x_samples, meanpath, 'r--o')
    plt.show()

def plot_2d(samplepaths, meanpath, ntraj, id):
    plt.figure(figsize=(15,10))
    for lines, mean in zip(samplepaths, meanpath):
        plt.plot(lines[:][0],lines[:][1], label="SampledJoi")
    plt.legend(loc="upper left", fontsize=7)
    plt.title("#{}.Sampeld Trajectory".format(id), fontsize=20)
    plt.ylim(0, 20)
    plt.xlim(0, 20)
    plt.xlabel("X", fontsize=15)
    plt.ylabel("Y", fontsize=15)
    plt.show()
    # plt.scatter(x_anchors, anchors, c='r', s=60)
    # plt.plot(x_samples, meanpath, 'r--o')


