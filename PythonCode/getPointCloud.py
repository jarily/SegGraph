# -*- coding: utf-8 -*-
"""
Created on Thu Oct 19 22:11:13 2017

@author: Jarily
"""

import itertools
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import pykitti


# Change this to the directory where you store KITTI data
basedir = 'F:\dataset'

# Specify the dataset to load
sequence = '05'
dataset = pykitti.odometry(basedir, sequence)
#dataset = pykitti.odometry(basedir, sequence, frames=range(0, 50, 5))


second_pose = next(iter(itertools.islice(dataset.poses, 0, None)))

for c in dataset.poses:
    print(c)

np.set_printoptions(precision=4, suppress=True)  #输出精度控制
fn='data05\scan'
i=0
for c in dataset.velo:
    fname=fn+str(i)+'.txt'
    f=open(fname,'w')
    print(c[5])
    print(c.shape)
    #c=c[1:2]
    #c=c[0:c.shape[0],0:3]
    c=c[:,0:3]
    print(c[5])
    print(c.shape)
    print("----")
    for j in range(0,c.shape[0],1):
        buff=str(c[j][0])+' '+str(c[j][1])+' '+str(c[j][2])+'\n'
        f.write(buff)
    f.close()
    i=i+1
    #f2 = plt.figure()
    #ax2 = f2.add_subplot(111, projection='3d')
    '''
    velo_range = range(0, c.shape[0],100)
    ax2.scatter(c[velo_range, 0],
            c[velo_range, 1],
            c[velo_range, 2],
            c=c[velo_range, 3],
            cmap='gray')
    ax2.set_title('Third Velodyne scan (subsampled)')
    plt.show()
    '''




third_velo = next(iter(itertools.islice(dataset.velo, 9, None)))

# Display some of the data
np.set_printoptions(precision=3, suppress=True)  #输出精度控制


f2 = plt.figure()
ax2 = f2.add_subplot(111, projection='3d')
#Plot every 100th point so things don't get too bogged down
velo_range = range(0, third_velo.shape[0],100)
print(velo_range)
print(third_velo[0])
print(third_velo[0,0])
print(third_velo[0,1])
print(third_velo[0,2])
ax2.scatter(third_velo[velo_range, 0],
            third_velo[velo_range, 1],
            third_velo[velo_range, 2],
            c=third_velo[velo_range, 3],
            cmap='gray')
ax2.set_title('Third Velodyne scan (subsampled)')

plt.show()
