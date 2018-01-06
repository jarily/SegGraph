# -*- coding: utf-8 -*-
"""
Created on Tue Oct 17 21:40:49 2017

@author: Jarily
"""

import numpy as np
pose_file='F:\\dataset\\poses\\05.txt'

fn='pose\seq'
fname=fn+'_05_pose'+'.txt'
f1=open(fname,'w')
with open(pose_file, 'r') as f:
    lines = f.readlines() 
    i=0
    for line in lines:
        #if(i==30):
        #    break
        T_w_cam0 = np.fromstring(line, dtype=float, sep=' ')
        #print(T_w_cam0)
        T_w_cam0 = T_w_cam0.reshape(3, 4)
        T_w_cam0 = np.vstack((T_w_cam0, [0, 0, 0, 1]))
        np.set_printoptions(precision=3, suppress=True)  #输出精度控制
        print(T_w_cam0)
        print(T_w_cam0[0][3])
        print(T_w_cam0[1][3])
        print(T_w_cam0[2][3])
        a = round(T_w_cam0[0][3],3)
        b = round(T_w_cam0[1][3],3)
        c = round(T_w_cam0[2][3],3)
        buff=str(a)+' '+str(b)+' '+str(c)+'\n'
        f1.write(buff)
        i=i+1
f1.close()