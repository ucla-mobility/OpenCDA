# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 10:18:50 2021

@author: anoop
"""

import pickle
import pdb
import sys
import model.GFS.FISmoduleGFSBestMergePoint as FISmoduleGFSBestMergePoint
sys.modules['FISmodule'] = FISmoduleGFSBestMergePoint
sys.modules['FISmoduleGFSBestMergePoint'] = FISmoduleGFSBestMergePoint

with open('/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-theBest37-Safe.pickle', 'rb') as f:
    gfs_m = pickle.load(f)

with open('/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-pl-score.pickle', 'rb') as g:
    gfs_pl_score = pickle.load(g)
    
with open('/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestGFS_PL_speed.pickle', 'rb') as h:
    gfs_pl_speed = pickle.load(h)

print('Done import model!')