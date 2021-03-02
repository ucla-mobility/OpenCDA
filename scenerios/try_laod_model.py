# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 10:18:50 2021

@author: anoop
"""

from model.GFS.GFS_controller import GFSController
from model.GFS.FISmoduleGFSBestMergePoint import FIS


import numpy as np
import pdb
import pickle

# load rules for GFS
with open(r'/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-theBest37-Safe.pickle','rb') as f:
    gfs_m_speed = pickle.load(f)

with open(r'/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-pl-score.pickle','rb') as g:
    gfs_pl_score = pickle.load(g)

with open(r'/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestGFS_PL_speed.pickle','rb') as h:
    gfs_pl_speed = pickle.load(h)

print('Done import model!')

