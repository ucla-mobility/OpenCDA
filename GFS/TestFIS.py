# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 10:18:50 2021

@author: anoop
"""


from FISmoduleGFSBestMergePoint import FIS
from RunScenario import RunScenarioSUMO

import numpy as np
import pdb
import pickle

with open('BestFIS-theBest37-Safe.pickle','rb') as f:
    gfs_m = pickle.load(f)
    
with open('BestFIS-pl-score.pickle','rb') as g:
    gfs_pl_score = pickle.load(g)
    
with open('BestGFS_PL_speed.pickle','rb') as h:
    gfs_pl_speed = pickle.load(h)


avg_speed = RunScenarioSUMO(gfs_pl_score,gfs_pl_speed, gfs_m, 1.0)

