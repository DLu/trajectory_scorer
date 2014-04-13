#!/usr/bin/python

import rospy
from math import radians
from trajectory_scorer.planner_interface import PlannerInterface
from trajectory_scorer.heatmap import heatmap
from numpy import arange

rospy.init_node('score_script')

p = PlannerInterface()
p.set_critics(['GoalAlign'])
p.set_goal(-3.0, 0.0, radians(0))

D = []

SAVES = {
    'x': (0.0, 4.0, .05),
    'y': (.1, .1025, .025),
    'z': (0.0, 3.15, 0.5),    
    'cvx': (.1, .5, .05),
    'cvy': (0, .2, .1),
    'cvz': (-.5, .5, .05)
}

X_AXIS = 'cvz'
Y_AXIS = 'y'

R1 = apply(arange, SAVES[Y_AXIS])
R2 = apply(arange, SAVES[X_AXIS])

M = {}
mx = 0.0
for v1 in R1:
    M[Y_AXIS] = v1
    r = []
    for v2 in R2:
        M[X_AXIS] = v2
        s = p.score(M.get('x', 0.00356),
                  M.get('y', 0),
                  M.get('z', -.44), 0, 0, 0, 
                  M.get('cvx', 0.0), 
                  M.get('cvy', 0), 
                  M.get('cvz', 0)
                 )
        r.append( s )
        mx = max(mx, s)
    D.append(r)
    

negs = False

for r in D:
    for i in range(len(r)):
        if r[i]<0.0:
            r[i] = mx + 1.0
            negs = True
T = ', '.join(p.critics)

heatmap(D, negs, x_axis=R2, y_axis=R1, x_label=X_AXIS, y_label=Y_AXIS, label=T)

