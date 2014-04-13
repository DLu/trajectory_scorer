#!/usr/bin/python

import rospy
from math import radians
from trajectory_scorer.planner_interface import PlannerInterface
from trajectory_scorer.heatmap import heatmap
from numpy import arange

def sig(b):
    if abs(b)<.001:
        return 1
    elif b>0:
        return 2
    else:
        return 0

def score(a):
    return sig(a[0])*9 + sig(a[1])*3 + sig(a[2])

rospy.init_node('score_script')

p = PlannerInterface()
p.set_critics(['GoalAlign'])
p.set_goal(3.0, 0.0, radians(0))

D = []

SAVES = {
    'x': (-1.0, 4.0, .25),
    'y': (-1.0, 1.0, .25),
    'z': (0.0, 3.15, 0.5)
}

X_AXIS = 'x'
Y_AXIS = 'y'

R1 = apply(arange, SAVES[Y_AXIS])
R2 = apply(arange, SAVES[X_AXIS])

M = {}
for v1 in R1:
    M[Y_AXIS] = v1
    r = []
    for v2 in R2:
        M[X_AXIS] = v2
        cmd = p.plan(M.get('x', 0.0),
                  M.get('y', 0.0),
                  M.get('z', 0.0)
                 )
        
        print v1,v2,cmd
        r.append( score(cmd) )
    D.append(r)
    

T = ', '.join(p.critics)

heatmap(D, False, x_axis=R2, y_axis=R1, x_label=X_AXIS, y_label=Y_AXIS, label=T)

