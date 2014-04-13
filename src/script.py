#!/usr/bin/python

import rospy
from math import radians
from trajectory_scorer.planner_interface import PlannerInterface
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
    

def limit(labels, n):
    labels = ['%.1f'%x for x in labels]
    NL = [''] * len(labels)
    NL[0] = labels[0]
    NL[-1] = labels[-1]
    N = len(labels)
    for i in range(0, N, max(N/n,1)):
        NL[i] = labels[i]
    return NL
    


import numpy
import matplotlib
import matplotlib.pyplot as plt
import pylab

negs = False

for r in D:
    for i in range(len(r)):
        if r[i]<0.0:
            r[i] = mx + 1.0
            negs = True


colors = [(pylab.cm.jet(i)) for i in xrange(1,255)] + [('white')] 
colors.reverse()
if negs:
    colors.append('black')

new_map = matplotlib.colors.LinearSegmentedColormap.from_list('new_map', colors, N=256)

mdata = numpy.array(D) 


fig, ax = plt.subplots()

heatmap = ax.pcolor(mdata,cmap=new_map)

ax.set_xticks(arange(len(R2))+0.5, minor=False)
ax.set_yticks(arange(len(R1))+0.5, minor=False)
ax.set_yticklabels(limit(R1,10))
ax.set_xticklabels(limit(R2,10))


T = ', '.join(p.critics)
ax.set_title(T)
plt.xlabel(X_AXIS)
plt.ylabel(Y_AXIS)
plt.show()

