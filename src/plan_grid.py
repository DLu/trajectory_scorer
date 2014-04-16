#!/usr/bin/python

import rospy
from math import radians
from trajectory_scorer.planner_interface import PlannerInterface
from trajectory_scorer.heatmap import *
from numpy import arange

rospy.init_node('score_script')

p = PlannerInterface()
#p.set_critics(['GoalDist'])
p.set_goal(3.0, 0.0, radians(0))

D = []

SAVES = {
    'x': (-1.0, 4.0, .25),
    'y': (-1.0, 1.01, .25),
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
        print v2,v1,cmd
        r.append( cmd )
    D.append(r)

extremes = [{},{},{}]
for i in range(3):
    for r in D:
        for a in r:
            extremes[i]['max'] = max(a[i], extremes[i].get('max', -1E6))
            extremes[i]['min'] = min(a[i], extremes[i].get('min', 1E6))
           
data =[]
cmap = {}
colors = []


for r in D:
    nr = []
    for x,y,z in r:
        xfrac = (x-extremes[0]['min'])/(extremes[0]['max']-extremes[0]['min'])
        yfrac = (y-extremes[1]['min'])/(extremes[1]['max']-extremes[1]['min'])
        zfrac = (z-extremes[2]['min'])/(extremes[2]['max']-extremes[2]['min'])
        key = xfrac,yfrac,zfrac
        if key in cmap:
            c = cmap[key]
        else:
            cmap[key] = len(colors)
            c = cmap[key]
            #if xfrac + yfrac + zfrac == 0:
            colors.append( (1-xfrac,1-yfrac,1-zfrac) )
            #else:
            #    colors.append( (0,0,0))
        nr.append(c)
    data.append(nr)

T = ', '.join(p.critics)

heatmap_custom_color(data, colors, x_axis=R2, y_axis=R1, x_label=X_AXIS, y_label=Y_AXIS, label=T)

