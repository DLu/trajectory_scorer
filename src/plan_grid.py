#!/usr/bin/python

import rospy
from math import radians
from trajectory_scorer.planner_interface import PlannerInterface
from trajectory_scorer.heatmap import *
from numpy import arange, append

rospy.init_node('score_script')

p = PlannerInterface()
#p.set_critics(['GoalDist'])
p.set_goal(3.0, 0.0, radians(0))

D = []

SAVES = {
    'x': (-1.0, 4.0, .25),
    'y': (-1.0, 1.0, .25),
    'z': (-3.15, 3.15, 0.5)
}

X_AXIS = 'x'
Y_AXIS = 'y'
R1 = apply(arange, SAVES[Y_AXIS]) 
R2 = apply(arange, SAVES[X_AXIS]) 
R1 = append(R1, SAVES[Y_AXIS][1])
R2 = append(R2, SAVES[X_AXIS][1])

M = {}
for i,v1 in enumerate(R1):
    print "%.2f"%(float(i)/len(R1))
    M[Y_AXIS] = v1
    r = []
    for v2 in R2:
        M[X_AXIS] = v2
        cmd = p.plan(M.get('x', 0.0),
                  M.get('y', 0.0),
                  M.get('z', 0.0)
                 )
        r.append( cmd )
    D.append(r)

plt.figure()

for i in range(3):
    minx = 1E6
    maxx = -1E6
    for r in D:
        for a in r:
            minx = min(minx, a[i])
            maxx = max(maxx, a[i])
            
    cmap = {}
    colors = []
    data = []
    d = max(abs(maxx), abs(minx))
    if d==0.0:
        d = 1.0
    
    for r in D:
        nr = []
        for a in r:
            frac = a[i]/d
            
            if frac in cmap:
                c = cmap[frac]
            else:
                cmap[frac] = len(colors)
                c = cmap[frac]
                af = abs(frac)
                if a[i] >= 0:
                    colors.append( (1,1-af,1-af) )
                else:
                    colors.append( (1-af,1-af,1) )
                    
                #print "%.2f %.2f %s"%(a[i], af, "%.1f %.1f %.1f"%colors[-1])
            nr.append(c)
        data.append(nr)
    ax = plt.subplot(3,1,i+1,axisbg='gray')
    if len(colors)==1:
        colors.append( (0,0,0) )
    heatmap_custom_color(data, colors, x_axis=R2, y_axis=R1, x_label=X_AXIS, y_label=Y_AXIS, label=['x','y','theta'][i], ax=ax)    
T = ', '.join(p.critics)


show()
