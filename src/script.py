#!/usr/bin/python

import rospy
from trajectory_scorer.srv import ScoreTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from numpy import arange
from tf.transformations import *
S_NAME = '/trajectory_scorer/score'
R_NAME = '/trajectory_scorer/reset'


rospy.init_node('score_script')

if True:
    rospy.set_param('/trajectory_scorer/planner/critics', [ 'GoalAlign'])
    rospy.wait_for_service(R_NAME)
    reset = rospy.ServiceProxy(R_NAME, Empty)
    reset()
    rospy.sleep(2)


rospy.wait_for_service(S_NAME)
print 'Ready'
score = rospy.ServiceProxy(S_NAME, ScoreTrajectory)
pub = rospy.Publisher('/trajectory_scorer/plan', Path, latch=True)

#publish global plan
x = -3.0
y = 0.0
z = 0 * 3.14 / 180

q = quaternion_from_euler(0,0,z)

path = Path()
path.header.frame_id = '/map'
N = 100
for i in range(N):
    p = PoseStamped()
    p.header.frame_id = '/map'
    p.pose.position.x = x * i / N
    p.pose.position.y = y * i / N
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    path.poses.append(p)
pub.publish(path)
rospy.sleep(1.0)

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
        s = score(M.get('x', 0.00356),
                  M.get('y', 0),
                  M.get('z', -.44), 0, 0, 0, 
                  M.get('cvx', 0.0), 
                  M.get('cvy', 0), 
                  M.get('cvz', 0)
                 ).score
        r.append( s )
        mx = max(mx, s)
        print "%.3f %.3f %.3f"%(v2, v1, s)
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


T = ', '.join(rospy.get_param('/trajectory_scorer/planner/critics'))
ax.set_title(T)
plt.xlabel(X_AXIS)
plt.ylabel(Y_AXIS)
plt.show()

