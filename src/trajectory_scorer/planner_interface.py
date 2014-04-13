import rospy
from trajectory_scorer.srv import ScoreTrajectory
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from numpy import arange
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
S_NAME = '/trajectory_scorer/score'

R_NAME = '/trajectory_scorer/reset'

def param(n):
    return rospy.get_param('/trajectory_scorer/planner/%s'%n)
    
def get_range(ax):
    n = param('v%s_samples'%ax)
    if ax=='th':
        m1 = param('max_rot_vel')
        m0 = -m1
    else:
        m0 = param('min_vel_%s'%ax)
        m1 = param('max_vel_%s'%ax)
    d = (m1-m0)/n
    return arange(m0,m1+.001,d)

class PlannerInterface:
    def __init__(self):
        rospy.wait_for_service(S_NAME)
        print 'Ready'
        self.score_srv = rospy.ServiceProxy(S_NAME, ScoreTrajectory)
        self.pub = rospy.Publisher('/trajectory_scorer/plan', Path, latch=True)
        self.critics = rospy.get_param('/trajectory_scorer/planner/critics')
        rospy.wait_for_service(R_NAME)
        self.reset = rospy.ServiceProxy(R_NAME, Empty)
        
    def set_critics(self, critics):
        if critics != self.critics:
            rospy.set_param('/trajectory_scorer/planner/critics', critics)
            self.critics = critics
            print "Resetting"
            self.reset()
            rospy.sleep(1)
        
    def set_goal(self, x, y, z):
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
        self.pub.publish(path)
        rospy.sleep(1.0)
        
    def score(self, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, cvx=0.0, cvy=0.0, cvz=0.0):
        resp = self.score_srv(x,y,z,vx,vy,vz,cvx,cvy,cvz)
        return resp.score
        
    def plan(self, x=0.0, y=0.0, z=0.0):
        bests = 1E11
        best = None
        for cvx in get_range('x'):
            for cvy in get_range('y'):
                for cvz in get_range('th'):
                    s = self.score(x,y,z,cvx=cvx,cvy=cvy,cvz=cvz)
                    if s >= 0 and s < bests:
                        best = [cvx,cvy,cvz]
                        bests=s
        return best
    
