#include <dwa_local_planner/dwa_planner_ros.h>
#include <std_srvs/Empty.h>
#include<trajectory_scorer/ScoreTrajectory.h>
#include<trajectory_scorer/MakeLocalPlan.h>
#include<nav_msgs/Path.h>

class Scorer {
    public:
    Scorer() : planner(NULL), costmap(NULL) {
        ros::NodeHandle nh("~");
        a = nh.advertiseService("reset", &Scorer::callback, this);
        b = nh.advertiseService("score", &Scorer::score, this);
        c = nh.advertiseService("plan", &Scorer::plan, this);
        sub = nh.subscribe<nav_msgs::Path>("plan", 1, boost::bind(&Scorer::get_plan, this, _1)); 

        reset();
    }
    
    
    bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        reset();
        return true;
    }
    
    bool plan(trajectory_scorer::MakeLocalPlan::Request& req,
              trajectory_scorer::MakeLocalPlan::Response& resp){
    
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose.position.x = req.x;
        pose.pose.position.y = req.y;
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,req.theta);
        
        tf::Stamped<tf::Pose> cpose;
        tf::poseStampedMsgToTF(pose, cpose);
        
        geometry_msgs::Twist cmd;
        planner->computeVelocityCommands(cpose, cmd);
        
        resp.x = cmd.linear.x;
        resp.y = cmd.linear.y;
        resp.theta = cmd.angular.z;
        
        return true;
        
    }
    
    bool score(trajectory_scorer::ScoreTrajectory::Request& r, 
               trajectory_scorer::ScoreTrajectory::Response& request)
    {
        
        double s = planner->scoreTrajectory(r.x,r.y,r.theta,r.vx,r.vy,r.vtheta,r.cvx,r.cvy,r.cvtheta);
        //ROS_INFO("Score: %f", s);
        request.score = s;
        return true;
    }
    
    void get_plan(const nav_msgs::Path::ConstPtr& path)
    {
        ROS_INFO("GOT PLAN %d", (int)path->poses.size());
        planner->setPlan(path->poses);
    }
    
    private:
        void reset(){ 
            if(planner) delete planner;
            if(costmap) delete costmap;
            
            planner = new dwa_local_planner::DWAPlannerROS();
            costmap = new costmap_2d::Costmap2DROS("local", tf);
            planner->initialize("planner", &tf, costmap);   
        }
        tf::TransformListener tf;
        
        
    dwa_local_planner::DWAPlannerROS* planner;
    costmap_2d::Costmap2DROS* costmap;
    ros::ServiceServer a,b,c;
    ros::Subscriber sub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_scorer");
    
    Scorer s;
    ros::spin();


    return 0;
}
