#include <dwa_local_planner/dwa_planner_ros.h>
#include <std_srvs/Empty.h>
#include<trajectory_scorer/ScoreTrajectory.h>

class Scorer {
    public:
    Scorer() : planner(NULL), costmap(NULL) {
        ros::NodeHandle nh("~");
        a = nh.advertiseService("reset", &Scorer::callback, this);
        b = nh.advertiseService("score", &Scorer::score, this);
        reset();
    }
    
    
    bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        reset();
        return true;
    }
    
    bool score(trajectory_scorer::ScoreTrajectory::Request& r, 
               trajectory_scorer::ScoreTrajectory::Response& request)
    {
        
        double s = planner->scoreTrajectory(r.x,0,0,0,0,0,r.cvx, 0,0);
        ROS_INFO("Score: %f", s);
        request.score = s;
        return true;
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
    ros::ServiceServer a,b;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_scorer");
    
    /*double x = 2, y = 0;
    if(argc>1){
        if(argc>2){
            x = atof(argv[1]);
            y = atof(argv[2]);
        }
    }
    
    std::vector<geometry_msgs::PoseStamped> plan;
    int N = 100;
    for(int i=0;i<N;i++){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose.orientation.w = 1.0;
        pose.pose.position.x = i * x / N;
        pose.pose.position.y = i * y / N;
        plan.push_back(pose);
    }
    //planner.setPlan(plan);*/
    Scorer s;
    ros::spin();


    return 0;
}
