#include <dwa_local_planner/dwa_planner_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "standalone");
    
    double x = 2, y = 0;
    bool loop = false;
    ROS_INFO("%d", argc);
    if(argc>1){
        if(argc>2){
            x = atof(argv[1]);
            y = atof(argv[2]);
        }
        if(strcmp(argv[argc-1], "-l")==0){
            loop = true;
        }
    }

    tf::TransformListener tf;
    
    dwa_local_planner::DWAPlannerROS planner;
    costmap_2d::Costmap2DROS costmap("local", tf);
    planner.initialize("planner", &tf, &costmap);   

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
    planner.setPlan(plan);

    for(double i=0.0;i<=2;i+=.1){
    for(double v=0.0;v<=.5;v+=.1)
        printf("%f ", planner.scoreTrajectory(i,0,0,0,0,0,v, 0,0));
        printf("\n");   
    }

    return 0;
}
