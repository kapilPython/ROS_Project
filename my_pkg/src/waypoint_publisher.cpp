#include "GoalDatabase.h"
#include <vector>
#include <map>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <string.h>

int main(int argc,char** argv){
    ros::init(argc, argv, "waypointPublisher");
    ros::NodeHandle nh;
    /**
        Object Of GoalDatabase class
    */
    GoalDatabase gdb;
    std::string path;
    std::cout<<"Please enter the path to your yaml file"<<std::endl;
    std::cout<<">";
    std::cin>>path;
    // please change the path as per your YAML file
    // example path
    //bool checkParse = gdb.createDatabase("/home/kapil/neo_kapil_ws/src/my_pkg/data/goalData.yaml");
    bool checkParse = gdb.createDatabase(path);
    while(!checkParse){ 
        ROS_INFO("Kill the node since the data is not correct");
    }
    gdb.print();
    //saving the data to local map
    std::map<int,std::vector<float> > DB = gdb.getDatabase();
    std::map<int,std::vector<float> >::iterator it;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("/waypoint", 1000);
    geometry_msgs::PoseArray poses = geometry_msgs::PoseArray();
    // frame given as the map
    poses.header.frame_id = "map";

    //Assigning DB data to Pose array 
    for(it = DB.begin(); it!=DB.end(); ++it){
        geometry_msgs::Pose tempPos;
        tempPos.position.x = (it->second)[0];
        tempPos.position.y = (it->second)[1];
        tempPos.orientation.w = (it->second)[2];
        poses.poses.push_back(tempPos);
    }

    ros::Rate r(10);
    // Publishing the data to waypoint topic
    while(ros::ok()){
        pub.publish(poses);
        r.sleep();
    }
    ros::spin();
    return 0;
}
