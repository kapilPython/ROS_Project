#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <signal.h>
#include "GoalDatabase.h"

// movebase client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// callback for sendgoal
void done_cb(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResult::ConstPtr& result)
             {
                //ROS_INFO("Finished in state [%s]",state.toString.c_str());
                //ROS_INFO("Answer: %i", result->sequence.back());
                if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("The base reached the Goal ");
                }
                else{
                    ROS_INFO("The base failed for some reason to reach Goal");
                }
                if(DEBUG == 1){
                    ros::shutdown();
                }
             }

// call back for send goal            
void active_cb(){
    ROS_INFO("Goal just got active");
}

// call back for send goal
void feedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback){
    //ROS_INFO("Got feedback of length %lu",feedback->sequence.size());
    ROS_INFO("I'm feedback");
}

// overriding siginthandler to allow shutdown of process
void mySigintHandler(int sig){
    ros::shutdown();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MultiGoalDriver");
    ros::NodeHandle n;
    signal(SIGINT,mySigintHandler);
    
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
    std::map<int,std::vector<float> > DB = gdb.getDatabase();
    std::map<int,std::vector<float> >::iterator it;

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    
    if(DEBUG == 0){
        ros::Time startTime = ros::Time::now();
        // for sending the goals one by one with unique timestamp to the move base server
        for(it = DB.begin();it != DB.end();++it){
            ROS_INFO("Going for goal %d",it->first);
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = (it->second)[0];
            goal.target_pose.pose.position.y = (it->second)[1];
            goal.target_pose.pose.orientation.w = (it->second)[2];
            ac.sendGoal(goal,&done_cb,&active_cb);
            printf(ac.getState().toString().c_str());
            printf(" current status of goal \n");
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("The base reached the Goal %d",it->first);
            }
            else{
                ROS_INFO("The base failed for some reason to reach Goal %d",it->first);
            }
        }
        ros::Time endTime = ros::Time::now();
        ros::Duration diff = endTime - startTime;
        std::cout<< "Total time taken to complete given goals is " << diff.toSec() << " seconds" <<std::endl;
    }
    if(DEBUG == 1){
        ROS_INFO("Going for goal %d",1);
        goal.target_pose.pose.position.x = 0.6;
        goal.target_pose.pose.position.y = 0.05;
        goal.target_pose.pose.orientation.w = 1.0;
        
        ac.sendGoal(goal, &done_cb, &active_cb);
        //ac.waitForResult(ros::Duration(5.0));
        //ROS_INFO(ac.getState().toString().c_str());
        //printf(ac.getState().toString().c_str());
        
        /*if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("The base reached the Goal %d",1);
        }
        else{
            ROS_INFO("The base failed for some reason to reach Goal %d",1);
        }*/
        ros::spin();
    }
    
    return 0;
}
