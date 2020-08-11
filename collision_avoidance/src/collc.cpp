//
// Created by jieming on 28.04.20.
//
//
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include "collision_avoidance/header.h"
#include "Config.h"
#include <chrono>
#include <thread>
#include "solver.h"
#include "PID.h"
#include "pubMsg.h"
#include "ipopttest.h"
#include <fstream>
using namespace std;


vector<float> getCurrentPos(tf::TransformListener& listener){
    tf::StampedTransform transform;
    vector<float> rst;

    try{
        listener.lookupTransform("odom", "front_steering", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s wocao",ex.what());
        ros::Duration(1.0).sleep();
    }
    rst.push_back(transform.getOrigin().x());
    rst.push_back(transform.getOrigin().y());

    try{
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s  nimade",ex.what());
        ros::Duration(1.0).sleep();
    }
    auto quaternion  = transform.getRotation();
    float angle = atan2(2*(quaternion.w()*quaternion.z()+quaternion.x()*quaternion.y()), 1-2*(pow(quaternion.y(),2)+pow(quaternion.z(),2)));
    if(angle>M_PI){
        angle = angle - 2*M_PI;
    }
    rst.push_back(angle);
    return rst;
}

std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> getPeopleInfo(ros::NodeHandle& n, std::vector<std::string> names){
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient client_v1 = n.serviceClient<actor_plugin::GetVel>("/actor1/GetActorVelocity");
    ros::ServiceClient client_v2 = n.serviceClient<actor_plugin::GetVel>("/actor2/GetActorVelocity");

    gazebo_msgs::GetModelState srv;
    actor_plugin::GetVel srv2;

    ros::Rate r(40);
    std::vector<std::vector<float>> poses;
    std::vector<std::vector<float>> speed;

    int i =0;

    for(const auto& name:names){
        srv.request.model_name=name;
        srv2.request.set_flag = false;
        srv2.response.x = 0;
        srv2.response.y= 0;

        if(client.call(srv)){
            auto pos = srv.response.pose;
//            ROS_INFO("%s is here %f, %f", name.c_str(), pos.position.x, pos.position.y);
            std::vector<float> temp(3,0);
            temp[0] = static_cast<float>(pos.position.x);
            temp[1] = static_cast<float>(pos.position.y);
            temp[2] = i;
            poses.push_back(temp);
	    cout << temp[0] <<" " << temp[1] << " " << temp[2] <<"\n";
        }

        if(name=="actor1"){

            client_v1.call(srv2);
            std::vector<float> temp(2,0);
            temp[0] = srv2.response.x;
            temp[1] = srv2.response.y;
            temp[1] = 0;
            speed.push_back(temp);
//            ROS_INFO("%s is here %f, %f", name.c_str(), temp[0], temp[1]);
        }
        else if(name=="actor2"){
            client_v2.call(srv2);
            std::vector<float> temp(2,0);
            temp[0] = srv2.response.x;
            temp[1] = srv2.response.y;
            temp[0] = 0;
            speed.push_back(temp);
//            ROS_INFO("%s is here %f, %f", name.c_str(), temp[0], temp[1]);
        }
        else{

            std::vector<float> temp(2,0);
            speed.push_back(temp);
        }

        i++;
        r.sleep();
    }
    auto rst = std::make_pair(poses, speed);
    return rst;
}

#if 1
// mpc
int main(int argc,char **argv){

    ros::init(argc,argv,"data_collect");
    ros::NodeHandle n;
    tf::TransformListener listener;
    ros::Rate r(10);

//    std::vector<std::string> names = {"xiaomei","dahei",  "jams", "actor1", "actor2"};
    std::vector<std::string> names = {"xiaomei","dahei","jams", "xiaobai"};

    std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> obstacles;


    std::vector<double> target_point = {25, 0, 0};
    int N_sim = 100000;
    ofstream myfile;
    myfile.open ("/home/jieming/sim_data.txt");


    for(auto t=0; t<N_sim; t++){

        obstacles = getPeopleInfo(n, names);
        auto current_state = getCurrentPos(listener);
        ros::spinOnce();
        r.sleep();
        myfile << current_state[0] << " " << current_state[1] << " " << current_state[2] << "\n";
	cout << "here" <<endl;
    }
    myfile.close();
    return 0;
}
#endif
