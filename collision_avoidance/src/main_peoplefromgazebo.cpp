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

std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> findNearObstacles(const std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>>& obstacles, const vector<float>& current_state){
    float distance;
    vector<vector<float>> near_obstacles, near_obstacles_v;

    for(auto i=0; i<obstacles.first.size(); i++){
        distance = pow(obstacles.first[i][0]-current_state[0], 2) + pow(obstacles.first[i][1]-current_state[1],2);
        if(distance <= pow(5,2)){
            near_obstacles.push_back(obstacles.first[i]);
            near_obstacles_v.push_back(obstacles.second[i]);
        }
    }

    auto rst = std::make_pair(near_obstacles, near_obstacles_v);
    return rst;
}
#if 1
// mpc
int main(int argc,char **argv){

    ros::init(argc,argv,"collision_avoidance_node");
    ros::NodeHandle n;
    ros::Publisher vis_pub =  n.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
    ros::Publisher marker_array_pub =  n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 10);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",4);
    tf::TransformListener listener;
    ros::Rate r(10);

    std::vector<std::string> names = {"xiaomei","dahei",  "jams", "actor1", "actor2"};
    std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> obstacles;

    Config::setParameterFile("/home/jieming/car_ws/src/collision_avoidance/config/default.yaml");
    std::vector<double> target_point = {10, -7, 0};
    int N_sim = 1000;

    std::pair<vector<double>, vector<float>> job_point;
    vector<float> current_state;
    job_point.first = vector<double>(3,0);
    job_point.second = vector<float>(2,0);

    ros::spinOnce();
    r.sleep();
    pubTerminal(vis_pub, target_point);

    Solver solver;
    for(auto t=0; t<N_sim; t++){

        obstacles = getPeopleInfo(n, names);
        pubObstacles(marker_array_pub, obstacles.first);

        current_state = getCurrentPos(listener);
        double angle = atan2(target_point[1]-current_state[1], target_point[0]-current_state[0]);
        if(abs(angle-current_state[2])>1.45){
            job_point.first[2] = current_state[2];
        }
        else{
            job_point.first[2] = angle;
        }
        target_point[2] = atan2(target_point[1]-current_state[1], target_point[0]-current_state[0]);

        if(pow(current_state[0]-target_point[0],2)+pow(current_state[1]-target_point[1],2) <= pow(0.3,2)){
            cout<<"arrive"<<endl;
            vector<float> temp={0,0};
            pubCommand(cmd_pub, temp);
            break;
        }

        auto near_obstacles = findNearObstacles(obstacles, current_state);
        auto solver_solution = solver.solve2(job_point, target_point, current_state, near_obstacles);

        pubCommand(cmd_pub, solver_solution);
        pubTraj2(marker_array_pub, solver_solution);

        ros::spinOnce();
        r.sleep();
        job_point.second[0] = 0.9;

    }

    return 0;
}
#endif
