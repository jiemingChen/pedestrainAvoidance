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
#include "CIAO.h"
#include "nav_msgs/Odometry.h"
vector<float> global_pos;
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

void getCurrentPos(nav_msgs::Odometry data){
    vector<float> rst(3,0);
    rst[0] = (data.pose.pose.position.x);
    rst[1] =(data.pose.pose.position.y);

    float w = data.pose.pose.orientation.w;
    float z = data.pose.pose.orientation.z;
    float x = data.pose.pose.orientation.x;
    float y = data.pose.pose.orientation.y;
    float angle = atan2(2*(w*z+x*y), 1-2*(pow(y, 2)+pow(z, 2)));
    if(angle>M_PI){
        angle = angle - 2*M_PI;
    }
    rst[2] = angle;
    global_pos = rst;
    cout<<"111"<<endl;
}
#if 1
// mpc
int main(int argc,char **argv){

    ros::init(argc,argv,"collision_avoidance_node");
    ros::NodeHandle n;
    ros::Publisher vis_pub =  n.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
    ros::Publisher marker_array_pub =  n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 10);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",4);
    ros::Subscriber pos_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, getCurrentPos);
    tf::TransformListener listener;
    ros::Rate r(10);

    std::vector<std::string> names = {"xiaomei","dahei",  "jams", "actor1", "actor2", "xiaobai"};
//    std::vector<std::string> names = {"actor1", "actor2", "xiaomei", "dahei" , "jams"};
//    std::vector<std::string> names = { "xiaomei", "dahei" , "jams"};

    std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> obstacles;

    Config::setParameterFile("/home/jieming/car_ws/src/collision_avoidance/config/default.yaml");

    std::vector<double> target_point = {11, -4, 0};

    int N_sim = 10000;

    std::pair<vector<double>, vector<float>> job_point;
    vector<float> current_state;
    job_point.first = vector<double>(3,0);
    job_point.second = vector<float>(2,0);

    ros::spinOnce();
    r.sleep();
    pubTerminal(vis_pub, target_point);


    CIAO solver;

    vector<vector<float>> initial_guess(10);
    ros::spinOnce();
    r.sleep();
    current_state = global_pos;

    for(auto i=0; i<10; i++){
        initial_guess[i].push_back(current_state[0]);
        initial_guess[i].push_back(current_state[1]);
    }
    for(auto t=0; t<N_sim; t++){

        obstacles = getPeopleInfo(n, names);
        pubObstacles(marker_array_pub, obstacles.first);

        current_state = global_pos;
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
        auto solver_solution = solver.ciaoIteration(initial_guess, job_point, target_point, current_state, near_obstacles);

//        pubCommand(cmd_pub, solver_solution);
        pubTraj2(marker_array_pub, solver_solution);

        ros::spinOnce();
        r.sleep();
        job_point.second[0] = 0.9;
        for(auto i=0; i<10; i++){
            initial_guess[i][0] = solver_solution[2+i*3];
            initial_guess[i][1] = solver_solution[3+i*3];
        }
    }

    return 0;
}
#endif

#if 0
// tube mpc
int main(int argc,char **argv){

    ros::init(argc,argv,"collision_avoidance_node");
    ros::NodeHandle n;
    ros::Publisher vis_pub =  n.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
    ros::Publisher marker_array_pub =  n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 2);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",4);
    tf::TransformListener listener;
    ros::Rate r(10);

//    std::vector<std::string> names = {"xiaomei","dahei",  "jams", "zhongbai", "actor1", "actor2"};
    std::vector<std::string> names = {"actor1", "actor2", "xiaomei", "dahei"};
    std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> obstacles;

    Config::setParameterFile("/home/jieming/car_ws/src/collision_avoidance/config/default.yaml");

    std::vector<float> target_point = {Config::get<float>("xTarget"), Config::get<float>("yTarget"), 0};

    int N_sim = Config::get<int>("N_sim");

    std::pair<vector<float>, vector<float>> job_point;
    vector<float> current_state;
    job_point.first = vector<float>(3,0);
    job_point.second.push_back(0.3);
    job_point.second.push_back(0);

    ros::spinOnce();
    r.sleep();
    pubTerminal(vis_pub, target_point);
    PID pid;

    Solver solver;
    for(auto t=0; t<N_sim; t++){
        obstacles = getPeopleInfo(n, names);
        pubObstacles(marker_array_pub, obstacles.first);

        current_state = getCurrentPos(listener);
        target_point[2] = atan2(target_point[1]-current_state[1], target_point[0]-current_state[0]);

        float angle = atan2(target_point[1]-current_state[1], target_point[0]-current_state[0]);
        if(abs(angle-current_state[2])>1.3){
            job_point.first[2] = current_state[2];
        }
        else{
            job_point.first[2] = angle;
        }
//        job_point.first[2] = angle;

        if(pow(current_state[0]-target_point[0],2)+pow(current_state[1]-target_point[1],2) <= pow(0.3,2)){
            cout<<"arrive"<<endl;
            vector<float> temp={0,0};
            pubCommand(cmd_pub, temp);
            break;
        }

        auto near_obstacles = findNearObstacles(obstacles, current_state);

        /// \brief mpc give reference
        auto solver_solution = solver.solve2(job_point, target_point, current_state, near_obstacles);
        auto next_state = solver.nominalSystem(solver_solution, current_state);

        /// \brief pid  cope with uncertainty, disturbance
        float vertical_error = 1000;
        pid.initialize(next_state, current_state);
        while(1){
            auto error = pow(pow(next_state[0]-current_state[0],2)+pow(next_state[1]-current_state[1],2),2);
            if( error<=0.15*0.15 && vertical_error<=0.1){
//                pubCommand(cmd_pub, vector<float>(2,0));
                break;
            }
            vertical_error = pid.control(next_state, current_state, solver_solution);

            pubCommand(cmd_pub, solver_solution);
            pubTraj2(marker_array_pub, solver_solution);
            ros::spinOnce();
            r.sleep();
            current_state = getCurrentPos(listener);
        }


//        pubTraj2(marker_array_pub, solver_solution);
        ros::spinOnce();
        r.sleep();

    }

    return 0;
}
#endif



#if 0
int main(int argc,char **argv){

    ros::init(argc,argv,"collision_avoidance_node");
    ros::NodeHandle n;
    ros::Publisher vis_pub =  n.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
    ros::Publisher marker_array_pub =  n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 2);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",4);
    tf::TransformListener listener;
    ros::Rate r(20);
    ros::spinOnce();
    r.sleep();

    Config::setParameterFile("/home/jieming/car_ws/src/collision_avoidance/config/default.yaml");
    std::vector<double> target_point = {Config::get<double>("xTarget"), Config::get<double>("yTarget"), M_PI};
    PID pid;
    vector<float> solver_solution={1,0};
    float steer_error=1000;
    auto current_state = getCurrentPos(listener);
    ros::spinOnce();
    r.sleep();
    pid.initialize(target_point, current_state);
    while(1){
         current_state = getCurrentPos(listener);
         ros::spinOnce();
        r.sleep();
        float rst= pow(target_point[0]-current_state[0],2) + pow(target_point[1]-current_state[1],2);
        if(rst<=0.2*0.2 && steer_error<0.2){
            break;
        }
         steer_error = pid.control(target_point, current_state, solver_solution);

        pubCommand(cmd_pub, solver_solution);
        pubTraj2(marker_array_pub, solver_solution);
        ros::spinOnce();
        r.sleep();
    }
    cout<<"arrive"<<endl;
    vector<float> temp={0,0};
    pubCommand(cmd_pub, temp);
    ros::spinOnce();
    r.sleep();
    return 0;
}
#endif
////       std::this_thread::sleep_for(std::chrono::seconds(5));
