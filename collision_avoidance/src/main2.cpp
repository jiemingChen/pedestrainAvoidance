//
// Created by jieming on 23.05.20.
//

#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include "collision_avoidance/header.h"
#include<time.h>
#include <deque>

void cameraCallback(const std_msgs::Float32MultiArray& msg);
void lidarCallback(const std_msgs::Float32MultiArray& msg);

struct Person{
    int id;
    float x;
    float y;
    float vx=0;
    float vy=0;
    std::deque<std::vector<float>> queue;

    void updateSpeed(float new_x, float new_y, float time){
        std::vector<float> temp  = {new_x, new_y, time};
        if(queue.size()>=15){
            queue.push_back(temp);
            float dt,xx,yy;
            vx = 0;
            vy = 0;
            for(auto i=0; i<15; i++){
                dt = (queue.at(i+1)[2] - queue.at(i)[2]) / CLOCKS_PER_SEC;
                xx = queue.at(i+1)[0] - queue.at(i)[0];
                yy = queue.at(i+1)[1] - queue.at(i)[1];
                vx += xx /dt;
                vy += yy /dt;
            }
            vx = vx / 15;
            vy = vy / 15;
            auto theta = atan2(vy, vx);
            vx = 1.0 * cos(theta);
            vy = 1.0 * sin(theta);
            queue.pop_front();
        }
        else{
            queue.push_back(temp);
        }
    }
};

visualization_msgs::MarkerArray array, lidar_array;
ros::Publisher pub;
ros::Subscriber sub, sub_lidar;
std::vector<Person> camera_detect_receive, lidar_detect_receive, detect_receive;

int last_cnt = 0;
#if 0
void cameraCallback(const std_msgs::Float32MultiArray& msg)
{
    array.markers.clear();
    int cnt=0;
    for(auto i=0; i<(msg.data.size()/5); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "/";
        marker.id = msg.data[i*5];
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg.data[i*5 + 1];
        marker.pose.position.y = msg.data[i*5 + 2];
        marker.pose.position.z = 0;
//        std::cout<< msg.data[i*5 + 1] << "  "<< msg.data[i*5 + 2]<< std::endl;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 0.5;
        if(marker.id%3==0){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else if(marker.id%3==1){
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        array.markers.push_back(marker);
        cnt++;
    }
    last_cnt = cnt;
}
void lidarCallback(const std_msgs::Float32MultiArray& msg)
{
    lidar_array.markers.clear();
    int cnt=0;
    for(auto i=0; i<(msg.data.size()/3); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "/";
        marker.id = msg.data[i*3]+100;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg.data[i*3 + 1];
        marker.pose.position.y = msg.data[i*3 + 2];
        marker.pose.position.z = 0;
//        std::cout<< msg.data[i*3 + 1] << "  "<< msg.data[i*3 + 2]<< std::endl;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;
        if(marker.id%3==0){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
        }
        else if(marker.id%3==1){
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.6;
        }
        lidar_array.markers.push_back(marker);
        cnt++;
    }
}
void pubFunc(){
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::MarkerArray array_del;
    array_del.markers.push_back(marker);
    pub.publish(array_del);

    pub.publish(array);
    pub.publish(lidar_array);

    array_del.markers.clear();
    array.markers.clear();
    lidar_array.markers.clear();
}
#endif

void cameraCallback(const std_msgs::Float32MultiArray& msg)
{
    Person person;
    std::vector<Person> new_receive;
    for(auto i=0; i<(msg.data.size()/5); i++) {
        person.id = int(msg.data[i*5]);
        person.x = msg.data[i*5 + 1];
        person.y = msg.data[i*5 + 2];
        bool new_idx = true;
        for(auto j=0; j<camera_detect_receive.size(); j++){
            if(person.id == camera_detect_receive[j].id){
                camera_detect_receive[j].updateSpeed(person.x, person.y, clock());
                camera_detect_receive[j].x = person.x;
                camera_detect_receive[j].y = person.y;

                new_idx  = false;
                new_receive.push_back(camera_detect_receive[j]);
                break;
            }
        }
        if(new_idx){
            person.vx = 0;
            person.vy = 0;
            new_receive.push_back(person);
        }
    }
    camera_detect_receive = new_receive;
}

void lidarCallback(const std_msgs::Float32MultiArray& msg){
    Person person;
    std::vector<Person> new_receive;
    for(auto i=0; i<(msg.data.size()/3); i++) {
        person.id = int(msg.data[i*3])+3000;
        person.x = msg.data[i*3 + 1];
        person.y = msg.data[i*3 + 2];
        bool new_idx = true;
        for(auto j=0; j<lidar_detect_receive.size(); j++){
            if(person.id == lidar_detect_receive[j].id){
                lidar_detect_receive[j].x = person.x;
                lidar_detect_receive[j].y = person.y;

                new_idx  = false;
                new_receive.push_back(lidar_detect_receive[j]);
                break;
            }
        }
        if(new_idx){
            new_receive.push_back(person);
        }
    }
    lidar_detect_receive = new_receive;
}

void pubFunc(){
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::MarkerArray array_del;
    array_del.markers.push_back(marker);
    pub.publish(array_del);

    for(auto i=0; i<detect_receive.size(); i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "/";
        marker.id = detect_receive[i].id;
//        std::cout<< marker.id<<std::endl;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = detect_receive[i].x;
        marker.pose.position.y = detect_receive[i].y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.3;
        marker.color.a = 0.5;
        if(marker.id%3==0){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else if(marker.id%3==1){
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        array.markers.push_back(marker);
    }
    pub.publish(array);
//    std::cout<<"---------------------------"<<std::endl;

    array_del.markers.clear();
    array.markers.clear();
}
void mergeMessage(){
    for(auto i=0; i<camera_detect_receive.size(); i++){
        const auto person_cam =  camera_detect_receive[i];
        for(auto j=0; j<lidar_detect_receive.size(); j++){
            const auto person_lidar =  lidar_detect_receive[j];
            float dist = pow(person_cam.x-person_lidar.x, 2) + pow(person_cam.y-person_lidar.y, 2);
            if(dist <=0.5*0.5){
                lidar_detect_receive.erase(lidar_detect_receive.begin()+j);
                break;
            }
        }
    }
    std::vector<Person> new_detect_receive;
    for(auto i=0; i<camera_detect_receive.size(); i++) {
        new_detect_receive.push_back(camera_detect_receive[i]);
       // std::cout<<camera_detect_receive[i].x<<"  "<<camera_detect_receive[i].y<<"  "<<camera_detect_receive[i].id<<"  "<<std::endl;
    }
    for(auto i=0; i<lidar_detect_receive.size(); i++) {
        new_detect_receive.push_back(lidar_detect_receive[i]);
    }
    detect_receive = new_detect_receive;
}

 int main(int argc, char **argv){

    ros::init(argc, argv, "subscrib");
    ros::NodeHandle n;
    ros::Rate  r(10);
    pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 10);
    sub_lidar = n.subscribe("lidarpart", 1000, lidarCallback);
    sub = n.subscribe("detectionPart", 1000, cameraCallback);

    while(ros::ok()){
        ros::spinOnce();
        mergeMessage();
        for(auto& d:camera_detect_receive){
            std::cout << d.vx <<"   " << d.vy << "  "<<d.x << "  "<<d.y << "  "<< d.id <<std::endl;
        }
        pubFunc();
        r.sleep();
    }

    return 0;
}