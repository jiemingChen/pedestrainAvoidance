//
// Created by jieming on 25.05.20.
//

#include "Merge.h"
std::pair<std::vector<std::vector<float>> ,std::vector<std::vector<float>>> Merge::mergeMessage(){
    for(auto i=0; i<camera_detect_receive_.size(); i++){
        const auto person_cam =  camera_detect_receive_[i];
        for(auto j=0; j<lidar_detect_receive_.size(); j++){
            const auto person_lidar =  lidar_detect_receive_[j];
            float dist = pow(person_cam.x-person_lidar.x, 2) + pow(person_cam.y-person_lidar.y, 2);
            if(dist <=0.5*0.5){
                lidar_detect_receive_.erase(lidar_detect_receive_.begin()+j);
                break;
            }
        }
    }
    std::vector<Person> new_detect_receive;
    for(auto i=0; i<camera_detect_receive_.size(); i++) {
        new_detect_receive.push_back(camera_detect_receive_[i]);
        //std::cout<<camera_detect_receive_[i].x<<"  "<<camera_detect_receive_[i].y<<"  "<<camera_detect_receive_[i].id<<"  "<<std::endl;
    }
    for(auto i=0; i<lidar_detect_receive_.size(); i++) {
        new_detect_receive.push_back(lidar_detect_receive_[i]);
    }
    detect_receive_ = new_detect_receive;
    std::vector<std::vector<float>> pos_vec, speed_vec;
    std::vector<float> temp_vec;
    for(auto& detect: detect_receive_){
        temp_vec.push_back(detect.x);
        temp_vec.push_back(detect.y);
        temp_vec.push_back(detect.id);
        pos_vec.push_back(temp_vec);
        temp_vec.clear();

        temp_vec.push_back(detect.vx);
        temp_vec.push_back(detect.vy);
        speed_vec.push_back(temp_vec);
        temp_vec.clear();
    }
    return std::make_pair(pos_vec, speed_vec);
}

void Merge::cameraCallback(const std_msgs::Float32MultiArray& msg)
{
    Person person;
    std::vector<Person> new_receive;
    for(auto i=0; i<(msg.data.size()/5); i++) {
        person.id = int(msg.data[i*5]);
        person.x = msg.data[i*5 + 1];
        person.y = msg.data[i*5 + 2];
        bool new_idx = true;
        for(auto j=0; j<camera_detect_receive_.size(); j++){
            if(person.id == camera_detect_receive_[j].id){
                camera_detect_receive_[j].updateSpeed(person.x, person.y, clock());
                camera_detect_receive_[j].x = person.x;
                camera_detect_receive_[j].y = person.y;
//                camera_detect_receive_[j].vx = 0;
//                camera_detect_receive_[j].vy = 0;

                new_idx  = false;
                new_receive.push_back(camera_detect_receive_[j]);
                break;
            }
        }
        if(new_idx){
            person.vx = 0;
            person.vy = 0;
            new_receive.push_back(person);
        }
    }
    camera_detect_receive_ = new_receive;
}

void Merge::lidarCallback(const std_msgs::Float32MultiArray& msg){
    Person person;
    std::vector<Person> new_receive;
    for(auto i=0; i<(msg.data.size()/3); i++) {
        person.id = int(msg.data[i*3])+3000;
        person.x = msg.data[i*3 + 1];
        person.y = msg.data[i*3 + 2];
        bool new_idx = true;
        for(auto j=0; j<lidar_detect_receive_.size(); j++){
            if(person.id == lidar_detect_receive_[j].id){
                lidar_detect_receive_[j].x = person.x;
                lidar_detect_receive_[j].y = person.y;

                new_idx  = false;
                new_receive.push_back(lidar_detect_receive_[j]);
                break;
            }
        }
        if(new_idx){
            new_receive.push_back(person);
        }
    }
    lidar_detect_receive_ = new_receive;
}
