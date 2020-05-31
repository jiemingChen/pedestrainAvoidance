//
// Created by jieming on 25.05.20.
//

#ifndef COLLISION_AVOIDANCE_MERGE_H
#define COLLISION_AVOIDANCE_MERGE_H

#include "collision_avoidance//header.h"
struct Person{
    int id;
    float x;
    float y;
    float vx=0;
    float vy=0;
    std::deque<std::vector<float>> queue;

    void updateSpeed(float new_x, float new_y, float time){
        std::vector<float> temp  = {new_x, new_y, time};
        if(queue.size()>=10){
            queue.push_back(temp);
            float dt,xx,yy;
            vx = 0;
            vy = 0;
            for(auto i=0; i<10; i++){
                dt = (queue.at(i+1)[2] - queue.at(i)[2]) / CLOCKS_PER_SEC;
                xx = queue.at(i+1)[0] - queue.at(i)[0];
                yy = queue.at(i+1)[1] - queue.at(i)[1];
                vx += xx /dt;
                vy += yy /dt;
            }
            vx = vx / 10;
            vy = vy / 10;

            if(vx>0.3 || vy>0.3){
                auto theta = atan2(vy, vx);
                vx = 1.0 * cos(theta);
                vy = 1.0 * sin(theta);
            }

            queue.pop_front();
        }
        else{
            queue.push_back(temp);
        }
    }
};

class Merge {
private:
    std::vector<Person> camera_detect_receive_;
    std::vector<Person> lidar_detect_receive_;
    std::vector<Person> detect_receive_;

public:
    std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> mergeMessage();
    void cameraCallback(const std_msgs::Float32MultiArray& msg);
    void lidarCallback(const std_msgs::Float32MultiArray& msg);
};


#endif //COLLISION_AVOIDANCE_MERGE_H
