//
// Created by jieming on 07.02.20.
//

#ifndef RGBD_DETECT_SORT_H
#define RGBD_DETECT_SORT_H
#include <algorithm>
#include "YoloRecognizer.h"
#include "KalmanFilter.h"
#include "Hungarian.h"
using std::vector;

class SORT {
private:
    int max_age; //for delet
    int min_hits; //for append
    vector<KalmanBoxTraker>trackers;
    int frame_count;//?

    std::vector<Eigen::Vector4d> measured;

    unsigned int  previous_timestamp_;
    bool is_initialized;
    ros::NodeHandle n_;
    ros::Rate rate;
public:
    SORT(ros::NodeHandle& n, ros::Rate& r);
    virtual ~SORT();
    void update();
    std::tuple< vector<pair<int, int>>,vector<int>,vector<int> > associate_detections_to_trackers(const vector<Eigen::Vector4d>&pred, const vector<Eigen::Vector4d>&meas,float);
    void initialize();
    Eigen::Matrix<double,7,1> convertToStateVector(const Eigen::Vector4d&);
    float iou(const Eigen::Vector4d & bbp, const Eigen::Vector4d& bbgt);
    friend void trackThread(SORT&);
};


#endif //RGBD_DETECT_SORT_H
