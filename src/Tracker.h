//
// Created by jieming on 04.02.20.
//

#ifndef RGBD_DETECT_TRACKER_H
#define RGBD_DETECT_TRACKER_H

#include "YoloRecognizer.h"
#include "KalmanFilter.h"
#include "Hungarian.h"

class Tracker {
    private:
        ros::NodeHandle n_;
        ros::Rate rate;

        unsigned int  previous_timestamp_;
        bool is_initialized;
        //acceleration noise components
        float noise_ax;
        float noise_ay;

public:
        std::vector<KalmanFilter>  kfV_;
        Hungarian hungarian_;
        Tracker(const ros::NodeHandle& n, ros::Rate&  r);
        ~Tracker()= default;

        void update();
        friend void trackThread(Tracker&);
};


#endif //RGBD_DETECT_TRACKER_H



//        std::mutex* mutex;
//        std::condition_variable* conditionVariable;
//        const YoloRecognizer recognizer;