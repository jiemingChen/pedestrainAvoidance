//
// Created by jieming on 26.01.20.
//

#ifndef RGBD_DETECT_YOLORECOGNIZER_H
#define RGBD_DETECT_YOLORECOGNIZER_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <getopt.h>
#include <map>
#include <string>
#include <thread>
#include <stdlib.h>
#include <string>
#include <pthread.h>

#include <message_filters/subscriber.h>
#include <ros/publisher.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

//#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <exception>

#include <Eigen/Dense>

#include "darknet.h"
#include "LaserRecognizer.h"

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace std;



class YoloRecognizer {
private:
    std::ofstream outFile;

    ros::NodeHandle n_;
    ros::Rate rate;

    message_filters::Subscriber<sensor_msgs::Image>* rgbSub_;
    message_filters::Subscriber<sensor_msgs::Image>* depthSub_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odomSub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy>* sync_;

    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_depth;

    ros::Publisher pub_;
    cv::Mat detectionImage;
    cv::Mat detectionDepthImage;


    std::vector<string> labelNames;
    std::string labelfile;
    char* cfgfile;
    char* weightfile;
    network* net;
    std::vector<std::vector<float>> boxes;
    std::vector<std::pair<std::string, float>> yoloRst;
    std::map<std::string, double> options;

    sensor_msgs::LaserScanConstPtr scan_ptr;
    LaserRecognizer laserRecognizer;

    nav_msgs::OdometryConstPtr odomPtr;
    thread* th1;
    thread* scanth2;
    thread* mainth0;


public:

    YoloRecognizer(ros::NodeHandle&, ros::Rate&);

    ~YoloRecognizer();
    void combineCallback(const ImageConstPtr& rgb, const ImageConstPtr& depth, const LaserScanConstPtr& scan, const OdometryConstPtr& odometry);

    friend void mainThread(YoloRecognizer& );
    friend void recognizationThread(YoloRecognizer& );
    friend void laserRecogThread(YoloRecognizer& );

    void test_yolo(char *cfgfile, char *weightfile,  std::string labelfile, char *filename, float thresh);
    void deProjection(const float& depth, const cv::Rect roi);

    friend class Tracker;
};


#endif //RGBD_DETECT_YOLORECOGNIZER_H
