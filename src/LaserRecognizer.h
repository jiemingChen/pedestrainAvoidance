//
// Created by jieming on 01.02.20.
//

#ifndef RGBD_DETECT_LASERRECOGNIZER_H
#define RGBD_DETECT_LASERRECOGNIZER_H

#include <algorithm>

#include <sensor_msgs/LaserScan.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/ml/ml.hpp"

#include <Eigen/Dense>
#include <fstream>

using namespace cv::ml;


struct SamplePoint{
    float x;
    float y;
    int index;
    SamplePoint(float xf, float yf, int i){
        x=xf;
        y=yf;
        index=i;
    }
};

class LaserRecognizer {
    private:
        typedef std::function<void(void)> Functional;
//    typedef void(LaserRecognizer::*pfunc)(void);
        Functional pfeature[5];

        std::vector<std::pair<float, float>> filteredScan;
        std::vector<SamplePoint> samples;
        std::vector<std::vector<SamplePoint>> segments;

        std::vector<float> feature1Vector;
        std::vector<float> feature2Vector;
        std::vector<float> feature3Vector;
        std::vector<float> feature4Vector;
        float featureThreshold[8] ={1.2,3.0, 0.13,0.5, 0.26,0.32,  0.06,0.12};

public:
        LaserRecognizer();
        ~LaserRecognizer(){};

        SamplePoint transform(const std::pair<float, float>&, const int);
        void segment(const sensor_msgs::LaserScanConstPtr&);
        void test();
        void generateDataSet();
        void train();
        void feature1(); //baseline
        void feature2(); //circle radius
        void feature3(); //width
        void feature4(); //average deviation
        void feature5(); //Sum of distances
};


#endif //RGBD_DETECT_LASERRECOGNIZER_H
