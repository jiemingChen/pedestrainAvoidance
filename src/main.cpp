#include <thread>
#include <iostream>

#include "YoloRecognizer.h"
#include "Tracker.h"
#include "SORT.h"
void mainThread(YoloRecognizer& );
void recognizationThread(YoloRecognizer& );
void laserRecogThread(YoloRecognizer& );
void trackThread(SORT&);

bool CAMERADETECTSIG=false;
//std::pair<std::vector<Eigen::Vector2d>> CAMERADetectOutput;
std::pair<ros::Time, std::vector<Eigen::Vector4d>> CAMERADetectOutput;

pthread_mutex_t Mutex;





int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle n_;
    ros::Rate rate(10);
    YoloRecognizer recognizer(n_, rate);
    SORT tracker(n_, rate);
    tracker.getDetector(&recognizer);

    thread camerath1(recognizationThread, std::ref(recognizer));
//    thread scanth2(boost::bind(&laserRecogThread, std::ref(recognizer)));
    thread mainth0(mainThread, std::ref(recognizer));
    thread trackth3(trackThread, std::ref(tracker));

    camerath1.join();
//    scanth2.join();
    mainth0.join();
    trackth3.join();
}
