//
// Created by jieming on 26.01.20.
//self.imagePub.publish(self.bridge.cv2_to_imgmsg(self.detectionImage,"bgr8"))
//

#include "YoloRecognizer.h"

extern  bool CAMERADETECTSIG;
//extern std::vector<Eigen::Vector2d>  CAMERADetectOutput;
extern std::pair<ros::Time, std::vector<Eigen::Vector4d>> CAMERADetectOutput;

extern pthread_mutex_t Mutex;


image ipl_to_image(IplImage* src)
{
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    image im = make_image(w, h, c);
    unsigned char *data = (unsigned char *)src->imageData;
    int step = src->widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return im;
}


void YoloRecognizer::test_yolo(char *cfgfile, char *weightfile,  std::string labelfile, char *filename, float thresh) {

    srand(2222222);

//    cv::Mat src=cv::imread(filename);
#if 1
    IplImage ipl = cv_ptr->image;
    image im = ipl_to_image(&ipl);
    rgbgr_image(im);
#else
    char buff[256];
    char *input = buff;
    strncpy(input, filename, 256);
    image im=load_image_color(filename,0,0);
#endif


    image sized = letterbox_image(im, net->w, net->h);
//    float *X = sized.data;

    // set parameter
    float nms = 0.4;

    // run network
    //network_predict_image(net,im);
    layer l = net->layers[net->n - 1];
    network_predict(net, sized.data);

    // get bounding boxes
    int nboxes = 0;
    detection *dets = get_network_boxes(net, im.w, im.h, thresh, 0.5, 0, 1, &nboxes);

    // nms
    if (nms){
        do_nms_sort(dets, nboxes, labelNames.size(), nms);
    }

    //rst
    for(int i=0; i<nboxes; i++) {
        for (int j = 0; j < labelNames.size(); j++) {
            if (dets[i].prob[j] > thresh) {
                box b = dets[i].bbox;
                yoloRst.push_back(std::make_pair(labelNames[j], dets[i].prob[j]));
                std::vector<float> box;
                box.push_back(b.x);
                box.push_back(b.y);
                box.push_back(b.w);
                box.push_back(b.h);
                boxes.push_back(box);
//                break;
            }
        }
    }
    free_detections(dets, nboxes);
    free_image(im);
    free_image(sized);
}


YoloRecognizer::YoloRecognizer(ros::NodeHandle& n, ros::Rate& r):rate(r), labelfile("/home/jieming/car_ws/src/rgbd_detect/darknet/coco.names"),  cfgfile(const_cast<char *>(("/home/jieming/car_ws/src/rgbd_detect/darknet/yolov3-tiny.cfg"))),
weightfile(const_cast<char *>("/home/jieming/car_ws/src/rgbd_detect/darknet/yolov3-tiny.weights"))
{
    n_ = n;
    // yolo reladted initial
    std::ifstream file(labelfile);
    // load labels
    if(!file.is_open()){
        std::cout << "Error opening file";
        exit (1);
    }
    std::string className;
    while (getline(file, className))
        labelNames.push_back(className);

    net = load_network(cfgfile, weightfile, 0);
    set_batch_network(net, 1);


    // ros related initial   jm1,2
    cv_ptr = nullptr;
    cv_ptr_depth = nullptr;
    rgbSub_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, "/camera/rgb/image_raw", 3); //camera/color/image_raw  /camera/rgb/image_raw
    depthSub_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, "/camera/depth/image_raw", 3);
    scanSub_= new message_filters::Subscriber<sensor_msgs::LaserScan>(n_, "/scan", 3);
    odomSub_= new message_filters::Subscriber<nav_msgs::Odometry>(n_, "/odom", 3);

    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *rgbSub_, *depthSub_, *scanSub_, *odomSub_);
    sync_->registerCallback(boost::bind(&YoloRecognizer::combineCallback,this, _1, _2, _3, _4));
//    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *rgbSub_, *rgbSub_);
//    sync_->registerCallback(boost::bind(&YoloRecognizer::camCallback,this, _1, _2));

    //pub_ = new ros::Publisher;
    //*pub_ = n_.advertise<sensor_msgs::Image>("chatter", 1000);
    while((cv_ptr== nullptr)  || (cv_ptr_depth== nullptr)){
//    while((cv_ptr== nullptr) ){
        rate.sleep();
        ros::spinOnce();
        std::cout<<"stuck"<<std::endl;
    }
    std::cout<<"start!!!!"<<std::endl;
//    th1 = new thread(&YoloRecognizer::recognizationThread, this);
//    scanth2 = new thread(&YoloRecognizer::laserRecogThread, this);
//    mainth0 = new thread(&YoloRecognizer::mainThread, this);
//    mainThread
}


//TODO 主要误差 源于深度计算，最好能提取轮廓
void YoloRecognizer::deProjection(const float& depth, const cv::Rect roi) {
    // set R,T,K
    float P[12] = {313.3672393106557, 0.0, 320.5, 0.0, 0.0, 313.3672393106557, 240.5, 0, 0, 0, 1, 0};
    float invP[12] = {0.0032, 0, -1.0228, 0, 0.0032, -0.7675, 0, 0, 1, 0, 0,  0};
    cv::Mat projectionMatrix(3, 4, CV_32FC1, P);
    cv::Mat invProjectionMatrix(4, 3, CV_32FC1, invP);
    //cv::Mat invProjectionMatrix = projectionMatrix.inv(CV_SVD);

    //deprojection
    cv::Mat xy(3,1,CV_32FC1);

    xy.at<float>(0,0) = roi.x+roi.width/2.0;
    xy.at<float>(1,0) = roi.y+roi.height/2.0;
    xy.at<float>(2,0) = 1;

    cv::Mat XYZ(4,1,CV_32FC1);
    cv::Mat TOPLEFT(4,1,CV_32FC1);

    XYZ=invProjectionMatrix*(xy*depth);

    // transform to odom coord
    Eigen::Vector3d baseXYZ, odomXYZ;

    Eigen::Matrix3d R; // baselink to odom
    Eigen::Vector3d T; // baselink to odom
    baseXYZ << static_cast<double>(XYZ.at<float>(2,0)+(0.26+0.0075)), static_cast<double>(-XYZ.at<float>(0,0)),
            static_cast<double>(XYZ.at<float>(1,0)+(0.825+0.10));

    T << odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y, odomPtr->pose.pose.position.z;
    Eigen::Quaterniond quaterniond(odomPtr->pose.pose.orientation.w, odomPtr->pose.pose.orientation.x, odomPtr->pose.pose.orientation.y, odomPtr->pose.pose.orientation.z);
    R = quaterniond.toRotationMatrix();
    odomXYZ = R*baseXYZ+T;

//    CAMERADetectOutput.second.emplace_back(odomXYZ(0,0), odomXYZ(1,0));
//    CAMERADetectOutput.first = cv_ptr->header.stamp;
/*
 * TODO for realsense use lib
 * */

}


void mainThread(YoloRecognizer& recognizer) {
    while(ros::ok()){
        pthread_mutex_lock(&Mutex);

#if 1
        recognizer.detectionImage =recognizer.cv_ptr->image;
#else
        detectionImage =cv::imread("/home/jieming/car_ws/src/rgbd_detect/darknet/person.jpg");
#endif
//        cv::merge(std::vector<cv::Mat>(3, recognizer.cv_ptr_depth->image), recognizer.detectionDepthImage);
        if(CAMERADETECTSIG){
            cout<< "wo cao last time data not sent to sort!!!!!!!!!!!!!!! jm";
//            throw "wo cao last time data not sent to sort!!!!!!!!!!!!!!! jm";
//            recognizer.rate.sleep();
            CAMERADETECTSIG =false;
            CAMERADetectOutput.second.clear();
        }

        recognizer.showImg = recognizer.detectionImage.clone();
        for(int i=0; i<recognizer.yoloRst.size(); i++){
            if(recognizer.yoloRst[i].first == "person") {

                int topLeftx = static_cast<int>((recognizer.boxes[i][0] - recognizer.boxes[i][2] / 2) *
                                                recognizer.detectionImage.cols);
                int topLefty = static_cast<int>((recognizer.boxes[i][1] - recognizer.boxes[i][3] / 2) *
                                                recognizer.detectionImage.rows);
                int w = static_cast<int>((recognizer.boxes[i][2]) * recognizer.detectionImage.cols);
                int h = static_cast<int>((recognizer.boxes[i][3]) * recognizer.detectionImage.rows);
                if (topLeftx < 0)
                    topLeftx = 0;
                if (topLefty < 0)
                    topLefty = 0;
                //TODO need check boundary
                if((topLeftx+w)>recognizer.detectionImage.cols){
                    w = recognizer.detectionImage.cols - topLeftx -1;
                }
                if((topLefty+h)>recognizer.detectionImage.rows){
                    h = recognizer.detectionImage.rows - topLefty -1;
                }

                CAMERADetectOutput.first = recognizer.cv_ptr->header.stamp;
                CAMERADetectOutput.second.emplace_back(static_cast<double>(recognizer.boxes[i][0]*recognizer.detectionImage.cols), static_cast<double>(recognizer.boxes[i][1]*recognizer.detectionImage.rows),
                                                       static_cast<double>(recognizer.boxes[i][2]*recognizer.detectionImage.cols), static_cast<double>(recognizer.boxes[i][3]*recognizer.detectionImage.rows));



                cv::rectangle(recognizer.showImg, cv::Rect(topLeftx, topLefty, w, h), cv::Scalar(0, 0, 255), 1,
                              1, 0);
                cv::rectangle(recognizer.showImg, cv::Rect(topLeftx, topLefty, w, h), cv::Scalar(0, 0, 255),
                              1, 1, 0);

#if 1
                cv::Mat thresholdDetectionImagep, sortROI;
                cv::threshold(recognizer.cv_ptr_depth->image, thresholdDetectionImagep, 15, 15, CV_THRESH_TRUNC);
                cv::Mat roi;
                thresholdDetectionImagep(cv::Rect(topLeftx, topLefty, w, h)).copyTo(roi);


                float depth = 0;
                sortROI = roi.reshape(1, roi.rows * roi.cols);
                cv::sort(sortROI, sortROI, CV_SORT_ASCENDING);
                int cnt = 0;
                for (int ii = 0; ii < int(sortROI.rows * sortROI.cols * 0.35); ii++) {

                    auto tempV = roi.at<float>(ii);
                    if (tempV > 0.2 && tempV < 11) {
                        depth += tempV;
                        cnt++;
                    }
                }
                depth = depth / cnt;


//                int baseLine = 0;
//                cv::Size labelSize = getTextSize(recognizer.yoloRst[i].first + std::to_string(depth),
//                                                 cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
//                cv::putText(recognizer.showImg, recognizer.yoloRst[i].first + std::to_string(depth),
//                            cv::Point(topLeftx, topLefty + labelSize.height),
//                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);

                //TODO realsense version + store rst csv
                recognizer.deProjection(depth, cv::Rect(topLeftx, topLefty, w, h));
#endif
            }
        }

        if(CAMERADetectOutput.second.size()>0){
            CAMERADETECTSIG = true;
            #if 0
            {
                recognizer.outFile.open("/home/jieming/simdata/cam_Remote.csv", std::ios::app);
                for(auto& mat:CAMERADetectOutput) {
                    recognizer.outFile << mat(0, 0) << "," << mat(1, 0) << std::endl;
                }
                recognizer.outFile.close();
//                CAMERADetectOutput.clear();
            }
            #endif
        }


//        cv::Mat depthf = recognizer.detectionDepthImage.mul(1.0/(10));

        cv::imshow("origion", recognizer.showImg);
//        cv::imshow("matImage2", depthf);

        recognizer.boxes.clear();
        recognizer.yoloRst.clear();

        pthread_mutex_unlock(&Mutex);
        cv::waitKey(1);

        ros::spinOnce();
        recognizer.rate.sleep();
    }
}

void recognizationThread(YoloRecognizer& recognizer) {
    string path="/home/jieming/car_ws/src/rgbd_detect/darknet/";

    while(ros::ok()){
        recognizer.test_yolo(const_cast<char *>((path+"yolov3-tiny.cfg").c_str()) ,  const_cast<char *>((path+"yolov3-tiny.weights").c_str()),
                  path+"coco.names", const_cast<char *>(( path+"person.jpg").c_str()) , 0.5);

//        ros::spinOnce();
        recognizer.rate.sleep();
    }
}


void laserRecogThread(YoloRecognizer& recognizer){
    while(ros::ok()) {
        //1. segmentation
        recognizer.laserRecognizer.segment(recognizer.scan_ptr);
        //2. detect
//        laserRecognizer.detect();
//        laserRecognizer.test();
//        laserRecognizer.train();
//        laserRecognizer.generateDataSet();
        ros::spinOnce();
        recognizer.rate.sleep();
    }
}

void YoloRecognizer::camCallback(const ImageConstPtr& rgb, const ImageConstPtr& rgb2){
    cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);

}

void YoloRecognizer::combineCallback(const ImageConstPtr& rgb,  const ImageConstPtr& depth, const LaserScanConstPtr& scan, const OdometryConstPtr& odometry)
{

    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
        cv_ptr_depth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
        scan_ptr = scan;
        odomPtr = odometry;

        assert(cv_ptr_depth->image.size() == cv_ptr->image.size());
        assert(cv_ptr_depth->image.size().height>0);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}


YoloRecognizer::~YoloRecognizer()
{
    delete rgbSub_;
    delete depthSub_;
    delete scanSub_;
    delete sync_;
    delete th1;
    delete scanth2;
    delete mainth0;
}