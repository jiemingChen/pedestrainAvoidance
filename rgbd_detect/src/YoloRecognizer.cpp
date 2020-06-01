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
    std::vector<float> box_vec;
    for(int i=0; i<nboxes; i++) {
        for (int j = 0; j < labelNames.size(); j++) {
            if (dets[i].prob[j] > thresh) {
                box b = dets[i].bbox;
                yoloRst.push_back(std::make_pair(labelNames[j], dets[i].prob[j]));

                box_vec.push_back(b.x);
                box_vec.push_back(b.y);
                box_vec.push_back(b.w);
                box_vec.push_back(b.h);
                boxes.push_back(box_vec);

                box_vec.clear();

//                break;
            }
        }
    }

    free_detections(dets, nboxes);
    free_image(im);
    free_image(sized);

}

YoloRecognizer::YoloRecognizer(ros::NodeHandle& n, ros::Rate& r):rate(r), labelfile("/home/jieming/car_ws/src/rgbd_detect/darknet/person.names"),  cfgfile(const_cast<char *>(("/home/jieming/car_ws/src/rgbd_detect/darknet/onlyperson.cfg"))),
weightfile(const_cast<char *>("/home/jieming/car_ws/src/rgbd_detect/darknet/onlyperson_last.weights"))
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

    lidarPub = n_.advertise<std_msgs::Float32MultiArray>("lidarpart", 100);
    cv_ptr = nullptr;
    cv_ptr_depth = nullptr;
    rgbSub_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, "/camera/rgb/image_raw", 40); //camera/color/image_raw  /camera/rgb/image_raw
    depthSub_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, "/camera/depth/image_raw", 40);
    scanSub_= new message_filters::Subscriber<sensor_msgs::LaserScan>(n_, "/scan", 40);
    odomSub_= new message_filters::Subscriber<nav_msgs::Odometry>(n_, "/odom", 10);
// for gazebo env
    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *rgbSub_, *depthSub_, *scanSub_, *odomSub_);
    sync_->registerCallback(boost::bind(&YoloRecognizer::combineCallback,this, _1, _2, _3, _4));
// for debug realsense
    //sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *rgbSub_, *rgbSub_);
    //sync_->registerCallback(boost::bind(&YoloRecognizer::camCallback,this, _1, _2));
// for debug laser scan
//    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *scanSub_, *scanSub_);
//    sync_->registerCallback(boost::bind(&YoloRecognizer::laserCallback,this, _1, _2));

    //pub_ = new ros::Publisher;
    //*pub_ = n_.advertise<sensor_msgs::Image>("chatter", 1000);
    while((cv_ptr== nullptr)  || (cv_ptr_depth== nullptr)){
//    while((cv_ptr== nullptr) ){
//    while((scan_ptr== nullptr) ){

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

Eigen::Vector3d YoloRecognizer::deProjection(const float& depth, const cv::Rect roi) {
    // set R,T,K
    float P[9] = { 616.99834, 0.0, 640.5, 0.0, 616.99834, 400.5, 0.0, 0.0, 1.0};
    float invP[9] = {    0.0016,         0,   -1.0381, 0,    0.0016,   -0.6491, 0,         0,    1.0000};


    cv::Mat projectionMatrix(3, 3, CV_32FC1, P);
    cv::Mat invProjectionMatrix(3, 3, CV_32FC1, invP);
    //cv::Mat invProjectionMatrix = projectionMatrix.inv(CV_SVD);

    //deprojection
    cv::Mat xy(3,1,CV_32FC1);

    xy.at<float>(0,0) = roi.x+roi.width/2.0;
    xy.at<float>(1,0) = roi.y+roi.height/2.0;
    xy.at<float>(2,0) = 1;
    cv::Mat XYZ(3,1,CV_32FC1);


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

    XYZ.release();
    xy.release();
    return odomXYZ;

}

void mainThread(YoloRecognizer& recognizer) {
    while(ros::ok()){
        pthread_mutex_lock(&Mutex);

#if 1
        recognizer.detectionImage =recognizer.cv_ptr->image;
        recognizer.depthImage = recognizer.cv_ptr_depth->image.clone();
        if(recognizer.depthImage.size().height<=0){
            throw("wei shen me");
        }
#else
        recognizer.detectionImage =cv::imread("/home/jieming/car_ws/src/rgbd_detect/darknet/person.jpg");
        recognizer.depthImage = recognizer.cv_ptr_depth->image;
#endif
         //cv::merge(std::vector<cv::Mat>(3, recognizer.cv_ptr_depth->image), recognizer.detectionDepthImage);
        if(CAMERADETECTSIG){
            cout<< "wo cao last time data not sent to sort!!!!!!!!!!!!!!! jm"<<endl;
            //recognizer.rate.sleep();
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
                if((topLeftx+w)>recognizer.detectionImage.cols){
                    w = recognizer.detectionImage.cols - topLeftx -1;
                }
                if((topLefty+h)>recognizer.detectionImage.rows){
                    h = recognizer.detectionImage.rows - topLefty -1;
                }

                CAMERADetectOutput.first = recognizer.cv_ptr->header.stamp;
                //CAMERADetectOutput.second.emplace_back(static_cast<double>(recognizer.boxes[i][0]*recognizer.detectionImage.cols), static_cast<double>(recognizer.boxes[i][1]*recognizer.detectionImage.rows),
                //                                       static_cast<double>(recognizer.boxes[i][2]*recognizer.detectionImage.cols), static_cast<double>(recognizer.boxes[i][3]*recognizer.detectionImage.rows));
                CAMERADetectOutput.second.emplace_back(static_cast<double>(recognizer.boxes[i][0]*recognizer.detectionImage.cols), static_cast<double>(recognizer.boxes[i][1]*recognizer.detectionImage.rows),
                                                       static_cast<double>(w), static_cast<double>(h));

                cv::rectangle(recognizer.showImg, cv::Rect(topLeftx, topLefty, w, h), cv::Scalar(0, 0, 255), 1, 1, 0);
            }
        }

        if(CAMERADetectOutput.second.size()>0){
            CAMERADETECTSIG = true;
            // for logging detection result
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


        cv::imshow("origion", recognizer.showImg);
        //cv::Mat depthf = recognizer.detectionDepthImage.mul(1.0/(10));
        //cv::imshow("matImage2", depthf);

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
        recognizer.test_yolo(const_cast<char *>((path+"onlyperson.cfg").c_str()) ,  const_cast<char *>((path+"onlyperson_last.weights").c_str()),
                  path+"coco.names", const_cast<char *>(( path+"person.jpg").c_str()) , 0.5);
        //ros::spinOnce();
        recognizer.rate.sleep();
    }
}

std::vector<double> YoloRecognizer::depthCalc(const Eigen::MatrixXd range) {
    std::vector<double> xy(2,0);
    double topLeftx = range(0,0);
    double topLefty = range(1,0);
    double w = range(2,0)-range(0,0);
    double h = range(3,0)-range(1,0);
    cv::Mat thresholdDetectionImagep, sortROI;
//    cv::threshold(cv_ptr_depth->image, thresholdDetectionImagep, 15, 15, CV_THRESH_TRUNC);
    if(depthImage.rows<=0 || depthImage.cols<=0){
        cout<<"wo qu qiguai"<<endl;
        depthImage = depthImage2.clone();
    }
    cv::threshold(depthImage, thresholdDetectionImagep, 15, 15, CV_THRESH_TRUNC);

    cv::Mat roi;
    if(topLeftx<0){
        topLeftx = 0;
    }
    else if(topLeftx>=depthImage.cols){
        topLeftx = depthImage.cols - 3;
    }

    if(topLefty<0){
        topLefty = 0;
    }
    else if(topLefty>=depthImage.rows){
        topLeftx = depthImage.rows - 3;
    }
    if(w<0 || h<0){
        cout<<"!!!!!!!!!!!!!!!!!!warning"<<endl;
    }
    if( (w+topLeftx)>depthImage.cols ){
        w = depthImage.cols - topLeftx -1;
    }
    if((h+topLefty>depthImage.rows)){
        h = depthImage.rows - topLefty -1;
    }

    thresholdDetectionImagep(cv::Rect(topLeftx, topLefty, w, h)).copyTo(roi);


    double depth = 0;
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

    if(cnt>0){
        depth = depth / cnt;
    }
    else{
        cout<<"cao depth"<<endl;
        depth = roi.at<float>(int(roi.rows * roi.cols/2));
    }
    xy[0] = depth;

    //TODO realsense version + store rst csv
    Eigen::Vector3d rst = deProjection(depth, cv::Rect(topLeftx, topLefty, w, h));
    xy[0] = rst(0);
    xy[1] = rst(1);
    depthImage2 = depthImage.clone();
    return  xy;

}


void laserRecogThread(YoloRecognizer& recognizer){
    while(ros::ok()) {
        recognizer.laserRecog();
        ros::spinOnce();
        recognizer.rate.sleep();
    }
}

void YoloRecognizer::laserRecog(){
    auto segs = laserRecognizer.segment(scan_ptr);
    pub(segs);
//    laserRecognizer.dTreeTest();
//            laserRecognizer.test();
//    laserRecognizer.train();
//        laserRecognizer.generateDataSet();
}

Eigen::Vector3d YoloRecognizer::transformGlobalCoord(float x, float y){
    Eigen::Vector3d baseXYZ, odomXYZ;
    Eigen::Matrix3d R; // baselink to odom
    Eigen::Vector3d T; // baselink to odom
    baseXYZ << static_cast<double>(x+(0.75-0.04479+0.01679)), static_cast<double>(y),
            static_cast<double>(1);

    T << odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y, odomPtr->pose.pose.position.z;
    Eigen::Quaterniond quaterniond(odomPtr->pose.pose.orientation.w, odomPtr->pose.pose.orientation.x, odomPtr->pose.pose.orientation.y, odomPtr->pose.pose.orientation.z);
    R = quaterniond.toRotationMatrix();
    odomXYZ = R*baseXYZ+T;
    return odomXYZ;
}

void YoloRecognizer::pub(std::vector<std::vector<SamplePoint>> segs){
    std_msgs::Float32MultiArray peopleMsg;
    for(auto i=0; i<segs.size(); i++){
        auto samples = segs[i];
        float x=0;
        float y=0;
        if(pow(samples.front().x - samples.back().x, 2) + pow(samples.front().y - samples.back().y, 2) > 2*2){
            continue;
        }

        for(auto& sample: samples){
            x += sample.x;
            y += sample.y;
        }
        x = x / samples.size();
        y = y / samples.size();
        auto vec = transformGlobalCoord(x, y);

        peopleMsg.data.push_back(i);
        peopleMsg.data.push_back(vec(0));
        peopleMsg.data.push_back(vec(1));
    }
    lidarPub.publish(peopleMsg);
}

void YoloRecognizer::camCallback(const ImageConstPtr& rgb, const ImageConstPtr& rgb2){
    cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);

}

void YoloRecognizer::laserCallback(const LaserScanConstPtr& scan, const LaserScanConstPtr& scan2){
    scan_ptr = scan;
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
        assert(cv_ptr_depth->image.size().width>0);
        assert(cv_ptr->image.size().height>0);
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


/*
 *     cv::Mat depthf = cv_ptr_depth->image.mul(1.0/(10));
    cv::rectangle(depthf, cv::Point(xy.at<float>(0,0),xy.at<float>(1,0)),  cv::Point(xy.at<float>(0,0)+1,xy.at<float>(1,0)+1), cv::Scalar(255), 8, 1, 0);
    cv::imshow("matImage2", depthf);
    cv::waitKey(1);
    */
