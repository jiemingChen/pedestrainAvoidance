//
// Created by jieming on 07.02.20.
//

#include "SORT.h"

extern  bool CAMERADETECTSIG;
extern std::pair<ros::Time, std::vector<Eigen::Vector4d>> CAMERADetectOutput;
extern pthread_mutex_t Mutex;

SORT::SORT(ros::NodeHandle& n, ros::Rate& r):n_(n), rate(r){
    max_age = 1;
    min_hits = 3;
    frame_count=0;

    is_initialized = false;
    previous_timestamp_ = 0;
}

/* initialize for kalman xinital and dt
 * receive the form(centerX, centerY, w, h)
 * */
void SORT::initialize(){
    previous_timestamp_ = CAMERADetectOutput.first.nsec;
    is_initialized = true;

    for(int i=0; i< CAMERADetectOutput.second.size(); i++){
        auto det = CAMERADetectOutput.second.at(i);
//        cout<<det<<endl;
        auto state = convertToStateVector(det);
        KalmanBoxTraker tracker(state);
        trackers.push_back(tracker);
    }


}

void SORT::update(){
    frame_count += 1;
/*   1 get predicted locations from existing trackers, and remove error tracker*/
    std::vector<Eigen::Vector4d> trks;
    std::vector<int> toDelIndx;
    bool isN;
    double dt = (CAMERADetectOutput.first.nsec - previous_timestamp_)/(1000000000.0)+0.0001;

    previous_timestamp_ = CAMERADetectOutput.first.nsec;
    measured = CAMERADetectOutput.second;
    // for plot
    vector<int> pid;
    //
    for(int i=0; i<trackers.size(); i++){
         auto& tracker = trackers[i];
         auto lastX = tracker.kf.x_;
         auto est = tracker.predict(dt);
         /*check value*/
         isN = false;
         for(int j=0; j<4; j++){
             if(isnan(est(j,0))){
                 toDelIndx.push_back(i);
                 isN = true;
                 cout<<dt<<endl;
                 cout<< "!!!!!山除了!!!"<<endl;
                 break;
             }
         }
         if(!isN){
            trks.emplace_back(est);
             pid.push_back(tracker.id);// for plot
         }
    }
    std::reverse(toDelIndx.begin(), toDelIndx.end());
    for(auto& dIdx: toDelIndx){
        cout<< "!!!!!镇山除了"<<endl;
        trackers.erase(trackers.begin()+dIdx);
    }
    assert(trackers.size()==trks.size());
    /*2. then use association*/
    cout<<"predict!!!!!"<<endl;
    for(auto& pred: trks){
        cout<<pred<<endl;
    }
    cout<<"measure!!!!!"<<endl;
    for(auto& me: measured){
        cout<<me<<endl;
    }
    auto associatedRst = associate_detections_to_trackers(trks, measured, 0.3);
    auto matchIdx= std::get<0>(associatedRst);
    auto unmatchM= std::get<1>(associatedRst);
    auto unmatchP= std::get<2>(associatedRst);
    cout<<"match result"<<endl;
    for (auto& mat: matchIdx){
        cout<<mat.first<<"   "<<mat.second<<endl;
    }
    cout<<"unmatchM result"<<endl;
    for (auto& mat: unmatchM){
        cout<<mat<<endl;
    }
    cout<<"unmatchP result"<<endl;
    for (auto& mat: unmatchP){
        cout<<mat<<endl;
    }
    cout<<"--------------------------------------------------"<<endl;
//    if(trks.size()>2){
//        cout<<endl;
//    }
#if 0
    if(!matchIdx.empty()){
        matchIdx.clear();
        matchIdx.emplace_back(0,0);
        unmatchM.clear();
        unmatchP.clear();
    }
#endif
//    update matched trackers with assigned detections
    for(int i=0; i<matchIdx.size(); i++){
        int measIdx = matchIdx[i].first;
        int trackerIdx = matchIdx[i].second;
        trackers[trackerIdx].update(measured[measIdx]);
    }
/*    create and initialise new trackers for unmatched detections*/
    for(auto& id: unmatchM){
        trackers.emplace_back(convertToStateVector(measured[id]));
    }
    int len = trackers.size();

    vector<pair<Eigen::MatrixXd, int>> trackRst;
    for(int i=len-1; i>=0; i--) {
        auto tracker = trackers[i];
        if( (tracker.time_since_update<1) && (tracker.hit_streak>=min_hits || frame_count<min_hits) ){
            // store rst
            auto bbox = tracker.getState();
            int id = tracker.id;
            trackRst.emplace_back(bbox,id);
        }

        if (tracker.time_since_update > max_age){
            trackers.erase(trackers.begin()+i);
        }
    }

    // imshow rst
    detImg =  detector->detectionImage.clone();
    cv::Mat detImg2 =  detector->detectionImage.clone();

    Eigen::Matrix<int,9,3> colors;
    colors << 0,0,255,  0,255,0,  255,0,0, 255,0,255, 0,255,255, 0,0,0, 125,125,255, 255,125,125, 125,255,125;
    int baseLine = 0;
    for(auto& rst:trackRst){
        int b = colors(rst.second%9,0);
        int g = colors(rst.second%9,1);
        int r = colors(rst.second%9,2);
        cv::rectangle(detImg, cv::Point(rst.first(0,0),rst.first(1,0)),  cv::Point(rst.first(2,0),rst.first(3,0)), cv::Scalar(b, g, r),
                      1, 1, 0);
        cv::Size labelSize = getTextSize(std::to_string(rst.second),
                                         cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::putText(detImg, std::to_string(rst.second),
                    cv::Point(rst.first(0,0), rst.first(1,0)-labelSize.height),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    }

    /*debug detect*/
    int ii=0;
    for(auto& tk:trks){
        cv::rectangle(detImg2, cv::Point(tk(0,0), tk(1,0)),  cv::Point(tk(2,0),tk(3,0)), cv::Scalar(255,255,0),
                      1, 1, 0);
        cv::Size labelSize = getTextSize(std::to_string(pid[ii]),
                                         cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::putText(detImg2, std::to_string(pid[ii]),
                    cv::Point(tk(0,0), tk(1,0)-labelSize.height),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        ii++;
    }

    cv::imshow("filtered",detImg);
    cv::imshow("predict",detImg2);

    cv::waitKey(1);
}



void trackThread(SORT& tracker){
    while(ros::ok()) {
        pthread_mutex_lock(&Mutex);

        if(CAMERADETECTSIG){
            CAMERADETECTSIG = false;

            if(!tracker.is_initialized){
                tracker.initialize();
            }
            else{
                tracker.update();
            }

            CAMERADetectOutput.second.clear();
        }

        pthread_mutex_unlock(&Mutex);
        ros::spinOnce();
        tracker.rate.sleep();
    }
}

/**
 * Assigns detections to tracked object (both represented as bounding boxes)
   Returns 3 lists of matches, unmatched_detections and unmatched_trackers
 */
std::tuple< vector<pair<int, int>>,vector<int>,vector<int> > SORT::associate_detections_to_trackers(const vector<Eigen::Vector4d>&pred, const vector<Eigen::Vector4d>&meas,float iou_threshold = 0.3){
    if(trackers.empty()){
        cout<<"nima.........."<<endl;
        vector<int> unmatchedM;
        vector<pair<int,int>> empty;
        vector<int> emptyy;
        for(int i=0; i<meas.size(); i++){
            unmatchedM.push_back(i);
        }
        return std::make_tuple(empty, unmatchedM, emptyy);
    }
    /*build edge matrix */
    std::vector<std::vector<int>> edges;
    std::vector<int> tempV;
    for(int i=0; i<meas.size(); i++){
        tempV.clear();
        for(int j=0; j<pred.size(); j++){
            tempV.push_back(iou(pred[j], meas[i]));  //add - become minimum problem
        }
        edges.push_back(tempV);
    }
    cout<<"edges"<<endl;
    for(int i=0; i<edges.size();i++){
        for(int j=0; j<edges[0].size();j++){
            cout<<edges[i][j]<<endl;
        }
    }
    /*match*/
    KM kmMatch;
    auto MatchRst = kmMatch.match(edges);
    return MatchRst; //idx
}



int SORT::iou(const Eigen::Vector4d & bbP, const Eigen::Vector4d& bbgt){
/* predict 是 两个对角, measured 要转换*/
    Eigen::Vector4d bbMeas;
    bbMeas<< bbgt[0] - bbgt[2]/2.0, bbgt[1] - bbgt[3]/2.0, bbgt[0] + bbgt[2]/2.0, bbgt[1] + bbgt[3]/2.0;
    auto xx1 = std::max(bbP[0], bbMeas[0]);
    auto yy1 = std::max(bbP[1], bbMeas[1]);
    auto xx2 = std::min(bbP[2], bbMeas[2]);
    auto yy2 = std::min(bbP[3], bbMeas[3]);
    auto w = std::max(0.0, xx2-xx1);
    auto h = std::max(0.0, yy2-yy1);
    auto wh = w*h;
    double o = wh / ((bbP[2]-bbP[0])*(bbP[3]-bbP[1]) + (bbMeas[2]-bbMeas[0])*(bbMeas[3]-bbMeas[1]) -wh);
    int rst = int(o*1000);
    return rst;
}

Eigen::Matrix<double,7,1> SORT::convertToStateVector(const Eigen::Vector4d& det){
    Eigen::Matrix<double,7,1> state;
    state << det(0,0),
                det(1,0),
                det(2,0)*det(3,0),
                det(2,0)*1.0/det(3,0),
                0.0, 0.0, 0.0;
//    for(int i=0; i<7;i++){
//        if(isnan(state(i,0)) ||state(i,0)>5000){
//            cout<<"woqu"<<endl;
//            cout<<state(i,0)<<endl;
//        }
//    }
    return state;
}

void SORT::getDetector(const YoloRecognizer* recog){
    detector = recog;
}

SORT::~SORT(){

}