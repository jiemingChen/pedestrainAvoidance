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
        auto state = convertToStateVector(det);
        KalmanBoxTraker tracker(state);
        trackers.push_back(tracker);
    }
//    cout<< "initial"<<trackers[0].kf.x_<<endl;
//    cout<<"-------------"<<endl;

}

void SORT::update(){
    frame_count += 1;
/*   1 get predicted locations from existing trackers, and remove error tracker*/
    std::vector<Eigen::Vector4d> trks;
    std::vector<int> toDelIndx;
    bool isN;
    double dt = (CAMERADetectOutput.first.nsec - previous_timestamp_)/(1000000000.0);
    previous_timestamp_ = CAMERADetectOutput.first.nsec;
    measured = CAMERADetectOutput.second;

    for(int i=0; i<trackers.size(); i++){
         auto& tracker = trackers[i];
         auto est = tracker.predict(dt);
//         cout<<est<<endl;
         /*check value*/
         isN = false;
         for(int j=0; j<4; j++){
             if(isnan(est(j,0))){
                 toDelIndx.push_back(i);
                 isN = true;
                 cout<< "!!!!!镇山除了!!!"<<endl;
             }
         }
         if(!isN){
//             trks.row(i) = est;
            trks.emplace_back(est);
         }
    }
    std::reverse(toDelIndx.begin(), toDelIndx.end());
    for(auto& dIdx: toDelIndx){
        cout<< "!!!!!镇山除了"<<endl;
        trackers.erase(trackers.begin()+dIdx);
    }
    assert(trackers.size()==trks.size());
//    cout<<"endl"<<endl;
    /*2. then use association*/
    auto associatedRst = associate_detections_to_trackers(trks, measured, 0.3);
    auto matchIdx= std::get<0>(associatedRst);
    auto unmatchM= std::get<1>(associatedRst);
    auto unmatchP= std::get<2>(associatedRst);
//    update matched trackers with assigned detections
    for(int i=0; i<matchIdx.size(); i++){
        int measIdx = matchIdx[i].first;
        int trackerIdx = matchIdx[i].second;
        //TODO 测量值要转换
        trackers[trackerIdx].update(measured[measIdx]);
    }
/*    create and initialise new trackers for unmatched detections*/

//    remove dead tracklet

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
    if(trackers.size()==0){
        cout<<"nima.........."<<endl;
        vector<int> unmatchedM;
        for(int i=0; i<meas.size(); i++){
            unmatchedM.push_back(i);
        }
        return std::make_tuple(vector<pair<int,int>>(), unmatchedM, vector<int>());
    }
    /*build edge matrix */
    std::vector<std::vector<float>> edges;
    std::vector<float> tempV;
    for(int i=0; i<meas.size(); i++){
        tempV.clear();
        for(int j=0; j<pred.size(); j++){
            tempV.push_back(-iou(pred[i], meas[i]));  //add - become minimum problem
        }
        edges.push_back(tempV);
    }
    /*match*/
    KM kmMatch;
    auto MatchRst = kmMatch.match(edges);
    return MatchRst; //idx
}

float SORT::iou(const Eigen::Vector4d & bbP, const Eigen::Vector4d& bbgt){
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
    float o = wh / ((bbP[2]-bbP[0])*(bbP[3]-bbP[1]) + (bbMeas[2]-bbMeas[0])*(bbMeas[3]-bbMeas[1]) -wh);
    return o;
}

Eigen::Matrix<double,7,1> SORT::convertToStateVector(const Eigen::Vector4d& det){
    Eigen::Matrix<double,7,1> state;
    state << det(0,0),
                det(1,0),
                det(2,0)*det(3,0),
                det(2,0)*1.0/det(3,0),
                0, 0, 0;
    return state;
}

SORT::~SORT(){

}