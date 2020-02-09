//
// Created by jieming on 04.02.20.
//

#include "Tracker.h"

#include <utility>

extern  bool CAMERADETECTSIG;
extern std::pair<ros::Time, std::vector<Eigen::Vector4d>> CAMERADetectOutput;

extern pthread_mutex_t Mutex;

Tracker::Tracker(const ros::NodeHandle& n, ros::Rate& r):n_(n), rate(r){
     noise_ax = 5;
     noise_ay = 5;
     is_initialized = false;
     previous_timestamp_ = 0;
}

/*
 *    #get predicted locations from existing trackers.
 *    #update matched trackers with assigned detections
 *    #create and initialise new trackers for unmatched detections
 *    #remove dead tracklet
 * */
 */
void Tracker::update(){
    // only receive from camera
    if(CAMERADETECTSIG){
        CAMERADETECTSIG=false;

        if(!is_initialized) {
            hungarian_.initial(CAMERADetectOutput.second);
            for (int i = 0; i < hungarian_.previous_frame_.size(); i++) {
                kfV_.emplace_back(KalmanFilter());
                kfV_[i].initial(CAMERADetectOutput.second[i]);
            }
            previous_timestamp_ = CAMERADetectOutput.first.nsec;
            is_initialized = true;
        }
        else{
            // 1.data association
            hungarian_.association(CAMERADetectOutput.second);

            // 2.assign kalman filter
            for(int i=0; i<hungarian_.current_frame_.size(); i++){
                if(){
                    kfV_.push_back(KalmanFilter());
                }
                else{
                    kfV_[i].do();
                }
            }
        }

        CAMERADetectOutput.second.clear();
    }
}


void trackThread(Tracker& tracker){
    while(ros::ok()) {
        pthread_mutex_lock(&Mutex);
        tracker.update();
        pthread_mutex_unlock(&Mutex);

        ros::spinOnce();
        tracker.rate.sleep();
    }
}


#if 0
std::cout<<"tracking thread!!!!"<<std::endl;
        for(int i=0; i<CAMERADetectOutput.size(); i++){
            std::cout<< CAMERADetectOutput[i]<< std::endl;
        }


void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        //cout << "Kalman Filter Initialization " << endl;

        // set the state with the initial location and zero velocity
        kf_.x_ << measurement_pack.raw_measurements_[0],
                measurement_pack.raw_measurements_[1],
                0,
                0;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // 1. Modify the F matrix so that the time is integrated
    // 2. Set the process covariance matrix Q
    // 3. Call the Kalman Filter predict() function
    // 4. Call the Kalman Filter update() function
    //      with the most recent raw measurements_
    kf_.F_ << 1, 0, dt, 0,
            0, 1,  0,dt,
            0, 0,  1, 0,
            0, 0,  0, 1;
    kf_.Q_ << pow(dt,4)/4.0*pow(noise_ax,2), 0,                             pow(dt,3)/2.0*pow(noise_ax,2), 0,
            0,                             pow(dt,4)/4.0*pow(noise_ay,2), 0,        pow(dt,3)/2.0*pow(noise_ay,2),
            pow(dt,3)/2.0*pow(noise_ax,2), 0,                             pow(dt,2)*pow(noise_ax,2), 0,
            0,                             pow(dt,3)/2.0*pow(noise_ay,2), 0, pow(dt,2)*pow(noise_ay,2);
    VectorXd Z(2);
    Z << measurement_pack.raw_measurements_[0],
            measurement_pack.raw_measurements_[1];

    kf_.Predict();
    kf_.Update(Z);

    cout << "x_= " << kf_.x_ << endl;
    cout<<endl;
    cout << "P_= " << kf_.P_ << endl;
}
#endif