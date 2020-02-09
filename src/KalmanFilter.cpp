//
// Created by jieming on 06.02.20.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

//void KalmanFilter::initial(const Eigen::Vector2d& vec){
//    x_ = vec;
//}

void KalmanFilter::predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}



/*  1.initialize kf
 *  2.set id & count++
 *  3. set inital value
 **/
int KalmanBoxTraker::count=0;
KalmanBoxTraker::KalmanBoxTraker(const Eigen::Matrix<double,7,1>& xInital){
    id = count;
    count++;
    age =0;

    time_since_update=0;
    hit_streak=0;

    kf.x_ = MatrixXd(7, 1);
    kf.x_ = xInital;
    kf.F_ = MatrixXd(7, 7);
    kf.F_ << 1, 0, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 0, 1,
             0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 1;
    kf.P_ = MatrixXd(7, 7);
    kf.P_ << 10,  0,  0,  0,    0,    0,    0,
              0, 10,  0,  0,    0,    0,    0,
              0,  0, 10,  0,    0,    0,    0,
              0,  0,  0, 10,    0,    0,    0,
              0,  0,  0,  0, 1000,    0,    0,
              0,  0,  0,  0,    0, 1000,    0,
              0,  0,  0,  0,    0,    0, 1000;
    kf.H_ = MatrixXd(4, 7);
    kf.H_ << 1, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, 0,
             0, 0, 0, 1, 0, 0, 0;
    //could change
    kf.R_ = MatrixXd(4, 4);
    kf.R_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 10,0,
             0, 0, 0, 10;

    kf.Q_ = MatrixXd(7,7); /*related with dt*/
}

KalmanBoxTraker::~KalmanBoxTraker(){
}
/*    1. cal current F based on deltT
 *    2. cal current Q based on deltT
 *    3. call kf.predict
 * */
Eigen::Vector4d KalmanBoxTraker::predict(const double& dt){
    kf.F_(0,4) = dt;
    kf.F_(1,5) = dt;
    kf.F_(2,6) = dt;

    double noise_ax=5;
    double noise_ay=5;
    kf.Q_ << pow(dt,4)/4.0*pow(noise_ax,2), 0, pow(dt,4)/4.0*pow(noise_ax,2), 0, pow(dt,3)/2.0*pow(noise_ax,2), 0, pow(dt,3)/2.0*pow(noise_ax,2),
             0, pow(dt,4)/4.0*pow(noise_ay,2), pow(dt,4)/4.0*pow(noise_ay,2), 0, 0, pow(dt,3)/2.0*pow(noise_ay,2),  pow(dt,3)/2.0*pow(noise_ay,2),
             pow(dt,3)/2.0*pow(noise_ax,2), pow(dt,4)/4.0*pow(noise_ay,2), pow(dt,4)/4.0*pow(noise_ax,2)+pow(dt,4)/4.0*pow(noise_ay,2),
                0, pow(dt,3)/2.0*pow(noise_ax,2), pow(dt,3)/2.0*pow(noise_ay,2), pow(dt,3)/2.0*pow(noise_ax,2)+pow(dt,3)/2.0*pow(noise_ay,2),
             0, 0,0,0,0,0,0,
            pow(dt,3)/2.0*pow(noise_ax,2), 0, pow(dt,3)/2.0*pow(noise_ax,2), 0, pow(dt,2)*pow(noise_ax,2), 0, pow(dt,2)*pow(noise_ax,2),
            0, pow(dt,3)/2.0*pow(noise_ay,2), pow(dt,3)/2.0*pow(noise_ay,2), 0, 0, pow(dt,2)*pow(noise_ay,2), pow(dt,2)*pow(noise_ay,2),
            pow(dt,3)/2.0*pow(noise_ax,2), pow(dt,3)/2.0*pow(noise_ay,2), pow(dt,3)/2.0*pow(noise_ax,2),
                0, pow(dt,2)*pow(noise_ax,2), pow(dt,2)*pow(noise_ay,2), pow(dt,2)*pow(noise_ax,2)+pow(dt,2)*pow(noise_ay,2);

    kf.predict();
//    cout<<"after predict"<<kf.x_<<endl;
    age += 1;
    if(time_since_update>0){
        hit_streak = 0;
    }
    time_since_update += 1;
    history.emplace_back(convert_x_to_bbox(kf.x_));
    return history.back();
}

void KalmanBoxTraker::update(VectorXd meas){
    kf.update(meas);
}

Eigen::Matrix<double,4,1> KalmanBoxTraker::convert_x_to_bbox( const Eigen::Matrix<double,7,1>& x){
    /*
    Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
    [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 */
    double w = sqrt(x(2,0)*x(3,0));
    double h = x(2,0)/(w+0.00000001);
    Eigen::Matrix<double,4,1> bbox;
    bbox << x(0,0)-w/2.0, x(1,0)-h/2.0, x(0,0)+w/2.0, x(1,0)+h/2.0;
    return bbox;
}

void KalmanBoxTraker::getState(){
}