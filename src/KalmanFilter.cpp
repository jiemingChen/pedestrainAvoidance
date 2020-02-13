//
// Created by jieming on 06.02.20.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
//    cout<<P_<<endl;
    P_ = F_ * P_ * Ft + Q_;
//    cout<<P_<<endl;

}

void KalmanFilter::update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    K_ = K;
    //new estimate
    auto temp = x_;
    x_ = x_ + (K * y);
    if(temp(0,0)-x_(0,0)>10){
        cout<<P_<<endl;
        cout<<"!!"<<endl;
        cout<<K<<endl;
        cout<<"!!"<<endl;
    }

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
//    P_ = (I - K * H_) * P_;
//    P = (I-KH)P(I-KH)' + KRK'
    P_ = (I - K * H_) * P_*(I - K * H_).transpose() + K*R_*K.transpose();

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
    hits = 0;
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
    kf.P_ <<  30,  0,  0,  0,    0,    0,    0,
              0,   30,  0,  0,    0,    0,    0,
              0,  0,   10,  0,    0,    0,    0,
              0,  0,  0,   10,    0,    0,    0,
              0,  0,  0,  0,      10000,    0,    0,
              0,  0,  0,  0,    0,     10000,    0,
              0,  0,  0,  0,    0,    0,     10000;

    kf.H_ = MatrixXd(4, 7);
    kf.H_ << 1, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, 0,
             0, 0, 0, 1, 0, 0, 0;
    //could change
    kf.R_ = MatrixXd(4, 4);
    kf.R_ << 5, 0, 0, 0,
             0, 5, 0, 0,
             0, 0, 10,0,
             0, 0, 0, 10;

    kf.Q_ = MatrixXd(7,7); /*related with dt*/
    //jm
//    kf.Q_ <<  1, 0, 0, 0, 0, 0, 0,
//              0, 1, 0, 0, 0, 0, 0,
//              0, 0, 1, 0, 0, 0, 1,
//              0, 0, 0, 1, 0, 0, 0,
//              0, 0, 0, 0, 0.1, 0, 0,
//              0, 0, 0, 0, 0, 0.1, 0,
//              0, 0, 0, 0, 0, 0, 0.01;
    kf.Q_ <<  1, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0.01, 0, 0,
              0, 0, 0, 0, 0, 0.01, 0,
              0, 0, 0, 0, 0, 0, 0.0001;
}

KalmanBoxTraker::~KalmanBoxTraker(){
}

Eigen::Vector4d KalmanBoxTraker::predict(const double& dt){
    kf.F_(0,4) = dt;
    kf.F_(1,5) = dt;
    kf.F_(2,6) = dt;

    double noise_ax=30;
    double noise_ay=30;
    double noise_area=5;
//jm
    kf.Q_(0,0) = pow(dt,4)/4.0*pow(noise_ax,2);
    kf.Q_(0,4) = pow(dt,3)/2.0*pow(noise_ax,2);

    kf.Q_(1,1) =pow(dt,4)/4.0*pow(noise_ay,2);
    kf.Q_(1,5) =pow(dt,3)/2.0*pow(noise_ay,2);

    kf.Q_(2,2) =pow(dt,4)/4.0*pow(noise_area,2);
    kf.Q_(2,6) =pow(dt,3)/2.0*pow(noise_area,2);

    kf.Q_(4,0) = pow(dt,3)/2.0*pow(noise_ax,2);
    kf.Q_(4,4) = pow(dt,2)*pow(noise_ax,2);

    kf.Q_(5,1) = pow(dt,3)/2.0*pow(noise_ay,2);
    kf.Q_(5,5) = pow(dt,2)*pow(noise_ay,2);

    kf.Q_(6,2) = pow(dt,3)*pow(noise_area,2);
//    kf.Q_(5,6) = pow(dt,2)*pow(noise_ay,2);

//    kf.Q_(2,2) =1;
//    kf.Q_(3,3) =1;
//    kf.Q_(4,4) =0.01;
//    kf.Q_(5,5) =0.01;
//    kf.Q_(6,6) =0.01;



    if((kf.x_(6)+kf.x_(2))<=0){
        kf.x_[6] *= 0.0;
    }
    kf.predict();

    age += 1;
    if(time_since_update>0){
        hit_streak = 0;
    }
    time_since_update += 1;
    history.emplace_back(convert_x_to_bbox(kf.x_));
    return history.back();
}

void KalmanBoxTraker::update(VectorXd meas){
    time_since_update = 0;
    history.clear();//
    hits += 1; // mei yong
    hit_streak += 1;
    kf.update(convertToZ(meas));
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

Eigen::MatrixXd KalmanBoxTraker::getState(){
    return convert_x_to_bbox(kf.x_);
}

Eigen::MatrixXd KalmanBoxTraker::convertToZ(const Eigen::Vector4d& meas){
    /* 4X1 */
    Eigen::Matrix<double,4,1> state;
    state << meas(0,0),
            meas(1,0),
            meas(2,0)*meas(3,0),
            meas(2,0)*1.0/meas(3,0);
    return state;
}


