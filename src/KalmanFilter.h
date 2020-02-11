//
// Created by jieming on 06.02.20.
//
#ifndef RGBD_DETECT_KALMANFILTER_H
#define RGBD_DETECT_KALMANFILTER_H

#include "YoloRecognizer.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
    public:

    // state vector
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // state transistion matrix
    MatrixXd F_;

    // process covariance matrix
    MatrixXd Q_;

    // measurement matrix
    MatrixXd H_;

    // measurement covariance matrix
    MatrixXd R_;

    KalmanFilter();
    virtual ~KalmanFilter();

//    void initial(const Eigen::Vector2d&);
    void predict();
    void update(const VectorXd &z);
};

class KalmanBoxTraker{
public:
    static int count;      /*default 0*/

    KalmanFilter kf;
    int time_since_update; /*多次没检测到, delet*/
    int hit_streak;        /*update*/
    unsigned int id;
    int age;
    vector<Eigen::Vector4d> history;
    int hits;

    KalmanBoxTraker(const Eigen::Matrix<double,7,1>&);
    ~KalmanBoxTraker();
    Eigen::Vector4d predict(const double&);
    void update(VectorXd);
    Eigen::MatrixXd getState();

    static Eigen::Matrix<double,4,1> convert_x_to_bbox(const Eigen::Matrix<double,7,1>&);
    static Eigen::MatrixXd convertToZ(const Eigen::Vector4d&);

};
#endif //RGBD_DETECT_KALMANFILTER_H
