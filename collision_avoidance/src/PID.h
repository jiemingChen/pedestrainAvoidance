#ifndef PID_H
#define PID_H
#include  "collision_avoidance/header.h"

class PID {
 public:
  PID();
  virtual ~PID();
  void initialize(const std::vector<double>&, const std::vector<float>&);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);


  double TotalError();
  float control(const std::vector<double>&, const std::vector<float>&, std::vector<float>&);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double kp_= 1;
  double kp_angle_=1;

    double ki_=0;
  double kd_=1.1;
  float vertical_error_last_=0;
  float euclidean_dist_last_=0;
};

#endif  // PID_H