#include "PID.h"

PID::PID(){}

PID::~PID() {}


#if 0
float PID::control(const std::vector<float>& reference, const std::vector<float>& current, std::vector<float>& solution){
    Eigen::Vector2f target_direction;
    target_direction << reference[0]-current[0], reference[1]-current[1];
    Eigen::Vector2f current_direction;
    current_direction << cos(current[2]), sin(current[2]);
    std::cout<< ( target_direction(0)*current_direction(0) + target_direction(1)*current_direction(1))<<std::endl;
    std::cout<< target_direction.norm()<<std::endl;
    std::cout<< current_direction.norm()<<std::endl;
    float theta = acos( ( target_direction(0)*current_direction(0) + target_direction(1)*current_direction(1))*1.0/(target_direction.norm()*current_direction.norm()));
//    theta = theta>1.57? 1.57:theta;
    if(isnan(theta))
    {
        std::cout<< "nana!!!!!!!!!!!!!"<<std::endl;
        std::cout<< ( target_direction(0)*current_direction(0) + target_direction(1)*current_direction(1))<<std::endl;
        std::cout<< target_direction.norm()<<std::endl;
        std::cout<< current_direction.norm()<<std::endl;
        std::cout<< ( target_direction(0)*current_direction(0) + target_direction(1)*current_direction(1))<<std::endl;
    }
    int sign =  (current_direction[0]*target_direction[1] - current_direction[1]*target_direction[0])>0 ? 1:-1;
    float error = theta * sign;//+ reference[2]-current[2];
    float u = kp_*error; //+ kd_*(error -error_last_);

    error_last_ = error;
//    solution[1] +=u;
//    solution[0] =0;

//    std::cout<<current[2]<< "   "<< current_direction[0] << "  "<<current_direction[1]<<std::endl;
//    std::cout<<theta  << " ---" << current[0] << "   " << current[1]  << "  "<< reference[0] << reference[1]<<std::endl;
    return u;
}
#endif

float PID::control(const std::vector<double>& reference, const std::vector<float>& current, std::vector<float>& solution){
/// \brief  ax+by+c = 0
    float k = tan(reference[2]);  /// test tan(90)!!!
    float b = reference[1] - k*reference[0];
    float A = k;
    float B = -1;
    float C = b;
    double vertical_dist = abs(A*current[0]+B*current[1]+C) / pow(pow(A,2)+pow(B,2), 0.5);
    if(A*current[0] + B*current[1]+C <0){
        vertical_dist *=  -1;
    }
    if( (k>0 && reference[2]>M_PI) || (k<0 && reference[2]>M_PI/2)){
        vertical_dist *= -1;
    }

    double vertical_error = vertical_dist*0.4 + 0.6*(reference[2]-current[2]);
//    double vertical_error = (reference[2]-current[2]);

    std::cout<<vertical_dist<< "  vertical_dist!!!!"<<std::endl;

    double steering_u = vertical_error*kp_angle_ ;//+ kd_*(vertical_error - vertical_error_last_);


    double euclidean_dist = pow(pow(reference[0]-current[0],2)+pow(reference[1]-current[1], 2), 0.5);
    double speed_u = euclidean_dist*kp_ ; //+ kd_*(euclidean_dist - euclidean_dist_last_);

    if (speed_u>1.1){
        speed_u = 1;
    } else if (speed_u<=0.1){
        speed_u= 0.1;
    }
    if(steering_u> M_PI/4){
        steering_u  =  M_PI/4;
    } else if (steering_u < -M_PI/4){
        steering_u  =  -M_PI/4;
    }
    solution[0] = speed_u;
    solution[1] = steering_u;
    vertical_error_last_ = vertical_error;
    euclidean_dist_last_ = euclidean_dist;

    std::cout <<speed_u <<"    speed u" <<std::endl;
    std::cout <<steering_u <<"    steering_u" <<std::endl;

    return vertical_error;
}

void PID::initialize(const std::vector<double>& reference, const std::vector<float>& current){
    euclidean_dist_last_ = pow(pow(reference[0]-current[0],2)+pow(reference[1]-current[1], 2), 0.5);

    float k = tan(reference[2]);  /// test tan(90)!!!
    float b = reference[1] - k*reference[0];
    float A = k;
    float B = -1;
    float C = b;
    float vertical_dist = abs(A*current[0]+B*current[1]+C) / pow(pow(A,2)+pow(B,2), 0.5);
    if(A*current[0] + B*current[1]+C <0){
        vertical_dist *=  -1;
    }
    vertical_error_last_ = vertical_dist;
}


void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}