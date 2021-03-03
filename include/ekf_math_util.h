#ifndef EKF_MATH_UTIL_H_
#define EKF_MATH_UTIL_H_


#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <file_parse_util.h>



class EKFMath {

    public:
        Eigen::MatrixXd calculate_jacobian(Eigen::VectorXd x, char sensor_type);

        Eigen::VectorXd cartesian_to_polar(Eigen::VectorXd x, char sensor_type);

};





#endif /* EKF_MATH_UTIL_H_ */