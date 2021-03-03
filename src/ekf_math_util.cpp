#include <ekf_math_util.h>


Eigen::MatrixXd EKFMath::calculate_jacobian(Eigen::VectorXd x, char sensor_type) {

    Eigen::MatrixXd Hj;

    if (sensor_type =='R') {

        Hj = Eigen::MatrixXd(3, x.size());
        Hj = Eigen::MatrixXd::Zero(3, x.size());

        double px = x(0);
        double py = x(1);
        double vx = x(2);
        double vy = x(3);

        double b1 = px*px + py*py;
        double b2 = sqrt(b1);
        double b3 = b1 * b2;

        //check division by zero
        if(fabs(b1) < 0.0001){
            std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
            return Hj;
        }

        // compute Jacobian matrix

        Hj << (px/b2),                 (py/b2),                  0.0,    0.0,
              -(py/b1),                (px/b1),                  0.0,    0.0,
              (py*(vx*py - vy*px)/b3), (px*(px*vy - py*vx)/b3),  px/b2,  py/b2;
    }

    return Hj;
}

Eigen::VectorXd EKFMath::cartesian_to_polar(Eigen::VectorXd x, char sensor_type) {

    double px, py, vx, vy, rho, phi, rho_dot;
    Eigen::VectorXd y;

    px = x(0);
    py = x(1);
    vx = x(2);
    vy = x(3);

    if (sensor_type == 'R') {

        y = Eigen::VectorXd(3);

        rho = sqrt(px*px + py*py);

        if (rho < 0.0001)
            rho = 0.0001;

        phi = atan2(py, px);

        rho_dot = (px*vx + py*vy)/rho;

        y << rho, phi, rho_dot;
    }
    return y;
}