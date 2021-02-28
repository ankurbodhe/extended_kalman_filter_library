#ifndef EKF_H_
#define EKF_H_

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <file_parse_util.h>
#include <yaml.h>


class ExtendedKalmanFilter 
{

    private:

        // declare a state vector
        Eigen::VectorXd x_;

        // declare a measurement vector z_n
        Eigen::VectorXd z_;

        // declare a control vector u
        Eigen::VectorXd u_;

        // declare a state transition matrix F
        Eigen::MatrixXd F_;

        // declare the Control matrix G
        Eigen::MatrixXd G_;

        // declare a estimate uncertainty vector P
        Eigen::MatrixXd P_;

        // declare a process uncertainty noise matrix
        Eigen::MatrixXd Q_;

        // declare a obervation matrix H_n
        Eigen::MatrixXd H_;

        // declare a obervation matrix H_n for sensor 1
        Eigen::MatrixXd H1_;

        // declare a obervation matrix H_n
        Eigen::MatrixXd H2_;

        // declare a measurement coavariance matrix R_n
        Eigen::MatrixXd R_;

        // declare a measurement coavariance matrix R_n for Sensor 1
        Eigen::MatrixXd R1_;

        // declare a measurement coavariance matrix R_n for Sensor 2
        Eigen::MatrixXd R2_;

        // flag to indicate if jacobian needs to be calculated
        bool required_jacobian_calc;

        // flag to indicate if state is is initialized with first measurement or not
        bool initialize_state;

        // variable to store the timestamp for dt calculation
        long long prev_timestamp_;

    private:

        // function to perform the prediction step
        void predict();

        // function to perform the update step
        void update();

        // function to compute the process uncertainty noise matrix
        Eigen::MatrixXd compute_process_noise_Q(double t);

        // function to compute the observation matrix H_n
        Eigen::MatrixXd compute_observation_matrix_H(data_val data);

        // function to compute the state transition matrix F
        Eigen::MatrixXd compute_state_transition_F(double t);

        // function to compute the control matrix G
        Eigen::MatrixXd compute_control_matrix_G(double t);

        // function to set the control_vector R
        Eigen::MatrixXd set_measurement_noise_R(int sensor_id);

        // function to set the control_vector u_
        Eigen::VectorXd set_control_vector(data_val data);

        // function to set the measurement vector z_
        Eigen::VectorXd set_measurement_vector(data_val data);

    public:

        // constructor
        ExtendedKalmanFilter(std::string config_file);

        // function to process the measurement
        void process_sensor_data(data_val data);

        // function to access the system state
        Eigen::VectorXd access_system_state();







};




#endif /* EKF_H_ */