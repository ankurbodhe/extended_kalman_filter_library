/*
 *
 * 
 * 
 */


#include <ekf.h>
#include "yaml-cpp/yaml.h"
#include <yaml-cpp/parser.h>
#include <yaml.h>
#include <math.h>
#include <ekf_math_util.h>


/* @brief : The constructor reads the config file and initializes the matrices used for implementing the kalman filter
 * @params: config file path
 * @return: all kalman filter variables are initialized 
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(std::string config_file) {

    std::cout << "\n-------- Loading Kalman Filter configuration -----------" << std::endl;

    // read the config file
    YAML::Node config = YAML::LoadFile(config_file);

    // read System model type to decide the State Transition matrix and the state vector
    if (config["System_Model"]) {

        cout << "Setting system model to : " << config["System_Model"].as<std::string>() << std::endl;

    }

    // Initialize transition matrix based on the type of system model
    if(config["System_Model"].as<std::string>() == "const_accel"){

        /* set the state transition matrix to 
         *      1 0 1 0
         *  F = 0 1 0 1  
         *      0 0 1 0
         *      0 0 0 1
        */

        F_ = Eigen::MatrixXd(4, 4);
        
        F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
        
        /* set the state vector
         *      
         * x =  [px py vx vy]
         *      
        */
        x_ = Eigen::VectorXd(4);

        /* set the state transition matrix to 
         *      1    0    0    0
         *  P = 0    1    0    1  
         *      0    0    1000 0
         *      0    0    0    1000
        */
        P_ = Eigen::MatrixXd(4, 4);
        P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    }
    else if(config["System_Model"].as<std::string>() == "const_vel") {

        std::cout<<"Kalman filter for this model is not available in this version"<<std::endl;
        std::cout<<"Exiting Program...." <<std::endl;
        std::exit(10);
    }
    else {

        std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
        std::cout << "Exiting Program...." <<std::endl;
        std::exit(10);
    }

    // read Sensor 1 type to decide the observation matrix and the control matrix
    if(config["Sensor_1"]) {

        std::cout << "Setting sensor 1 to " << config["Sensor_1"].as<std::string>() << std::endl;
    }

    if ((config["Sensor_1"].as<std::string>() == "odometer") || (config["Sensor_1"].as<std::string>() == "GPS")) {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Constant(4, 0.0);

            /* Set the observation matrix H1
             *       1 0 0 0
             *  H1 = 0 1 0 0  
             *       0 0 1 0
             *       0 0 0 1 
             */
            H1_ = Eigen::MatrixXd(4, 4);
            H1_ << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;    
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }

    }
    else if (config["Sensor_1"].as<std::string>() == "IMU") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G
            G_ = Eigen::MatrixXd(4, 2);

            G_ << 0.5, 0.0,
                  0.0, 0.5,
                  1.0, 0.0,
                  0.0, 1.0;
            
            // Set the control vector u
            u_ = Eigen::VectorXd(2);    
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }

    }
    else if (config["Sensor_1"].as<std::string>() == "Laser") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Constant(2, 0.0);

            /* Set the observation matrix H1
             *  H1 = 1 0 0 0
             *       0 1 0 0   
             */
            H1_ = Eigen::MatrixXd(2, 4);
            H1_ << 1, 0, 0, 0,
                   0, 1, 0, 0;
            
            /* Set the measurement matrix R
             *  R1 = 0.0225 0 
             *       0 0.0225   
             */
            R1_ = Eigen::MatrixXd(2, 2);
            R1_ << 0.0225, 0,
                   0, 0.0225;    
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }

        req_jacobian_calc_s1 = false;


    }
    else if (config["Sensor_1"].as<std::string>() == "Radar") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Zero(2);

            /* Set the observation matrix H1
             *  H1 = Hj
             *          
             */
            H1_ = Eigen::MatrixXd(3, 4);

            /* Set the measurement matrix R
             *  R1 = 0.09 0      0 
             *       0    0.0009 0
             *       0    0      0.09   
             */
            R1_ = Eigen::MatrixXd(3, 3);
            R1_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }

        req_jacobian_calc_s1 = true;
        
    }
    else  {
        
        std::cout << "Unable to find sensor model. Please set the sensor model accurately to proceed." << std::endl;
        std::cout << "Exiting Program...." << std::endl;
        std::exit(10); 
    }

    // read Sensor 1 type to decide the observation matrix and the control matrix
    if(config["Sensor_2"]) {

        std::cout << "Setting sensor 2 to " << config["Sensor_2"].as<std::string>() << std::endl;
    }

    if ((config["Sensor_2"].as<std::string>() == "odometer") || (config["Sensor_2"].as<std::string>() == "GPS")) {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Constant(4, 0);

            /* Set the observation matrix H1
             *       1 0 0 0
             *  H1 = 0 1 0 0  
             *       0 0 1 0
             *       0 0 0 1 
             */
            H2_ = Eigen::MatrixXd(4, 4);
            H2_ << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;    
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }

    }
    else if (config["Sensor_2"].as<std::string>() == "IMU") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G
            G_ = Eigen::MatrixXd(4, 2);

            G_ << 0.5, 0.0,
                  0.0, 0.5,
                  1.0, 0.0,
                  0.0, 1.0;
            
            // Set the control vector u
            u_ = Eigen::VectorXd(4);    
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }

    }
    else if (config["Sensor_2"].as<std::string>() == "Laser") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Constant(4, 0.0);

            /* Set the observation matrix H1
             *  H1 = 1 0 0 0
             *       0 1 0 0   
             */
            H2_ = Eigen::MatrixXd(2, 4);
            H2_ << 1, 0, 0, 0,
                   0, 1, 0, 0;
            /* Set the measurement matrix R
             *  R2 = 0.0225 0 
             *       0 0.0225   
             */
            R2_ = Eigen::MatrixXd(2, 2);
            R2_ << 0.0225, 0,
                   0, 0.0225;     
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            exit(10);
        }
        req_jacobian_calc_s2 = false;

    }
    else if (config["Sensor_2"].as<std::string>() == "Radar") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Zero(2);

            /* Set the observation matrix H1
             *  H2 = Hj
             *          
             */
            H2_ = Eigen::MatrixXd(3, 4);

            /* Set the measurement matrix R
             *  R1 = 0.09 0      0 
             *       0    0.0009 0
             *       0    0      0.09   
             */
            R2_ = Eigen::MatrixXd(3, 3);
            R2_ << 0.09, 0,      0,
                   0,    0.0009, 0,
                   0,    0,      0.09;
   
        }
        // if the system model is constant velocity
        else if(config["System_Model"].as<std::string>() == "const_vel") {
            
            std::cout << "Kalman filter for this model is not available in this version." << std::endl;
            std::cout << "Exiting Program...." <<std::endl;
            std::exit(10);
        }
        else {
            
            std::cout << "Unable to find system model. Please set the system model accurately to proceed." << std::endl;
            std::cout << "Exiting Program...." << std::endl;
            std::exit(10);
        }
        req_jacobian_calc_s2 = true;
    }
    else  {
        
        std::cout << "Unable to find sensor model. Please set the sensor model accurately to proceed." << std::endl;
        std::cout << "Exiting Program...." << std::endl;
        std::exit(10); 
    }

    initialize_state = false;

    std::cout <<"The are the set Paramters : " <<std::endl;
    std::cout <<"State transition Matrix F : " <<std::endl;
    std::cout <<F_ <<std::endl;
    std::cout <<"Control Matrix G : " <<std::endl;
    std::cout <<G_ <<std::endl;
    std::cout <<"Estimate uncertainty matrix P : " <<std::endl;
    std::cout <<P_ <<std::endl;
    std::cout <<"Observation matrix for sensor 1 H1 : " <<std::endl;
    std::cout <<H1_ <<std::endl;
    std::cout <<"Measurement error matrix for sensor 1 R1 : " <<std::endl;
    std::cout <<R1_ <<std::endl;

}


/**
 * @brief : This function computes the state transition matrix wrt to time interval dt
 * @params: time interval dt
 * @return: updates the state transition matrix F_
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_state_transition_F(double t) {

    Eigen::MatrixXd A = F_;

    A(0, 2) = t;
    A(1, 3) = t;

    return A;
}


/** 
 * @brief : This function computes the process noise covariance Q_ wrt to time
 * @params: time interval dt
 * @return: Updates the process noise matrix Q_
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_process_noise_Q(double t) {

    Eigen::MatrixXd A(x_.size(), x_.size());

    double t_2 = t * t;
    double t_3 = t_2 * t;
    double t_4 = t_3 * t;

    double noise_ax = 9;
    double noise_ay = 9;

    A << t_4/4*noise_ax,    0.0,            t_3/2*noise_ax, 0.0,
         0.0,               t_4/4*noise_ay, 0.0,            t_3/2*noise_ay,
         t_3/2*noise_ax,    0.0,            t_2*noise_ax,   0.0,
         0.0,               t_3/2*noise_ay, 0.0,            t_2*noise_ay;

    return A;

}


/**
 * @brief :
 * @params:
 * @return:
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_control_matrix_G(double t) {


    return G_;

}


/** 
 * @brief : 
 * @params:
 * @return:
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_observation_matrix_H(int sensor_id, bool s1_j, bool s2_j) {

    Eigen::MatrixXd H(z_.size(), x_.size());
    char s1_type, s2_type;

    if (sensor_id == 1) {

        if (s1_j){
            s1_type = 'R';
            H = tools.calculate_jacobian(x_, s1_type);
        } 
        else {
            H = H1_;
        }
    }
    else if (sensor_id == 2) {

        if (s2_j) {
            s2_type = 'R';
            H = tools.calculate_jacobian(x_, s2_type);
        }
        else {
            H = H2_;
        }
    }
    return H;
}

/** 
 * @brief : This function computes the control vector given the sensor input
 * @params: Sensor measurement data
 * @return: u_ vector
 */
Eigen::VectorXd ExtendedKalmanFilter::set_control_vector(data_val data) {

    return u_;
}

/** 
 * @brief : This function calculates the measurement vector given the sensor input
 * @params: Sensor measurement data
 * @return: z_ vector
 */
Eigen::VectorXd ExtendedKalmanFilter::set_measurement_vector(data_val data) {

    Eigen::VectorXd z;

    if (data.sensor_type == 'L') {

        z = Eigen::VectorXd(2);
        z << data.lidar_meas_px, data.lidar_meas_py;
    }
    else if (data.sensor_type == 'R') {

        z = Eigen::VectorXd(3);
        z << data.radar_meas_rho, data.radar_meas_phi, data.meas_rho_dot;
    }

    return z;
}


/**
 * @brief : This function sets the measurement noise matrix R based on the sensor id
 * @params: sensor id
 * @return: Measurement noise matrix R
 */
Eigen::MatrixXd ExtendedKalmanFilter::set_measurement_noise_R(int sensor_id) {

    Eigen::MatrixXd R(z_.size(), z_.size());

    if (sensor_id == 1)
        R = R1_;
    else if (sensor_id == 2)
        R = R2_;
    
    return R;

}

/**
 * @brief : This function performs the State prediction and the estimate uncertainty prediction
 * @params: 
 * @return:
 */
void ExtendedKalmanFilter::predict() {

    //std::cout << "doing prediction" << std::endl;

    // compute the state prediction
    x_ = F_*x_ + G_*u_;

    //std::cout << "finsihed state prediction" << std::endl;
    // compute estimate uncertainty covariance
    P_ = F_*P_*(F_.transpose()) + Q_;

}

/** 
 * @brief : The function perform the state update, the estimate uncertainty update based on the sensor measurement
 * @params: 
 * @return:
 */
void ExtendedKalmanFilter::update() {

    //std::cout << "doing update" << std::endl;

    // calculate the measurement prediction
    Eigen::VectorXd z_pred = H_ * x_;
    
    //std::cout << "finished measurement pred" << std::endl << z_pred << std::endl;
    // compute the Kalman Gain
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S  = (H_ * P_ * Ht) + R_;
    Eigen::MatrixXd Kn = P_ * Ht * S.inverse();
    //std::cout << "finished Kalman gain calc" << std::endl << Kn << std::endl;
    // calculate the state update
    x_ = x_ + Kn*(z_ - z_pred);
    //std::cout << "finished state update calc" << std::endl << x_ << std::endl;
    // calculate the covariance update
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - Kn * H_) * P_;
}

/** 
 * @brief : The function perform the state update, the estimate uncertainty update based on the sensor measurement
 * @params: 
 * @return:
 */
void ExtendedKalmanFilter::update(char sensor_type) {

    // calculate the measurement prediction
    Eigen::VectorXd z_pred = tools.cartesian_to_polar(x_, sensor_type);
    Eigen::VectorXd y = z_ - z_pred;

    // normalize the angle between -pi to pi
    while(y(1) > M_PI){
        y(1) -= 2*M_PI;
    }

    while(y(1) < -M_PI){
        y(1) += 2*M_PI;
    }
    // compute the Kalman Gain
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S  = (H_ * P_ * Ht) + R_;
    Eigen::MatrixXd Kn = P_ * Ht * S.inverse();
    
    // calculate the state update
    x_ = x_ + Kn*y;

    // calculate the covariance update
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - Kn * H_) * P_;

}

/** 
 * @brief :
 * @params:
 * @return:
 */
void ExtendedKalmanFilter::process_sensor_data(data_val data) {

    int sensor_id;

    sensor_id = data.sensor_number;

    //std::cout << "Sensor type " << data.sensor_type <<std::endl;

    // read first measurement and provide initial estimate
    if (!initialize_state){

        // if sensor is Laser
        if (data.sensor_type == 'L') {

            // read sensor data and store in x_
            x_ << data.lidar_meas_px, data.lidar_meas_py, 0.0, 0.0;
            

        }
        // if sensor is radar
        else if (data.sensor_type == 'R') {

            x_ << (data.radar_meas_rho)*cos(data.radar_meas_phi), (data.radar_meas_rho)*sin(data.radar_meas_phi), 0.0, 0.0;

        }

        // store the timestamp for the dt calucations
        prev_timestamp_ = data.timestamp;

        // set the initialize_state flag to true as the flag is now initialized
        initialize_state = true;

        //std::cout << "First state x_ : " << x_;
        return;
    }

    // caluclate dt
    double dt = (data.timestamp - prev_timestamp_)/1000000.0;
    prev_timestamp_ = data.timestamp;
    std::cout<<"dt : " << dt <<std::endl;
    // compute state transition matrix F
    F_ = compute_state_transition_F(dt);
    //std::cout << "Computed State Transition Matrix" << std::endl << F_ <<std::endl;
    // compute the control matrix G
    G_ = compute_control_matrix_G(dt);
    //std::cout << "Computed control Matrix" << std::endl << G_ <<std::endl;
    // compute the observation matrix H_
    H_ = compute_observation_matrix_H(sensor_id, req_jacobian_calc_s1, req_jacobian_calc_s2);
    //std::cout << "Computed observation Matrix" << std::endl << H_ <<std::endl;;
    // compute the process noise covariance matrix Q_
    Q_ = compute_process_noise_Q(dt);
    //std::cout << "Computed process noise covariance matrix" << std::endl << Q_ <<std::endl;;
    // set the control vector u_
    u_ = set_control_vector(data);
    //std::cout << "Computed control vector" << std::endl << u_ <<std::endl;;
    // set the measurement vector z_
    z_ = set_measurement_vector(data);
    //std::cout << "Computed measurement vector" << std::endl << z_ <<std::endl;;
    // set the measurement noise matrix R_
    R_ = set_measurement_noise_R(sensor_id);
    //std::cout << "Computed measurement noise" << std::endl << R_ <<std::endl;;
    // perform prediction
    predict();
    //std::cout << "finished prediction" << std::endl;
    // perform update
    if (data.sensor_type == 'R')
        update(data.sensor_type);
    else
        update();

}

/**
 * @brief : 
 * @params:
 * @return:
 */
Eigen::VectorXd ExtendedKalmanFilter::access_system_state() {

    // return the state vector
    return x_;
} 