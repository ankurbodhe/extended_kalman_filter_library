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
            u_ = Eigen::VectorXd::Constant(4, 0);

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
    else if (config["Sensor_1"].as<std::string>() == "Laser") {

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


    }
    else if (config["Sensor_1"].as<std::string>() == "Radar") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Constant(4, 0.0);

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


    }
    else if (config["Sensor_2"].as<std::string>() == "Radar") {

        // if the system model is constant acceleration
        if(config["System_Model"].as<std::string>() == "const_accel") {

            // Set the control matrix G to 0
            G_ = Eigen::MatrixXd::Constant(4, 2, 0.0);
            
            // Set the control vector u to 0
            u_ = Eigen::VectorXd::Constant(4, 0.0);

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
            R2_ << 0.09, 0, 0,
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


/* @brief :
 * @params:
 * @return:
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_state_transition_F(double t) {

    Eigen::MatrixXd a(4,2);

    return a;

}


/* @brief :
 * @params:
 * @return:
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_process_noise_Q(double t) {

    Eigen::MatrixXd a(4,2);

    return a;

}


/* @brief :
 * @params:
 * @return:
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_control_matrix_G(double t) {

    Eigen::MatrixXd a(4,2);

    return a;

}


/* @brief :
 * @params:
 * @return:
 */
Eigen::MatrixXd ExtendedKalmanFilter::compute_observation_matrix_H() {

    Eigen::MatrixXd a(4,2);

    return a;
}



/* @brief : This function performs the State prediction and the estimate uncertainty prediction
 * @params: 
 * @return:
 */
void ExtendedKalmanFilter::predict() {

    // compute the state prediction
    x_ = F_*x_ + G_*u_;

    // compute estimate uncertainty covariance
    P_ = F_*P_*(F_.transpose()) + Q_;

}

/* @brief : The function perform the state update, the estimate uncertainty update based on the sensor measurement
 * @params: 
 * @return:
 */
void ExtendedKalmanFilter::update() {


    // calculate the measurement prediction
    
    

    // compute the Kalman Gain
    

    // calculate the measurement vector z_n based on the sensor measurement

    // calculate the 




}



/* @brief :
 * @params:
 * @return:
 */
void ExtendedKalmanFilter::process_sensor_data(data_val data) {

    int sensor_id;

    sensor_id = data.sensor_number;


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

        return;
    }

    // caluclate dt
    double dt = (data.timestamp - prev_timestamp_)/1000000.0;
    prev_timestamp_ = data.timestamp;

    // compute state transition matrix F
    F_ = compute_state_transition_F(dt);

    // compute the control matrix G
    G_ = compute_control_matrix_G(dt);

    // compute the observation matrix H_
    H_ = compute_observation_matrix_H();

    // compute the process noise covariance matrix Q_
    Q_ = compute_process_noise_Q(dt);

    // set the control vector u_
    u_ = set_control_vector(data);

    // set the measurement vector z_
    z_ = set_measurement_vector(data);

    // set the measurement noise matrix R_
    R_ = set_measure

    // perform prediction
    predict();

    // perform update
    update();

}

/* @brief : 
 * @params:
 * @return:
 */
Eigen::VectorXd ExtendedKalmanFilter::access_system_state() {

    // return the state vector
    return x_;
} 