#include <iostream>
#include "file_parse_util.h"
#include <ekf.h>


int main(int argc, char *argv[]){

    std::string file_name = argv[1];
    std::string config_file = argv[2];
    std::cout<<"Reading file ...."<< std::endl;
    std::cout<<"Reading config file ...."<< std::endl;
    
    Data file_data;
    ExtendedKalmanFilter ekf(config_file);
    file_data.parse_file(file_name.c_str());
    //file_data.display_data_records();

    // read in the config file for the Matrices

    // Create an EKF object and process EKF measurements

    // Store the estimates in a file 

    // append the ground truth in a file

    // append the measurements in a file

    // plot the graphs and display the statistics

    return 0;
}