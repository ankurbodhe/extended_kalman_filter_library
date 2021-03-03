#include <iostream>
#include "file_parse_util.h"
#include <ekf.h>
#include <fstream>

int main(int argc, char *argv[]){

    std::string file_name = argv[1];
    std::string config_file = argv[2];
    std::string output_file = argv[3];
    std::cout<<"Reading file ...."<< std::endl;
    std::cout<<"Reading config file ...."<< std::endl;
    
    Data file_data;
    data_val dat;
    Eigen::VectorXd estimate;
    ofstream output(output_file.c_str(), ofstream::out);

    // read in the config file for the Matrices
    ExtendedKalmanFilter ekf(config_file);
    file_data.parse_file(file_name.c_str());
    
    //file_data.display_data_records();

    std::cout << "------------- System states ---------------" << std::endl;
    //std::cout << "Record size : " << file_data.data_record_size() << std::endl;
    // Create an EKF object and process EKF measurements
    for (int i = 0; i < file_data.data_record_size(); i++)
    {
        dat = file_data.access_data_record(i);
        ekf.process_sensor_data(dat);
        estimate = ekf.access_system_state();
        cout << "Seq : " << i <<" estimate | "<< estimate(0) << " " << estimate(1) << " " << estimate(2) << " " << estimate(3) << std::endl;
        cout << "        " <<" gt       | " <<dat.gt_px << " " << dat.gt_py << " " << dat.gt_vx << " " << dat.gt_vy << std::endl;
        // Store the estimates in a file 
        output << i << ";" << estimate(0) << ";" << estimate(1) << ";" << estimate(2) << ";" << estimate(3) <<";";
        output << dat.gt_px << ";" << dat.gt_py << ";" << dat.gt_vx << ";" << dat.gt_vy << ";";
        if (dat.sensor_type == 'L'){
            output << dat.lidar_meas_px << ";" << dat.lidar_meas_py << "\n";
        } else if (dat.sensor_type == 'R') {
            double x = (dat.radar_meas_rho)*cos(dat.radar_meas_phi);
            double y = (dat.radar_meas_rho)*sin(dat.radar_meas_phi);
            output << x << ";" << y << "\n";
        }
    }

    if (output.is_open())
        output.close();

    // plot the graphs and display the statistics

    return 0;
}